/*
 * capture - the standalone, button-driven ride capture.
 *
 * One task owns the whole state machine. Every entry point in capture.h only
 * validates what it can validate cheaply and then posts a command; the task
 * pops it, drives NimBLE and publishes a status block the UI can copy at 5 Hz.
 * The NimBLE GAP/GATT callbacks run on the host task and are kept to the two
 * things that cannot deadlock: updating counters under a spinlock and posting
 * an event with a zero timeout.
 *
 * Both peers were measured, see README.md and docs/capture.md:
 *
 *   Fardriver YuanQuFOC274 - service 0xffe0, data characteristic 0xffec.
 *     Writing its CCCD is the entire protocol: 16 byte frames then arrive
 *     unsolicited at ~35.5 Hz and nothing is ever sent to the controller. Its
 *     BLE address rotates between sessions, so it is matched by the name
 *     substring "Yuan" or by the advertised service UUID 0xffe0, never by MAC,
 *     and a reconnect always starts with a fresh scan.
 *
 *   Daly BMS CH10250DD0097 - service 0xfff0, notify 0xfff1, control 0xfff2.
 *     Its address is stable (40:18:03:01:20:e9, random). It advertises neither
 *     0xfff0 nor "DL" as its name - the complete local name in the scan
 *     response is "CH10250DD0097" and the "DL" tag sits at the end of the
 *     manufacturer payload - so it is matched on the address, on a name
 *     containing "DL" and on that manufacturer tag.
 *
 *     Unlike the Fardriver it is strict request/response. Its BLE module is a
 *     UART bridge that never sends anything unsolicited, so a link that only
 *     subscribes stays silent for the whole ride. One Modbus request written
 *     to 0xfff2 produces exactly one 129 byte answer on 0xfff1, and the ATT
 *     MTU has to have been raised first or the peer truncates that answer at
 *     20 bytes and drops the remainder rather than fragmenting it - which is
 *     why the poll below exists and why a failed exchange is fatal for it.
 *
 * Handles are discovered by UUID rather than taken from the capture. They have
 * been stable in practice (0x0010/0x0011 on the Fardriver), but a firmware
 * update on the bike would silently break a hardcoded value and nobody is
 * watching the console while the capture runs.
 */
#include "capture.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "ble_explorer.h"
#include "board.h"
#include "capture_store.h"
#include "daly.h"
#include "imu.h"
#include "rtc_bm8563.h"

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_id.h"
#include "host/ble_uuid.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "os/os_mbuf.h"

static const char *TAG = "cap";

/* ------------------------------------------------------------------ tuning */

#define CAP_TASK_STACK      6144
#define CAP_TASK_PRIO       6
#define CAP_QUEUE_LEN       16

#define CAP_TICK_MS         200     /* how often the task does periodic work */
#define CAP_TELEM_MS        5000    /* WFREC_TELEM interval */
#define CAP_SPACE_MS        30000   /* how often the free space is checked */
#define CAP_STALE_MS        10000   /* connected+subscribed but silent = dead */
#define CAP_BATT_MIN_MV     3400    /* below this the capture closes its file */
#define CAP_SPACE_MIN       (64 * 1024)  /* stop before FAT runs out entirely */

#define CAP_CONNECT_MS      8000
#define CAP_RESCAN_MS       8000
#define CAP_GATT_MS         10000
#define CAP_MAX_CHRS        16      /* characteristics per discovered service */
#define CAP_MAX_NOTIFY      255     /* WFREC payload length is a uint8_t */

#define UUID_CCCD           0x2902

/* 1 s, then 2 s, then 5 s for every further attempt. Fast enough that a
 * momentary dropout costs almost no frames, slow enough that an out-of-range
 * peer does not keep the radio busy for the link that is still delivering. */
static const uint32_t k_backoff_ms[] = { 1000, 2000, 5000 };

/* ------------------------------------------------------------- the targets */

/* One request per second. Fast enough that a throttle burst still shows up in
 * the pack current and the sag it causes, slow enough to leave the air to the
 * Fardriver's 35.5 Hz stream on the other link. */
#define CAP_BMS_POLL_MS     1000

/* What the poll asks for: the 62 register block at 0 that carries the 28 cell
 * voltages, the pack voltage, current, SoC and the temperatures. Only the 0xd2
 * protocol variant answers on this unit; the 0x81 one is silent. */
#define CAP_BMS_POLL_ADDR   0x0000
#define CAP_BMS_POLL_COUNT  0x003e

/* A polled link, or all zeroes for one that is only ever pushed to. */
typedef struct {
    uint8_t  start;           /* protocol variant, the Modbus slave address slot */
    uint16_t address;         /* first holding register */
    uint16_t count;           /* how many of them */
    uint32_t interval_ms;     /* 0 means this link is never polled */
} cap_poll_t;

typedef struct {
    const char *label;        /* short name used in log lines and events */
    const char *name_substr;  /* advertised name match */
    uint16_t    adv_uuid;     /* 16-bit service UUID in the advertisement, 0=none */
    uint16_t    svc_uuid;     /* service to discover after connecting */
    uint16_t    chr_uuid;     /* notify characteristic inside it */
    uint16_t    ctrl_uuid;    /* characteristic the poll is written to, 0=none */
    cap_poll_t  poll;         /* what to ask for and how often */
    uint8_t     rec_type;     /* WFREC_* written for its notifications */
    bool        rescan_first; /* address rotates, so scan before reconnecting */
} cap_target_t;

static const cap_target_t s_target[CAP_LINK_COUNT] = {
    [CAP_LINK_MCU] = {
        .label = "mcu", .name_substr = "Yuan", .adv_uuid = 0xffe0,
        .svc_uuid = 0xffe0, .chr_uuid = 0xffec,
        /* Pure push: nothing is ever sent to the Fardriver, so it declares
         * neither a control characteristic nor a poll. */
        .rec_type = WFREC_MCU, .rescan_first = true,
    },
    [CAP_LINK_BMS] = {
        .label = "bms", .name_substr = "DL", .adv_uuid = 0x0000,
        .svc_uuid = 0xfff0, .chr_uuid = 0xfff1, .ctrl_uuid = 0xfff2,
        .poll = { .start = DALY_START_D2, .address = CAP_BMS_POLL_ADDR,
                  .count = CAP_BMS_POLL_COUNT, .interval_ms = CAP_BMS_POLL_MS },
        .rec_type = WFREC_BMS, .rescan_first = false,
    },
};

static bool target_polled(int idx)
{
    return s_target[idx].poll.interval_ms != 0;
}

/* NimBLE keeps addresses LSB first, so 40:18:03:01:20:e9 reads backwards. */
static const ble_addr_t s_bms_addr = {
    .type = BLE_ADDR_RANDOM,
    .val = { 0xe9, 0x20, 0x01, 0x03, 0x18, 0x40 },
};

/* --------------------------------------------------------------- task mail */

typedef enum {
    MSG_SCAN_START = 0,
    MSG_SCAN_STOP,
    MSG_REC_START,
    MSG_REC_STOP,
    MSG_DISCONNECT,     /* from the GAP callback */
    MSG_DISC_COMPLETE,  /* from the GAP callback */
    MSG_SHUTDOWN,
} cap_msg_type_t;

typedef struct {
    uint8_t type;
    uint8_t link;
    int     reason;
} cap_msg_t;

/* ------------------------------------------------------------------ state */

/* Everything the capture task owns privately, plus the few fields the NimBLE
 * host task touches; those are marked and guarded by s_mux. */
typedef struct {
    ble_addr_t addr;          /* guarded: written by the scan callback */
    bool       addr_valid;    /* guarded */
    uint16_t   conn_handle;   /* guarded: cleared by the disconnect callback */
    uint16_t   svc_start;
    uint16_t   svc_end;
    uint16_t   val_handle;
    uint16_t   cccd_handle;
    uint16_t   ctrl_handle;   /* where the poll goes, 0 on a push-only link */
    uint32_t   last_rx_ms;    /* guarded: uptime of the last notification */
    bool       ever_rx;       /* guarded: this link has notified at least once */
    bool       want;          /* the running capture wants this link up */
    uint32_t   retry_at_ms;   /* uptime at which the next reconnect is due */
    uint32_t   poll_at_ms;    /* uptime at which the next request is due */
    uint8_t    backoff_idx;
} cap_link_ctx_t;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static cap_status_t   s_pub;              /* guarded by s_mux */
static cap_link_ctx_t s_link[CAP_LINK_COUNT];
static wf_ctrl_live_t s_live;             /* guarded by s_mux, see cap_live_get() */

static QueueHandle_t     s_queue;
static SemaphoreHandle_t s_op_sem;        /* one GATT op in flight, on our task */
static SemaphoreHandle_t s_down_sem;      /* cap_ble_shutdown() completion */
static TaskHandle_t      s_task;

static volatile int      s_op_rc;
static volatile uint16_t s_op_conn_handle;
static volatile bool     s_inited;
static volatile bool     s_scanning;      /* our own discovery is running */
static volatile bool     s_rec_on;        /* notifications go to the store */
static volatile bool     s_ble_down;
/* Set while a stop or a shutdown is queued, so the long connect and rescan
 * waits inside the task give up instead of making the caller wait half a
 * minute for a link that is out of range anyway. */
static volatile bool     s_abort;
static int64_t           s_t0_us;         /* esp_timer at t_ms = 0 */
static uint8_t           s_own_addr_type = BLE_OWN_ADDR_PUBLIC;

/* Scratch for the one GATT discovery that can be in flight, owned by the
 * capture task. */
typedef struct {
    uint16_t       def_handle;
    uint16_t       val_handle;
    uint16_t       end_handle;
    uint8_t        properties;
    ble_uuid_any_t uuid;
} cap_chr_t;

static cap_chr_t s_chr[CAP_MAX_CHRS];
static int       s_chr_n;
static uint16_t  s_disc_svc_start, s_disc_svc_end;
static bool      s_disc_svc_found;
static uint16_t  s_disc_cccd;

/* ------------------------------------------------------------------ utils */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Milliseconds since the start of the capture. s_t0_us is written while
 * s_rec_on is false and published by the spinlock that follows it, so a reader
 * that has seen s_rec_on == true also sees the final t0. It never moves after
 * that, which is what keeps t_ms monotonic. */
static uint32_t cap_t_ms(void)
{
    int64_t t0 = s_t0_us;
    int64_t now = esp_timer_get_time();
    if (t0 <= 0 || now <= t0) {
        return 0;
    }
    return (uint32_t)((now - t0) / 1000);
}

static void copy_str(char *dst, size_t cap, const char *src)
{
    if (cap == 0) {
        return;
    }
    size_t n = strlen(src);
    if (n > cap - 1) {
        n = cap - 1;
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static void set_state(cap_state_t st)
{
    portENTER_CRITICAL(&s_mux);
    s_pub.state = st;
    portEXIT_CRITICAL(&s_mux);
    ESP_LOGI(TAG, "state=%s", cap_state_str(st));
}

static void set_err(const char *fmt, ...)
{
    char buf[sizeof(s_pub.err)] = {0};
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    portENTER_CRITICAL(&s_mux);
    memcpy(s_pub.err, buf, sizeof(buf));
    portEXIT_CRITICAL(&s_mux);
    ESP_LOGW(TAG, "err: %s", buf);
}

static void clear_err(void)
{
    portENTER_CRITICAL(&s_mux);
    s_pub.err[0] = '\0';
    portEXIT_CRITICAL(&s_mux);
}

/* One ASCII WFREC_EVENT line. Only ever called from the capture task, so the
 * stack buffer is safe; the payload length field is a uint8_t. */
static void cap_event(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if (n > (int)sizeof(buf) - 1) {
        n = (int)sizeof(buf) - 1;
    }
    ESP_LOGI(TAG, "event: %s", buf);
    if (s_rec_on) {
        store_write(WFREC_EVENT, cap_t_ms(), buf, (uint8_t)n);
    }
}

/* Posts from any context, including the NimBLE host task, and never blocks. */
static void post_msg(uint8_t type, uint8_t link, int reason)
{
    if (s_queue == NULL) {
        return;
    }
    cap_msg_t m = { .type = type, .link = link, .reason = reason };
    if (xQueueSend(s_queue, &m, 0) != pdTRUE) {
        ESP_LOGW(TAG, "command queue full, dropped msg=%u", type);
    }
}

/* ------------------------------------------------------- controller decode
 *
 * What the bytes mean lives in wfdecode, which is pure C99 and knows nothing
 * of this file: the same functions run here and in the offline replay of a
 * recorded Capture, so a decoding claim can be checked without a bike (see
 * ADR-0002 and tests/host/replay.c). What stays here is the two things
 * wfdecode must not own - the lock, and the live state it guards.
 *
 * Validation deliberately happens outside the critical section: the CRC is 14
 * bytes of bit-shifting, and it is the frames that fail it which are the least
 * worth holding a spinlock for. A frame that does not validate is discarded
 * silently, same as a type wfdecode's table does not cover.
 *
 * Runs off every MCU-link notification independent of s_rec_on - see
 * notify_sink() - so the live screen works whenever the link is up, capture
 * running or not.
 */
static void controller_decode(const uint8_t *data, uint16_t len)
{
    wf_ctrl_frame_t frame;
    if (!wf_ctrl_frame_parse(data, len, &frame)) {
        return;
    }
    portENTER_CRITICAL(&s_mux);
    wf_ctrl_apply(&s_live, &frame);
    portEXIT_CRITICAL(&s_mux);
}

/* ------------------------------------------------------ notification sink */

static void notify_sink(int idx, const uint8_t *data, uint16_t len)
{
    if (len > CAP_MAX_NOTIFY) {
        len = CAP_MAX_NOTIFY;
    }

    if (idx == CAP_LINK_MCU) {
        controller_decode(data, len);
    }

    /* Liveness is tracked even before the file is open, so the stale watchdog
     * has a valid baseline the moment recording starts. */
    uint32_t uptime = now_ms();

    if (!s_rec_on) {
        portENTER_CRITICAL(&s_mux);
        s_link[idx].last_rx_ms = uptime;
        s_link[idx].ever_rx = true;
        portEXIT_CRITICAL(&s_mux);
        return;
    }

    uint32_t t = cap_t_ms();
    store_write(s_target[idx].rec_type, t, data, (uint8_t)len);

    portENTER_CRITICAL(&s_mux);
    s_link[idx].last_rx_ms = uptime;
    s_link[idx].ever_rx = true;
    s_pub.link[idx].frames++;
    s_pub.link[idx].bytes += len;
    s_pub.link[idx].last_frame_ms = t;
    s_pub.frames++;
    portEXIT_CRITICAL(&s_mux);
}

/* ------------------------------------------------------------ GAP handlers */

/* True when this advertising report belongs to target idx. Runs inside the
 * s_mux critical section, so it must stay short: a name compare over at most
 * 24 bytes and a walk of the advertised 16-bit UUID list. */
static bool link_matches(int idx, const struct ble_gap_disc_desc *d,
                         const struct ble_hs_adv_fields *f, const char *name)
{
    if (idx == CAP_LINK_BMS) {
        /* The measured Daly address always wins, and once it has been found
         * nothing weaker may take its place: "DL" is short enough to appear
         * in an unrelated neighbour's name. */
        if (ble_addr_cmp(&s_bms_addr, &d->addr) == 0) {
            return true;
        }
        if (s_link[idx].addr_valid &&
            ble_addr_cmp(&s_link[idx].addr, &s_bms_addr) == 0) {
            return false;
        }
    }
    /* An address we have already locked on to: keeps RSSI and the scan
     * response (which is where the Daly's name arrives) attached to it. */
    if (s_link[idx].addr_valid && ble_addr_cmp(&s_link[idx].addr, &d->addr) == 0) {
        return true;
    }
    if (name[0] != '\0' && strstr(name, s_target[idx].name_substr) != NULL) {
        return true;
    }
    if (s_target[idx].adv_uuid != 0) {
        for (int i = 0; i < f->num_uuids16; i++) {
            if (f->uuids16[i].value == s_target[idx].adv_uuid) {
                return true;
            }
        }
    }
    if (idx == CAP_LINK_BMS) {
        /* The Daly advertises neither 0xfff0 nor a name containing "DL" - its
         * complete local name is CH10250DD0097 - but its advertisement is
         * 0201060cff 0203 16 <address> 444c: company 0x0302 with a trailing
         * "DL" tag. That is the last resort if the address ever changes. */
        if (f->mfg_data != NULL && f->mfg_data_len >= 4 &&
            f->mfg_data[0] == 0x02 && f->mfg_data[1] == 0x03 &&
            f->mfg_data[f->mfg_data_len - 2] == 'D' &&
            f->mfg_data[f->mfg_data_len - 1] == 'L') {
            return true;
        }
    }
    return false;
}

static void scan_report(const struct ble_gap_disc_desc *d)
{
    struct ble_hs_adv_fields f;
    if (ble_hs_adv_parse_fields(&f, d->data, d->length_data) != 0) {
        return;
    }

    /* Everything expensive happens before the lock is taken. */
    char name[sizeof(s_pub.link[0].name)] = "";
    if (f.name != NULL && f.name_len > 0) {
        size_t n = f.name_len;
        if (n > sizeof(name) - 1) {
            n = sizeof(name) - 1;
        }
        memcpy(name, f.name, n);
        name[n] = '\0';
    }
    char addr[18];
    blex_addr_str(&d->addr, addr);

    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        if (!link_matches(i, d, &f, name)) {
            continue;
        }
        s_link[i].addr = d->addr;
        s_link[i].addr_valid = true;
        s_pub.link[i].seen = true;
        s_pub.link[i].rssi = d->rssi;
        memcpy(s_pub.link[i].addr, addr, sizeof(s_pub.link[i].addr));
        if (name[0] != '\0') {
            memcpy(s_pub.link[i].name, name, sizeof(name));
        }
    }
    portEXIT_CRITICAL(&s_mux);
}

static int on_disc_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        scan_report(&event->disc);
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scanning = false;
        post_msg(MSG_DISC_COMPLETE, 0, event->disc_complete.reason);
        return 0;

    default:
        return 0;
    }
}

/* One handler for both links; cb_arg carries which one. All it may do is set
 * the op result, update counters and post - the capture task does the work. */
static int on_link_event(struct ble_gap_event *event, void *arg)
{
    int idx = (int)(intptr_t)arg;
    if (idx < 0 || idx >= CAP_LINK_COUNT) {
        return 0;
    }

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        s_op_rc = event->connect.status;
        s_op_conn_handle = (event->connect.status == 0)
                               ? event->connect.conn_handle
                               : BLE_HS_CONN_HANDLE_NONE;
        xSemaphoreGive(s_op_sem);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        portENTER_CRITICAL(&s_mux);
        s_link[idx].conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_pub.link[idx].connected = false;
        s_pub.link[idx].subscribed = false;
        portEXIT_CRITICAL(&s_mux);
        post_msg(MSG_DISCONNECT, (uint8_t)idx, event->disconnect.reason);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        /* Only the NimBLE host task delivers this, so one static buffer is
         * enough and keeps 255 bytes off that task's stack. */
        static uint8_t buf[CAP_MAX_NOTIFY];
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        if (os_mbuf_copydata(event->notify_rx.om, 0, len, buf) == 0) {
            notify_sink(idx, buf, len);
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        return 0;   /* accept whatever the peer prefers */

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* Nothing is bonded on purpose; let the peer start over. */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        return 0;
    }
}

/* ------------------------------------------------------------ GATT helpers */

static int wait_op(int timeout_ms)
{
    if (xSemaphoreTake(s_op_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return BLE_HS_ETIMEOUT;
    }
    return s_op_rc;
}

static int on_mtu(uint16_t conn_handle, const struct ble_gatt_error *error,
                  uint16_t mtu, void *arg)
{
    (void)conn_handle;
    (void)mtu;
    (void)arg;
    s_op_rc = error->status;
    xSemaphoreGive(s_op_sem);
    return 0;
}

static int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg)
{
    (void)conn_handle;
    (void)arg;

    if (error->status == 0 && service != NULL) {
        if (!s_disc_svc_found) {
            s_disc_svc_start = service->start_handle;
            s_disc_svc_end = service->end_handle;
            s_disc_svc_found = true;
        }
        return 0;
    }
    s_op_rc = (error->status == BLE_HS_EDONE) ? 0 : error->status;
    xSemaphoreGive(s_op_sem);
    return 0;
}

static int on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)conn_handle;
    (void)arg;

    if (error->status == 0 && chr != NULL) {
        if (s_chr_n < CAP_MAX_CHRS) {
            cap_chr_t *c = &s_chr[s_chr_n++];
            c->def_handle = chr->def_handle;
            c->val_handle = chr->val_handle;
            c->end_handle = 0;
            c->properties = chr->properties;
            c->uuid = chr->uuid;
        }
        return 0;
    }
    s_op_rc = (error->status == BLE_HS_EDONE) ? 0 : error->status;
    xSemaphoreGive(s_op_sem);
    return 0;
}

static int on_disc_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                       void *arg)
{
    (void)conn_handle;
    (void)chr_val_handle;
    (void)arg;

    if (error->status == 0 && dsc != NULL) {
        if (s_disc_cccd == 0 &&
            blex_uuid_is16(&dsc->uuid.u, UUID_CCCD)) {
            s_disc_cccd = dsc->handle;
        }
        return 0;
    }
    s_op_rc = (error->status == BLE_HS_EDONE) ? 0 : error->status;
    xSemaphoreGive(s_op_sem);
    return 0;
}

static int on_write(uint16_t conn_handle, const struct ble_gatt_error *error,
                    struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;
    s_op_rc = error->status;
    xSemaphoreGive(s_op_sem);
    return 0;
}

/* ------------------------------------------------------------- link set-up */

static uint16_t link_conn_handle(int idx)
{
    uint16_t h;
    portENTER_CRITICAL(&s_mux);
    h = s_link[idx].conn_handle;
    portEXIT_CRITICAL(&s_mux);
    return h;
}

static bool link_connected(int idx)
{
    return link_conn_handle(idx) != BLE_HS_CONN_HANDLE_NONE;
}

/* Discovers the target service, its characteristics, the notify
 * characteristic's CCCD, and subscribes. Every step is by UUID. */
static int link_setup(int idx)
{
    const cap_target_t *t = &s_target[idx];
    uint16_t conn = link_conn_handle(idx);
    if (conn == BLE_HS_CONN_HANDLE_NONE) {
        return BLE_HS_ENOTCONN;
    }

    /* The MTU exchange is not required for the Fardriver's 16 byte frames, but
     * it removes MTU as a variable and matches what the host-driven capture
     * did. On a polled link it is load bearing: the answer is 129 bytes and a
     * peer that has not agreed a larger MTU truncates it to what fits and
     * throws the rest away, so the link would come up, look healthy and record
     * nothing usable for the whole ride. Fail it instead and let the reconnect
     * path try again. */
    xSemaphoreTake(s_op_sem, 0);
    int rc = ble_gattc_exchange_mtu(conn, on_mtu, NULL);
    if (rc == 0) {
        rc = wait_op(CAP_GATT_MS);
    }
    if (target_polled(idx)) {
        if (rc != 0) {
            ESP_LOGE(TAG, "%s mtu exchange rc=%d, cannot poll", t->label, rc);
            return rc;
        }
        /* Three bytes of the MTU go to the ATT notification header. */
        uint16_t need = (uint16_t)(daly_response_len(t->poll.count) + 3);
        uint16_t mtu = ble_att_mtu(conn);
        if (mtu < need) {
            ESP_LOGE(TAG, "%s mtu %u, needs %u for a %u byte response",
                     t->label, mtu, need, daly_response_len(t->poll.count));
            return BLE_HS_EMSGSIZE;
        }
    } else if (rc != 0) {
        ESP_LOGW(TAG, "%s mtu exchange rc=%d, continuing", t->label, rc);
    }

    s_disc_svc_found = false;
    s_disc_svc_start = 0;
    s_disc_svc_end = 0;
    xSemaphoreTake(s_op_sem, 0);
    rc = ble_gattc_disc_svc_by_uuid(conn, BLE_UUID16_DECLARE(t->svc_uuid),
                                    on_disc_svc, NULL);
    if (rc == 0) {
        rc = wait_op(CAP_GATT_MS);
    }
    if (rc != 0) {
        return rc;
    }
    if (!s_disc_svc_found) {
        ESP_LOGE(TAG, "%s has no service 0x%04x", t->label, t->svc_uuid);
        return BLE_HS_ENOENT;
    }

    /* Every characteristic of the service, so the notify characteristic's end
     * handle is known: a descriptor search bounded by the service end could
     * otherwise pick up a neighbour's CCCD. */
    s_chr_n = 0;
    xSemaphoreTake(s_op_sem, 0);
    rc = ble_gattc_disc_all_chrs(conn, s_disc_svc_start, s_disc_svc_end,
                                 on_disc_chr, NULL);
    if (rc == 0) {
        rc = wait_op(CAP_GATT_MS);
    }
    if (rc != 0) {
        return rc;
    }
    for (int c = 0; c < s_chr_n; c++) {
        s_chr[c].end_handle = (c + 1 < s_chr_n)
                                  ? (uint16_t)(s_chr[c + 1].def_handle - 1)
                                  : s_disc_svc_end;
    }

    int want = -1;
    int ctrl = -1;
    for (int c = 0; c < s_chr_n; c++) {
        if (want < 0 && blex_uuid_is16(&s_chr[c].uuid.u, t->chr_uuid)) {
            want = c;
        }
        if (ctrl < 0 && t->ctrl_uuid != 0 &&
            blex_uuid_is16(&s_chr[c].uuid.u, t->ctrl_uuid)) {
            ctrl = c;
        }
    }
    if (want < 0) {
        ESP_LOGE(TAG, "%s has no characteristic 0x%04x", t->label, t->chr_uuid);
        return BLE_HS_ENOENT;
    }
    /* No control characteristic on a polled target means no way to ask for
     * anything, and the link would be silent. Fail it here rather than record
     * an empty stream. */
    if (target_polled(idx) && ctrl < 0) {
        ESP_LOGE(TAG, "%s has no control characteristic 0x%04x", t->label,
                 t->ctrl_uuid);
        return BLE_HS_ENOENT;
    }
    if ((s_chr[want].properties & BLE_GATT_CHR_PROP_NOTIFY) == 0) {
        ESP_LOGW(TAG, "%s 0x%04x has props 0x%02x, no notify bit",
                 t->label, t->chr_uuid, s_chr[want].properties);
    }
    if (s_chr[want].val_handle >= s_chr[want].end_handle) {
        ESP_LOGE(TAG, "%s 0x%04x has no room for a CCCD", t->label, t->chr_uuid);
        return BLE_HS_ENOENT;
    }

    s_disc_cccd = 0;
    xSemaphoreTake(s_op_sem, 0);
    /* NimBLE adds the +1 itself when it builds the FIND_INFO request, so the
     * value handle is passed unchanged - see the same note in ble_explorer.c. */
    rc = ble_gattc_disc_all_dscs(conn, s_chr[want].val_handle,
                                 s_chr[want].end_handle, on_disc_dsc, NULL);
    if (rc == 0) {
        rc = wait_op(CAP_GATT_MS);
    }
    if (rc != 0) {
        return rc;
    }
    if (s_disc_cccd == 0) {
        ESP_LOGE(TAG, "%s 0x%04x has no CCCD", t->label, t->chr_uuid);
        return BLE_HS_ENOENT;
    }

    s_link[idx].svc_start = s_disc_svc_start;
    s_link[idx].svc_end = s_disc_svc_end;
    s_link[idx].val_handle = s_chr[want].val_handle;
    s_link[idx].cccd_handle = s_disc_cccd;
    s_link[idx].ctrl_handle = (ctrl >= 0) ? s_chr[ctrl].val_handle : 0;

    /* Writing this is the entire Fardriver protocol: nothing is ever sent to
     * the controller afterwards. On the Daly it only opens the path the polled
     * answers come back on. */
    static const uint8_t notify_on[2] = { 0x01, 0x00 };
    xSemaphoreTake(s_op_sem, 0);
    rc = ble_gattc_write_flat(conn, s_disc_cccd, notify_on, sizeof(notify_on),
                              on_write, NULL);
    if (rc == 0) {
        rc = wait_op(CAP_GATT_MS);
    }
    if (rc != 0) {
        return rc;
    }

    uint32_t uptime = now_ms();
    /* Only now is the poll allowed to run: the CCCD write above has completed,
     * so the answer to the first request has somewhere to arrive. Due at once,
     * so the first block of registers is in the file before the bike moves. */
    s_link[idx].poll_at_ms = uptime;
    portENTER_CRITICAL(&s_mux);
    s_pub.link[idx].subscribed = true;
    s_link[idx].last_rx_ms = uptime;   /* baseline for the stale watchdog */
    portEXIT_CRITICAL(&s_mux);

    cap_event("%s subscribed val=0x%04x cccd=0x%04x ctrl=0x%04x mtu=%u",
              t->label, s_link[idx].val_handle, s_disc_cccd,
              s_link[idx].ctrl_handle, ble_att_mtu(conn));
    return 0;
}

/* Writes one request to a polled link's control characteristic.
 *
 * Write-without-response, which is what the working bench run used and what
 * the characteristic's properties (0x0e) allow: the answer is the only
 * acknowledgement worth having, and a write response would cost a round trip
 * on a radio already carrying the Fardriver's stream. It also means there is
 * no callback to wait on, so the capture task never blocks in here.
 */
static void link_poll(int idx)
{
    const cap_target_t *t = &s_target[idx];
    uint16_t conn = link_conn_handle(idx);
    if (conn == BLE_HS_CONN_HANDLE_NONE || s_link[idx].ctrl_handle == 0) {
        return;
    }

    uint8_t frame[DALY_REQ_LEN];
    daly_build_request(frame, t->poll.start, DALY_FUNC_READ, t->poll.address,
                       t->poll.count);

    int rc = ble_gattc_write_no_rsp_flat(conn, s_link[idx].ctrl_handle, frame,
                                         sizeof(frame));
    if (rc != 0) {
        /* Nothing to recover: a lost request costs one sample, and a link that
         * keeps failing stops answering and is rebuilt by the stale watchdog. */
        ESP_LOGW(TAG, "%s poll rc=%d", t->label, rc);
    }
}

/* ---------------------------------------------------------------- scanning */

/* The NimBLE host resets itself when the controller reports an error, and it
 * comes back with no identity address: every GAP call then fails with
 * BLE_HS_ENOADDR until the host has re-synced and the address has been
 * inferred again. Observed on the bench as a scan that never recovered - which
 * on a ride would silently cost the rest of the capture. So every GAP entry
 * point re-arms through here instead of trusting the value cached at startup.
 */
static bool own_addr_refresh(void)
{
    if (!blex_ready()) {
        return false;
    }
    uint8_t type = BLE_OWN_ADDR_PUBLIC;
    if (ble_hs_id_infer_auto(0, &type) != 0) {
        return false;
    }
    s_own_addr_type = type;
    return true;
}

/* True for the two return codes that mean "the host is not usable yet", as
 * opposed to a genuine failure of the operation. */
static bool rc_is_unsynced(int rc)
{
    return rc == BLE_HS_ENOADDR || rc == BLE_HS_ENOTSYNCED;
}

static int scan_begin(int32_t duration_ms)
{
    /* Deliberately no duplicate filtering: the RSSI on the arming screen has
     * to stay live, and the Daly's name only ever arrives in a scan response. */
    struct ble_gap_disc_params p = {
        .itvl = 0x0060,           /* 60 ms interval */
        .window = 0x0050,         /* 50 ms window -> ~83 % duty cycle */
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,
        .filter_duplicates = 0,
    };

    if (s_scanning) {
        ble_gap_disc_cancel();
        s_scanning = false;
    }

    int rc = ble_gap_disc(s_own_addr_type, duration_ms, &p, on_disc_event, NULL);
    if (rc_is_unsynced(rc) && own_addr_refresh()) {
        ESP_LOGW(TAG, "host had lost its address, re-inferred type=%u, rescanning",
                 s_own_addr_type);
        rc = ble_gap_disc(s_own_addr_type, duration_ms, &p, on_disc_event, NULL);
    }
    if (rc != 0) {
        if (rc == BLE_HS_EALREADY) {
            set_err("ble_explorer is scanning");
        } else {
            set_err("scan failed rc=%d", rc);
        }
        return rc;
    }
    s_scanning = true;
    return 0;
}

static void scan_end(void)
{
    if (s_scanning) {
        /* A cancel can race the procedure finishing on its own; the return
         * value is only interesting in the log. */
        int rc = ble_gap_disc_cancel();
        if (rc != 0 && rc != BLE_HS_EALREADY) {
            ESP_LOGD(TAG, "disc cancel rc=%d", rc);
        }
        s_scanning = false;
    }
}

/* Scans until this target answers, or timeout_ms passes. The Fardriver's MAC
 * rotates between sessions, so a reconnect has to find it again rather than
 * reuse the address the capture started with. */
static int scan_for(int idx, int timeout_ms)
{
    portENTER_CRITICAL(&s_mux);
    s_link[idx].addr_valid = false;
    s_pub.link[idx].seen = false;
    portEXIT_CRITICAL(&s_mux);

    int rc = scan_begin(BLE_HS_FOREVER);
    if (rc != 0) {
        return rc;
    }

    bool found = false;
    for (int waited = 0; waited < timeout_ms && !found; waited += 100) {
        if (s_abort) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        portENTER_CRITICAL(&s_mux);
        found = s_link[idx].addr_valid;
        portEXIT_CRITICAL(&s_mux);
    }
    scan_end();
    return found ? 0 : BLE_HS_ETIMEOUT;
}

/* ------------------------------------------------------- connect / drop */

static int link_connect(int idx)
{
    ble_addr_t addr;
    bool have;
    portENTER_CRITICAL(&s_mux);
    have = s_link[idx].addr_valid;
    addr = s_link[idx].addr;
    portEXIT_CRITICAL(&s_mux);
    if (!have) {
        return BLE_HS_ENOENT;
    }

    /* NimBLE allows only one outstanding connect and refuses to start one
     * while a discovery is running, so the scan goes down first. */
    scan_end();

    /* Wide interval bounds so the peer's own preference wins; a long
     * supervision timeout keeps a chatty controller from dropping us. */
    struct ble_gap_conn_params params = {
        .scan_itvl = 0x0060,
        .scan_window = 0x0050,
        .itvl_min = 24,             /* 30 ms  */
        .itvl_max = 80,             /* 100 ms */
        .latency = 0,
        .supervision_timeout = 600, /* 6 s */
        .min_ce_len = 0,
        .max_ce_len = 0,
    };

    xSemaphoreTake(s_op_sem, 0);
    s_op_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    int rc = ble_gap_connect(s_own_addr_type, &addr, CAP_CONNECT_MS, &params,
                             on_link_event, (void *)(intptr_t)idx);
    if (rc_is_unsynced(rc) && own_addr_refresh()) {
        rc = ble_gap_connect(s_own_addr_type, &addr, CAP_CONNECT_MS, &params,
                             on_link_event, (void *)(intptr_t)idx);
    }
    if (rc != 0) {
        return rc;
    }
    rc = wait_op(CAP_CONNECT_MS + 2000);
    if (rc == BLE_HS_ETIMEOUT) {
        ble_gap_conn_cancel();
        /* The cancelled attempt still reports itself; drop that event so the
         * next wait_op() does not return immediately. If it raced us and did
         * connect, the link is terminated rather than leaked. */
        if (xSemaphoreTake(s_op_sem, pdMS_TO_TICKS(1000)) == pdTRUE &&
            s_op_rc == 0 && s_op_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(s_op_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return BLE_HS_ETIMEOUT;
    }
    if (rc != 0) {
        return rc;
    }

    char addr_str[18];
    blex_addr_str(&addr, addr_str);
    portENTER_CRITICAL(&s_mux);
    s_link[idx].conn_handle = s_op_conn_handle;
    s_pub.link[idx].connected = true;
    s_pub.link[idx].subscribed = false;
    memcpy(s_pub.link[idx].addr, addr_str, sizeof(s_pub.link[idx].addr));
    portEXIT_CRITICAL(&s_mux);
    return 0;
}

static void link_drop(int idx)
{
    uint16_t conn = link_conn_handle(idx);
    if (conn != BLE_HS_CONN_HANDLE_NONE) {
        int rc = ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0 && rc != BLE_HS_ENOTCONN) {
            ESP_LOGW(TAG, "%s terminate rc=%d", s_target[idx].label, rc);
        }
    }
    portENTER_CRITICAL(&s_mux);
    s_link[idx].conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_link[idx].val_handle = 0;
    s_link[idx].cccd_handle = 0;
    s_link[idx].ctrl_handle = 0;
    s_pub.link[idx].connected = false;
    s_pub.link[idx].subscribed = false;
    portEXIT_CRITICAL(&s_mux);
}

/* Connect and subscribe. rescan forces a fresh scan first. */
static int link_bring_up(int idx, bool rescan)
{
    const cap_target_t *t = &s_target[idx];
    bool have_addr;

    portENTER_CRITICAL(&s_mux);
    have_addr = s_link[idx].addr_valid;
    portEXIT_CRITICAL(&s_mux);

    if (rescan || !have_addr) {
        int rc = scan_for(idx, CAP_RESCAN_MS);
        if (rc != 0) {
            ESP_LOGW(TAG, "%s not seen in %d ms", t->label, CAP_RESCAN_MS);
            return rc;
        }
    }

    if (s_abort) {
        return BLE_HS_EAGAIN;
    }

    int rc = link_connect(idx);
    if (rc != 0) {
        ESP_LOGW(TAG, "%s connect rc=%d", t->label, rc);
        return rc;
    }

    rc = link_setup(idx);
    if (rc != 0) {
        ESP_LOGW(TAG, "%s setup rc=%d", t->label, rc);
        link_drop(idx);
        return rc;
    }
    return 0;
}

/* ------------------------------------------------------------- telemetry */

static int8_t link_rssi(int idx)
{
    uint16_t conn = link_conn_handle(idx);
    int8_t rssi = 0;    /* 0 means "unknown"; a real reading is negative */
    if (conn != BLE_HS_CONN_HANDLE_NONE) {
        if (ble_gap_conn_rssi(conn, &rssi) != 0) {
            rssi = 0;
        }
    }
    return rssi;
}

static void write_telem(void)
{
    wflog_telem_t t = {0};
    int mv = board_battery_mv();
    t.batt_mv = (mv > 0) ? (uint16_t)mv : 0;
    t.rssi_mcu = link_rssi(CAP_LINK_MCU);
    t.rssi_bms = link_rssi(CAP_LINK_BMS);
    t.dropped = store_dropped();

    portENTER_CRITICAL(&s_mux);
    t.frames_mcu = s_pub.link[CAP_LINK_MCU].frames;
    t.frames_bms = s_pub.link[CAP_LINK_BMS].frames;
    /* Keep the arming/recording screen showing the live link quality. */
    if (t.rssi_mcu != 0) {
        s_pub.link[CAP_LINK_MCU].rssi = t.rssi_mcu;
    }
    if (t.rssi_bms != 0) {
        s_pub.link[CAP_LINK_BMS].rssi = t.rssi_bms;
    }
    portEXIT_CRITICAL(&s_mux);

    store_write(WFREC_TELEM, cap_t_ms(), &t, sizeof(t));
}

/* --------------------------------------------------- start / stop a capture */

static void refresh_store_counters(void)
{
    uint32_t dropped = store_dropped();
    uint64_t bytes = store_bytes();
    portENTER_CRITICAL(&s_mux);
    s_pub.dropped = dropped;
    /* store_bytes() reports the *current* capture, so it reads back as 0 once
     * the file is closed; the summary screen keeps the last non-zero value. */
    if (bytes != 0) {
        s_pub.bytes = bytes;
    }
    portEXIT_CRITICAL(&s_mux);
}

static void do_record_stop(const char *reason)
{
    cap_state_t st;
    portENTER_CRITICAL(&s_mux);
    st = s_pub.state;
    portEXIT_CRITICAL(&s_mux);

    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        s_link[i].want = false;
    }

    if (st != CAP_RECORDING && st != CAP_CONNECTING) {
        /* "Safe to call in any state": drop anything still connected, but
         * leave a running scan alone - stopping a capture that is not running
         * must not disarm the rider. */
        for (int i = 0; i < CAP_LINK_COUNT; i++) {
            link_drop(i);
        }
        return;
    }

    set_state(CAP_STOPPING);

    if (s_rec_on) {
        cap_event("stop: %s", reason != NULL ? reason : "user");
        refresh_store_counters();
        portENTER_CRITICAL(&s_mux);
        s_pub.elapsed_ms = cap_t_ms();
        portEXIT_CRITICAL(&s_mux);
        s_rec_on = false;
        /* Stop the IMU task before the file is closed: it writes on its own
         * schedule and imu_log_stop() only returns once it has actually left
         * the loop, so no sample can reach a closed capture. */
        imu_log_stop();
        /* A notification could be inside store_write() right now; give it a
         * couple of ticks to leave before the file is closed. */
        vTaskDelay(pdMS_TO_TICKS(20));
        esp_err_t err = store_end();
        if (err != ESP_OK) {
            set_err("store_end: %s", esp_err_to_name(err));
        }
    }

    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        link_drop(i);
    }
    scan_end();
    refresh_store_counters();
    set_state(CAP_DONE);

    ESP_LOGI(TAG, "capture done seq=%d frames=%" PRIu32 " dropped=%" PRIu32
                  " bytes=%" PRIu64,
             s_pub.seq, s_pub.frames, s_pub.dropped, s_pub.bytes);
}

static void do_record_start(void)
{
    cap_state_t st;
    portENTER_CRITICAL(&s_mux);
    st = s_pub.state;
    portEXIT_CRITICAL(&s_mux);

    if (st != CAP_ARMED) {
        set_err("not armed (%s)", cap_state_str(st));
        return;
    }
    if (!store_ready()) {
        set_err("capture store not mounted");
        return;
    }

    clear_err();
    set_state(CAP_CONNECTING);
    scan_end();

    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        s_pub.link[i].frames = 0;
        s_pub.link[i].bytes = 0;
        s_pub.link[i].reconnects = 0;
        s_pub.link[i].last_frame_ms = 0;
    }
    s_pub.frames = 0;
    s_pub.dropped = 0;
    s_pub.bytes = 0;
    portEXIT_CRITICAL(&s_mux);

    int up = 0;
    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        s_link[i].backoff_idx = 0;
        s_link[i].retry_at_ms = now_ms();
        s_link[i].ever_rx = false;
        /* Sequentially: NimBLE allows only one outstanding connect. The
         * scan was running right up to this point, so both addresses are
         * fresh and no rescan is needed here. */
        if (link_bring_up(i, false) == 0) {
            s_link[i].want = true;
            up++;
        } else {
            /* Wanted anyway: the reconnect path keeps trying for the whole
             * ride, and half a capture beats none. */
            s_link[i].want = true;
            set_err("%s did not come up", s_target[i].label);
        }
    }

    if (up == 0) {
        set_err("neither link came up");
        for (int i = 0; i < CAP_LINK_COUNT; i++) {
            s_link[i].want = false;
            link_drop(i);
        }
        set_state(CAP_SCANNING);
        if (scan_begin(BLE_HS_FOREVER) != 0) {
            set_state(CAP_IDLE);
        }
        return;
    }

    int64_t unix_start = bm8563_unix();
    if (unix_start < 0) {
        unix_start = 0;         /* the RTC never had a valid time */
    }

    char note[32];
    snprintf(note, sizeof(note), "wildfire idf %s", esp_get_idf_version());

    uint32_t seq = 0;
    char name[sizeof(s_pub.file)] = "";
    esp_err_t err = store_begin(unix_start, note, &seq, name, sizeof(name));
    if (err != ESP_OK) {
        set_err("store_begin: %s", esp_err_to_name(err));
        for (int i = 0; i < CAP_LINK_COUNT; i++) {
            s_link[i].want = false;
            link_drop(i);
        }
        set_state(CAP_ERROR);
        return;
    }

    char mcu_addr[18], bms_addr[18];
    portENTER_CRITICAL(&s_mux);
    memcpy(mcu_addr, s_pub.link[CAP_LINK_MCU].addr, sizeof(mcu_addr));
    memcpy(bms_addr, s_pub.link[CAP_LINK_BMS].addr, sizeof(bms_addr));
    s_pub.seq = (int)seq;
    memcpy(s_pub.file, name, sizeof(name));
    portEXIT_CRITICAL(&s_mux);
    store_set_addrs(link_connected(CAP_LINK_MCU) ? mcu_addr : "",
                    link_connected(CAP_LINK_BMS) ? bms_addr : "");

    /* t0 is fixed before any frame can reach the store, so t_ms starts at 0
     * and can never run backwards. */
    s_t0_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_mux);
    s_rec_on = true;
    portEXIT_CRITICAL(&s_mux);

    /* The IMU rides on the same t0 as the frames. Without it the capture is
     * still valid, only harder to interpret: no independent movement signal to
     * correlate a candidate payload field against. */
    esp_err_t imu_err = imu_log_start(s_t0_us);
    if (imu_err != ESP_OK) {
        cap_event("imu logging unavailable: %s", esp_err_to_name(imu_err));
    }

    set_state(CAP_RECORDING);
    cap_event("capture start seq=%" PRIu32 " file=%s unix=%" PRId64 " batt=%d mv",
              seq, name, unix_start, board_battery_mv());
    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        if (link_connected(i)) {
            cap_event("%s connected addr=%s", s_target[i].label,
                      i == CAP_LINK_MCU ? mcu_addr : bms_addr);
        }
    }
}

/* ----------------------------------------------------------- periodic work */

static void tick_recording(void)
{
    static uint32_t s_next_telem_ms;
    static uint32_t s_next_space_ms;
    static int s_low_batt_hits;

    uint32_t now = now_ms();

    /* Stale link watchdog. A link that is connected and subscribed but has
     * stopped delivering is the failure mode that would ruin a whole ride, so
     * it is torn down and rebuilt rather than trusted. It only arms once the
     * link has notified at least once, which keeps a peer that answers nothing
     * at all - a BMS on a protocol variant it does not speak, say - from being
     * reconnected in a loop for the whole ride.
     *
     * CAP_STALE_MS still holds with a polled link in the picture: 10 s is 350
     * missing frames of the Fardriver's 35.5 Hz stream and ten unanswered
     * requests at the BMS's 1 Hz poll. Both are far past any plausible burst
     * of interference, and neither is long enough for a rebuild to cost more
     * than a couple of seconds of the capture. */
    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        bool armed, subscribed;
        uint32_t last;
        portENTER_CRITICAL(&s_mux);
        armed = s_link[i].ever_rx;
        subscribed = s_pub.link[i].subscribed;
        last = s_link[i].last_rx_ms;
        portEXIT_CRITICAL(&s_mux);

        if (!armed || !subscribed || !link_connected(i)) {
            continue;
        }
        if ((uint32_t)(now - last) < CAP_STALE_MS) {
            continue;
        }
        cap_event("%s stale, no frame for %" PRIu32 " ms, dropping link",
                  s_target[i].label, (uint32_t)(now - last));
        link_drop(i);
        s_link[i].retry_at_ms = now;   /* reconnect on the next tick */
    }

    /* Reconnect. Never gives up while recording. */
    for (int i = 0; i < CAP_LINK_COUNT && !s_abort; i++) {
        if (!s_link[i].want || link_connected(i)) {
            continue;
        }
        now = now_ms();
        if ((int32_t)(now - s_link[i].retry_at_ms) < 0) {
            continue;
        }
        /* The Fardriver's address rotates, so it always needs a fresh scan;
         * the Daly's is stable until it is not, so it gets one after a couple
         * of failures. */
        bool rescan = s_target[i].rescan_first || s_link[i].backoff_idx >= 2;
        int rc = link_bring_up(i, rescan);
        now = now_ms();
        if (rc == 0) {
            portENTER_CRITICAL(&s_mux);
            s_pub.link[i].reconnects++;
            portEXIT_CRITICAL(&s_mux);
            s_link[i].backoff_idx = 0;
            cap_event("%s reconnected (attempt %" PRIu32 ")", s_target[i].label,
                      s_pub.link[i].reconnects);
        } else {
            uint8_t b = s_link[i].backoff_idx;
            if (b >= sizeof(k_backoff_ms) / sizeof(k_backoff_ms[0])) {
                b = sizeof(k_backoff_ms) / sizeof(k_backoff_ms[0]) - 1;
            }
            s_link[i].retry_at_ms = now + k_backoff_ms[b];
            if (s_link[i].backoff_idx < 250) {
                s_link[i].backoff_idx++;
            }
        }
    }

    now = now_ms();

    /* Polled links. The Daly never speaks unsolicited, so the frame rate in
     * the capture is exactly this loop's rate; a deadline rather than a tick
     * counter, because the task also wakes early whenever a command arrives. */
    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        bool subscribed;
        portENTER_CRITICAL(&s_mux);
        subscribed = s_pub.link[i].subscribed;
        portEXIT_CRITICAL(&s_mux);

        if (!target_polled(i) || !subscribed || !link_connected(i)) {
            continue;
        }
        if ((int32_t)(now - s_link[i].poll_at_ms) < 0) {
            continue;
        }
        s_link[i].poll_at_ms = now + s_target[i].poll.interval_ms;
        link_poll(i);
    }

    if ((int32_t)(now - s_next_telem_ms) >= 0) {
        s_next_telem_ms = now + CAP_TELEM_MS;
        write_telem();
        refresh_store_counters();

        /* Low battery: close the file rather than let a brownout truncate it.
         * Two consecutive readings, because the radio makes a single one dip. */
        int mv = board_battery_mv();
        if (mv > 0 && mv < CAP_BATT_MIN_MV) {
            if (++s_low_batt_hits >= 2) {
                cap_event("battery low (%d mV), stopping", mv);
                s_low_batt_hits = 0;
                do_record_stop("battery low");
                return;
            }
        } else {
            s_low_batt_hits = 0;
        }
    }

    if ((int32_t)(now - s_next_space_ms) >= 0) {
        s_next_space_ms = now + CAP_SPACE_MS;
        uint64_t total = 0, freeb = 0;
        if (store_space(&total, &freeb) == ESP_OK && freeb < CAP_SPACE_MIN) {
            cap_event("storage full (%" PRIu64 " bytes free), stopping", freeb);
            do_record_stop("storage full");
            return;
        }
    }

    portENTER_CRITICAL(&s_mux);
    s_pub.elapsed_ms = cap_t_ms();
    portEXIT_CRITICAL(&s_mux);
}

static void tick_scanning(void)
{
    bool both;
    cap_state_t st;
    portENTER_CRITICAL(&s_mux);
    both = s_pub.link[CAP_LINK_MCU].seen && s_pub.link[CAP_LINK_BMS].seen;
    st = s_pub.state;
    portEXIT_CRITICAL(&s_mux);

    /* The scan keeps running in CAP_ARMED so the RSSI on the arming screen
     * stays live and a rotated Fardriver address is picked up before the
     * rider ever presses the button. */
    if (both && st == CAP_SCANNING) {
        set_state(CAP_ARMED);
    }
}

/* ------------------------------------------------------------ BLE shutdown */

static void do_ble_shutdown(void)
{
    if (s_ble_down) {
        xSemaphoreGive(s_down_sem);
        return;
    }
    do_record_stop("ble shutdown");
    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        s_link[i].want = false;
        link_drop(i);
    }
    scan_end();
    s_ble_down = true;
    set_state(CAP_IDLE);

    /* Give the controller a moment to actually take the terminations out. */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Both of these exist in IDF v6.1: nimble_port_stop() runs the host stop
     * procedure and then makes nimble_port_run() return, which lets the host
     * task created by blex_init() call nimble_port_freertos_deinit() on
     * itself. nimble_port_deinit() tears the host down and, because
     * CONFIG_BT_CONTROLLER_ENABLED is set, also does esp_bt_controller_disable()
     * and esp_bt_controller_deinit() - so neither is called here. */
    int rc = nimble_port_stop();
    if (rc != 0) {
        ESP_LOGW(TAG, "nimble_port_stop rc=%d", rc);
    }
    /* Best effort: the host task has to have left nimble_port_run() before
     * the deinit frees what it is standing on, and there is no handshake for
     * that in v6.1. */
    vTaskDelay(pdMS_TO_TICKS(200));

    esp_err_t err = nimble_port_deinit();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nimble_port_deinit: %s", esp_err_to_name(err));
    }
    /* Best effort as well: returns the controller and host BSS/data to the
     * heap, which is the whole point - Wi-Fi readout mode needs that RAM. It
     * is irreversible, hence "reboot to capture again". */
    err = esp_bt_mem_release(ESP_BT_MODE_BTDM);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_bt_mem_release: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "BLE down, free heap %lu bytes",
             (unsigned long)esp_get_free_heap_size());
    xSemaphoreGive(s_down_sem);
}

/* ------------------------------------------------------------- the task */

static void handle_msg(const cap_msg_t *m)
{
    cap_state_t st;
    portENTER_CRITICAL(&s_mux);
    st = s_pub.state;
    portEXIT_CRITICAL(&s_mux);

    switch (m->type) {
    case MSG_SCAN_START:
        if (st == CAP_CONNECTING || st == CAP_RECORDING || st == CAP_STOPPING) {
            set_err("busy (%s)", cap_state_str(st));
            break;
        }
        clear_err();
        portENTER_CRITICAL(&s_mux);
        s_pub.link[CAP_LINK_MCU].seen = false;
        s_pub.link[CAP_LINK_BMS].seen = false;
        portEXIT_CRITICAL(&s_mux);
        set_state(CAP_SCANNING);
        if (scan_begin(BLE_HS_FOREVER) != 0) {
            set_state(CAP_IDLE);
        }
        break;

    case MSG_SCAN_STOP:
        if (st == CAP_SCANNING || st == CAP_ARMED) {
            scan_end();
            set_state(CAP_IDLE);
        }
        break;

    case MSG_REC_START:
        s_abort = false;
        do_record_start();
        break;

    case MSG_REC_STOP:
        s_abort = false;
        do_record_stop("user");
        break;

    case MSG_DISCONNECT:
        if (m->link >= CAP_LINK_COUNT) {
            break;
        }
        if (s_link[m->link].want && st == CAP_RECORDING) {
            cap_event("%s disconnected reason=%d", s_target[m->link].label,
                      m->reason);
            /* Retry immediately; the backoff only grows on failed attempts. */
            s_link[m->link].retry_at_ms = now_ms();
        } else {
            ESP_LOGI(TAG, "%s disconnected reason=%d",
                     s_target[m->link].label, m->reason);
        }
        break;

    case MSG_DISC_COMPLETE:
        /* With BLE_HS_FOREVER this only happens on a host reset or a cancel
         * that raced us, so put the scan back if the state still wants one. */
        if (st == CAP_SCANNING || st == CAP_ARMED) {
            ESP_LOGW(TAG, "scan ended unexpectedly, reason=%d", m->reason);
            vTaskDelay(pdMS_TO_TICKS(200));
            if (scan_begin(BLE_HS_FOREVER) != 0) {
                set_state(CAP_IDLE);
            }
        }
        break;

    case MSG_SHUTDOWN:
        s_abort = false;
        do_ble_shutdown();
        break;

    default:
        break;
    }
}

static void cap_task(void *arg)
{
    (void)arg;

    for (;;) {
        cap_msg_t m;
        if (xQueueReceive(s_queue, &m, pdMS_TO_TICKS(CAP_TICK_MS)) == pdTRUE) {
            if (s_ble_down && m.type != MSG_SHUTDOWN) {
                continue;       /* the radio is gone; only a reboot brings it back */
            }
            handle_msg(&m);
        }
        if (s_ble_down) {
            continue;
        }

        cap_state_t st;
        portENTER_CRITICAL(&s_mux);
        st = s_pub.state;
        portEXIT_CRITICAL(&s_mux);

        if (st == CAP_RECORDING) {
            tick_recording();
        } else if (st == CAP_SCANNING || st == CAP_ARMED) {
            tick_scanning();
        }
    }
}

/* -------------------------------------------------------------- public API */

const char *cap_state_str(cap_state_t s)
{
    switch (s) {
    case CAP_IDLE:       return "idle";
    case CAP_SCANNING:   return "scanning";
    case CAP_ARMED:      return "armed";
    case CAP_CONNECTING: return "connecting";
    case CAP_RECORDING:  return "recording";
    case CAP_STOPPING:   return "stopping";
    case CAP_DONE:       return "done";
    case CAP_ERROR:      return "error";
    default:             return "?";
    }
}

esp_err_t cap_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    memset(&s_pub, 0, sizeof(s_pub));
    memset(s_link, 0, sizeof(s_link));
    s_pub.state = CAP_IDLE;
    s_pub.seq = -1;
    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        s_link[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
        copy_str(s_pub.link[i].name, sizeof(s_pub.link[i].name),
                 s_target[i].label);
    }

    s_queue = xQueueCreate(CAP_QUEUE_LEN, sizeof(cap_msg_t));
    s_op_sem = xSemaphoreCreateBinary();
    s_down_sem = xSemaphoreCreateBinary();
    if (s_queue == NULL || s_op_sem == NULL || s_down_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* blex_init() has already brought NimBLE up and waited for the sync
     * callback, so the identity address is available here. */
    if (blex_ready()) {
        if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) {
            s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
        }
    } else {
        ESP_LOGW(TAG, "NimBLE not synced yet, own address type deferred");
    }

    if (xTaskCreate(cap_task, "capture", CAP_TASK_STACK, NULL, CAP_TASK_PRIO,
                    &s_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_inited = true;
    ESP_LOGI(TAG, "capture ready, own_addr_type=%u", s_own_addr_type);
    return ESP_OK;
}

/* Everything that can be checked cheaply and without a state transition is
 * checked here, so the caller gets a real return code and a readable err
 * instead of an asynchronous failure. */
static esp_err_t cap_precheck(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ble_down) {
        set_err("BLE is down, reboot to capture");
        return ESP_ERR_INVALID_STATE;
    }
    if (!blex_ready()) {
        set_err("NimBLE not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t cap_scan_start(void)
{
    esp_err_t err = cap_precheck();
    if (err != ESP_OK) {
        return err;
    }
    /* Mutually exclusive with the console tool: both would be driving the same
     * NimBLE host. */
    if (blex_connected()) {
        set_err("ble_explorer holds a connection");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_scanning && ble_gap_disc_active()) {
        /* Not our scan, so it is the explorer's; starting ours would come
         * back as BLE_HS_EALREADY anyway. */
        set_err("ble_explorer is scanning");
        return ESP_ERR_INVALID_STATE;
    }

    /* The own address type may not have been known at cap_init() time. */
    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) {
        s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
    }

    post_msg(MSG_SCAN_START, 0, 0);
    return ESP_OK;
}

esp_err_t cap_scan_stop(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    post_msg(MSG_SCAN_STOP, 0, 0);
    return ESP_OK;
}

esp_err_t cap_record_start(void)
{
    esp_err_t err = cap_precheck();
    if (err != ESP_OK) {
        return err;
    }
    if (blex_connected()) {
        set_err("ble_explorer holds a connection");
        return ESP_ERR_INVALID_STATE;
    }

    cap_state_t st = cap_state();
    if (st != CAP_ARMED) {
        set_err("not armed (%s)", cap_state_str(st));
        return ESP_ERR_INVALID_STATE;
    }
    if (!store_ready()) {
        set_err("capture store not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    post_msg(MSG_REC_START, 0, 0);
    return ESP_OK;
}

esp_err_t cap_record_stop(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    s_abort = true;
    post_msg(MSG_REC_STOP, 0, 0);
    return ESP_OK;
}

esp_err_t cap_marker(const char *text)
{
    if (!s_inited || !s_rec_on) {
        return ESP_ERR_INVALID_STATE;
    }
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "marker: %s",
                     (text != NULL && text[0] != '\0') ? text : "-");
    if (n < 0) {
        return ESP_FAIL;
    }
    if (n > (int)sizeof(buf) - 1) {
        n = (int)sizeof(buf) - 1;
    }
    /* store_write() is safe from any task and never blocks, so a marker does
     * not have to take the detour through the capture task. */
    if (!store_write(WFREC_EVENT, cap_t_ms(), buf, (uint8_t)n)) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "%s", buf);
    return ESP_OK;
}

void cap_status(cap_status_t *out)
{
    if (out == NULL) {
        return;
    }
    if (!s_inited) {
        memset(out, 0, sizeof(*out));
        out->seq = -1;
        return;
    }

    /* One short critical section, so the UI can poll this at 5 Hz and never
     * sees half of a link's counters from before an update and half from
     * after. Nothing here allocates, blocks or calls into NimBLE. */
    portENTER_CRITICAL(&s_mux);
    memcpy(out, &s_pub, sizeof(*out));
    int64_t t0 = s_t0_us;
    bool rec = s_rec_on;
    portEXIT_CRITICAL(&s_mux);

    if (rec && t0 > 0) {
        int64_t now = esp_timer_get_time();
        out->elapsed_ms = (now > t0) ? (uint32_t)((now - t0) / 1000) : 0;
    }
}

cap_state_t cap_state(void)
{
    cap_state_t st;
    portENTER_CRITICAL(&s_mux);
    st = s_pub.state;
    portEXIT_CRITICAL(&s_mux);
    return st;
}

void cap_live_get(wf_ctrl_live_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_mux);
    memcpy(out, &s_live, sizeof(*out));
    portEXIT_CRITICAL(&s_mux);
}

esp_err_t cap_ble_shutdown(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ble_down) {
        return ESP_OK;
    }

    s_abort = true;
    post_msg(MSG_SHUTDOWN, 0, 0);
    /* Generous: a running capture has to be closed, both links terminated and
     * the host stop procedure has to complete before this returns. */
    if (xSemaphoreTake(s_down_sem, pdMS_TO_TICKS(30000)) != pdTRUE) {
        set_err("BLE shutdown timed out");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

bool cap_ble_down(void)
{
    return s_ble_down;
}
