#include "ble_explorer.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "host/ble_att.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_id.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/hci_common.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "os/os_mbuf.h"

static const char *TAG = "blex";

#define REC_BUF_SIZE (20 * 1024)

/* The record header is written straight into the byte buffer, so it has to be
 * packed and read back with memcpy - the buffer offsets are not aligned. */
struct __attribute__((packed)) rec_hdr {
    uint32_t t_us;
    uint16_t handle;
    uint16_t len;
};

static bool s_ready;
static uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;

/* Serialises console-issued operations; each one posts to the NimBLE host and
 * then waits on s_op_sem for the matching callback. */
static SemaphoreHandle_t s_api_mtx;
static SemaphoreHandle_t s_op_sem;
static SemaphoreHandle_t s_sync_sem;
static volatile int s_op_rc;
static volatile uint16_t s_op_att_status;

static portMUX_TYPE s_rec_mux = portMUX_INITIALIZER_UNLOCKED;

/* --- scan state --- */
static blex_dev_t s_devs[BLEX_MAX_DEVS];
static int s_dev_count;
static volatile bool s_scanning;

/* --- connection state --- */
static volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_connected;
static volatile int s_last_disc_reason;
static uint16_t s_peer_req_itvl_min, s_peer_req_itvl_max;
static uint16_t s_peer_req_latency, s_peer_req_timeout;
static uint32_t s_peer_req_count;

/* --- GATT state --- */
static blex_svc_t s_svcs[BLEX_MAX_SVCS];
static blex_chr_t s_chrs[BLEX_MAX_CHRS];
static blex_dsc_t s_dscs[BLEX_MAX_DSCS];
static int s_svc_count, s_chr_count, s_dsc_count;
static bool s_discovered;
static int s_disc_svc_idx;   /* service the in-flight chr discovery belongs to */
static int s_disc_chr_idx;   /* characteristic the in-flight dsc discovery is for */

/* --- read/write state --- */
static uint8_t *s_read_buf;
static size_t s_read_cap;
static size_t s_read_len;

/* --- notification state --- */
static volatile bool s_nlog;
static volatile uint16_t s_nfilter;
static uint8_t *s_rec_buf;
static volatile size_t s_rec_used;
static volatile bool s_rec_active;
static volatile uint32_t s_rec_frames;
static volatile uint32_t s_rec_dropped;
static int64_t s_rec_t0;
static blex_nstat_t s_nstat[BLEX_MAX_NSTAT];
static int s_nstat_count;

/* ------------------------------------------------------------------ utils */

void blex_addr_str(const ble_addr_t *addr, char *dst)
{
    const uint8_t *v = addr->val;
    sprintf(dst, "%02x:%02x:%02x:%02x:%02x:%02x", v[5], v[4], v[3], v[2], v[1], v[0]);
}

const char *blex_addr_type_str(uint8_t type)
{
    switch (type) {
    case BLE_ADDR_PUBLIC:      return "public";
    case BLE_ADDR_RANDOM:      return "random";
    case BLE_ADDR_PUBLIC_ID:   return "public_id";
    case BLE_ADDR_RANDOM_ID:   return "random_id";
    default:                   return "?";
    }
}

const char *blex_evt_type_str(uint8_t evt_type)
{
    switch (evt_type) {
    case BLE_HCI_ADV_RPT_EVTYPE_ADV_IND:     return "adv_ind";
    case BLE_HCI_ADV_RPT_EVTYPE_DIR_IND:     return "dir_ind";
    case BLE_HCI_ADV_RPT_EVTYPE_SCAN_IND:    return "scan_ind";
    case BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND: return "nonconn_ind";
    case BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP:    return "scan_rsp";
    default:                                 return "?";
    }
}

void blex_props_str(uint8_t props, char *dst)
{
    dst[0] = (props & BLE_GATT_CHR_PROP_BROADCAST)     ? 'B' : '-';
    dst[1] = (props & BLE_GATT_CHR_PROP_READ)          ? 'R' : '-';
    dst[2] = (props & BLE_GATT_CHR_PROP_WRITE_NO_RSP)  ? 'w' : '-';
    dst[3] = (props & BLE_GATT_CHR_PROP_WRITE)         ? 'W' : '-';
    dst[4] = (props & BLE_GATT_CHR_PROP_NOTIFY)        ? 'N' : '-';
    dst[5] = (props & BLE_GATT_CHR_PROP_INDICATE)      ? 'I' : '-';
    dst[6] = (props & BLE_GATT_CHR_PROP_AUTH_SIGN_WRITE) ? 'S' : '-';
    dst[7] = '\0';
}

void blex_print_hex(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        printf("%02x", data[i]);
    }
}

void blex_print_ascii(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        printf("%c", (data[i] >= 0x20 && data[i] < 0x7f) ? data[i] : '.');
    }
}

static const char *ad_type_name(uint8_t type)
{
    switch (type) {
    case 0x01: return "flags";
    case 0x02: return "uuid16_incomplete";
    case 0x03: return "uuid16_complete";
    case 0x04: return "uuid32_incomplete";
    case 0x05: return "uuid32_complete";
    case 0x06: return "uuid128_incomplete";
    case 0x07: return "uuid128_complete";
    case 0x08: return "name_short";
    case 0x09: return "name_complete";
    case 0x0a: return "tx_power";
    case 0x0d: return "class_of_device";
    case 0x12: return "conn_itvl_range";
    case 0x14: return "sol_uuid16";
    case 0x15: return "sol_uuid128";
    case 0x16: return "svc_data_uuid16";
    case 0x19: return "appearance";
    case 0x1a: return "adv_interval";
    case 0x1b: return "le_device_addr";
    case 0x1c: return "le_role";
    case 0x20: return "svc_data_uuid32";
    case 0x21: return "svc_data_uuid128";
    case 0x24: return "uri";
    case 0x27: return "le_supported_features";
    case 0xff: return "manufacturer";
    default:   return "unknown";
    }
}

void blex_print_ad(const uint8_t *data, size_t len, const char *prefix)
{
    size_t i = 0;
    while (i < len) {
        uint8_t flen = data[i];
        if (flen == 0) {
            break;              /* padding, the rest of the payload is unused */
        }
        if (i + 1 + flen > len) {
            printf("%s truncated at offset %u (len byte %u)\n", prefix,
                   (unsigned)i, flen);
            break;
        }
        uint8_t type = data[i + 1];
        const uint8_t *val = &data[i + 2];
        uint8_t vlen = flen - 1;

        printf("%s type=0x%02x %-22s len=%u data=", prefix, type,
               ad_type_name(type), vlen);
        blex_print_hex(val, vlen);

        /* Company ID is the first thing worth knowing about vendor payloads,
         * and names are worth seeing as text. */
        if (type == 0xff && vlen >= 2) {
            printf(" company=0x%04x", (unsigned)(val[0] | (val[1] << 8)));
        } else if ((type == 0x08 || type == 0x09) && vlen > 0) {
            printf(" text=\"");
            blex_print_ascii(val, vlen);
            printf("\"");
        }
        printf("\n");
        i += 1 + flen;
    }
}

int blex_parse_addr(const char *str, const char *type_str, ble_addr_t *out)
{
    unsigned v[6];
    if (sscanf(str, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        return -1;
    }
    for (int i = 0; i < 6; i++) {
        if (v[i] > 0xff) {
            return -1;
        }
        out->val[5 - i] = (uint8_t)v[i];   /* NimBLE stores addresses LSB first */
    }
    out->type = BLE_ADDR_PUBLIC;
    if (type_str != NULL) {
        if (strcmp(type_str, "random") == 0) {
            out->type = BLE_ADDR_RANDOM;
        } else if (strcmp(type_str, "public") != 0) {
            return -1;
        }
    }
    return 0;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int blex_parse_hex(const char *str, uint8_t *out, size_t cap, size_t *out_len)
{
    size_t n = 0;
    int hi = -1;
    for (const char *p = str; *p != '\0'; p++) {
        if (*p == ':' || *p == ' ' || *p == '-' || *p == ',') {
            continue;
        }
        int v = hexval(*p);
        if (v < 0) {
            return -1;
        }
        if (hi < 0) {
            hi = v;
        } else {
            if (n >= cap) {
                return -1;
            }
            out[n++] = (uint8_t)((hi << 4) | v);
            hi = -1;
        }
    }
    if (hi >= 0) {
        return -1;              /* odd number of nibbles */
    }
    *out_len = n;
    return 0;
}

/* ------------------------------------------------------- notification sink */

static blex_nstat_t *nstat_for(uint16_t handle)
{
    for (int i = 0; i < s_nstat_count; i++) {
        if (s_nstat[i].handle == handle) {
            return &s_nstat[i];
        }
    }
    if (s_nstat_count >= BLEX_MAX_NSTAT) {
        return NULL;
    }
    blex_nstat_t *st = &s_nstat[s_nstat_count++];
    memset(st, 0, sizeof(*st));
    st->handle = handle;
    st->len_min = UINT16_MAX;
    return st;
}

static void bitset_add(uint32_t *set, uint8_t v)
{
    set[v >> 5] |= (uint32_t)1 << (v & 31);
}

static void bitset_print(const uint32_t *set)
{
    bool first = true;
    for (int v = 0; v < 256; v++) {
        if (set[v >> 5] & ((uint32_t)1 << (v & 31))) {
            printf("%s%02x", first ? "" : ",", v);
            first = false;
        }
    }
    if (first) {
        printf("-");
    }
}

static void notify_sink(uint16_t handle, const uint8_t *data, size_t len)
{
    int64_t now = esp_timer_get_time();

    if (s_nfilter != 0 && handle != s_nfilter) {
        return;
    }

    blex_nstat_t *st = nstat_for(handle);
    if (st != NULL) {
        if (st->count == 0) {
            st->first_us = now;
        }
        st->count++;
        st->bytes += len;
        st->last_us = now;
        if (len < st->len_min) st->len_min = (uint16_t)len;
        if (len > st->len_max) st->len_max = (uint16_t)len;
        if (len >= 1) bitset_add(st->b0_seen, data[0]);
        if (len >= 2) bitset_add(st->b1_seen, data[1]);
    }

    if (s_rec_active && s_rec_buf != NULL) {
        struct rec_hdr hdr = {
            .t_us = (uint32_t)(now - s_rec_t0),
            .handle = handle,
            .len = (uint16_t)len,
        };
        portENTER_CRITICAL(&s_rec_mux);
        if (s_rec_used + sizeof(hdr) + len <= REC_BUF_SIZE) {
            memcpy(s_rec_buf + s_rec_used, &hdr, sizeof(hdr));
            memcpy(s_rec_buf + s_rec_used + sizeof(hdr), data, len);
            s_rec_used += sizeof(hdr) + len;
            s_rec_frames++;
        } else {
            s_rec_dropped++;
        }
        portEXIT_CRITICAL(&s_rec_mux);
    }

    if (s_nlog) {
        printf("NTF t_us=%" PRId64 " h=0x%04x len=%u data=", now, handle, (unsigned)len);
        blex_print_hex(data, len);
        printf("\n");
    }
}

/* ------------------------------------------------------------ GAP handlers */

static blex_dev_t *dev_find_or_add(const ble_addr_t *addr)
{
    for (int i = 0; i < s_dev_count; i++) {
        if (s_devs[i].addr.type == addr->type &&
            memcmp(s_devs[i].addr.val, addr->val, 6) == 0) {
            return &s_devs[i];
        }
    }
    if (s_dev_count >= BLEX_MAX_DEVS) {
        return NULL;
    }
    blex_dev_t *d = &s_devs[s_dev_count++];
    memset(d, 0, sizeof(*d));
    d->addr = *addr;
    d->rssi_min = 127;
    d->rssi_max = -128;
    d->itvl_min_us = UINT32_MAX;
    return d;
}

static void dev_capture_name(blex_dev_t *d, const uint8_t *data, size_t len)
{
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, data, len) != 0) {
        return;
    }
    if (fields.name != NULL && fields.name_len > 0) {
        size_t n = fields.name_len;
        if (n > sizeof(d->name) - 1) {
            n = sizeof(d->name) - 1;
        }
        memcpy(d->name, fields.name, n);
        d->name[n] = '\0';
    }
}

static int on_disc_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        const struct ble_gap_disc_desc *d = &event->disc;
        int64_t now = esp_timer_get_time();

        blex_dev_t *dev = dev_find_or_add(&d->addr);
        if (dev == NULL) {
            return 0;
        }
        if (dev->adv_count == 0 && dev->rsp_count == 0) {
            dev->first_us = now;
        }
        dev->last_us = now;
        dev->rssi_last = d->rssi;
        if (d->rssi < dev->rssi_min) dev->rssi_min = d->rssi;
        if (d->rssi > dev->rssi_max) dev->rssi_max = d->rssi;
        if (d->event_type <= 7) {
            dev->evt_mask |= (uint8_t)(1u << d->event_type);
        }

        if (d->event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP) {
            dev->rsp_count++;
            dev->rsp_len = d->length_data > 31 ? 31 : d->length_data;
            memcpy(dev->rsp, d->data, dev->rsp_len);
            dev_capture_name(dev, d->data, d->length_data);
        } else {
            /* Interval is only meaningful between two advertising events, so
             * scan responses must not be folded into the measurement. */
            if (dev->prev_adv_us != 0) {
                uint32_t dt = (uint32_t)(now - dev->prev_adv_us);
                if (dt < dev->itvl_min_us) dev->itvl_min_us = dt;
                if (dt > dev->itvl_max_us) dev->itvl_max_us = dt;
                dev->itvl_sum_us += dt;
                dev->itvl_n++;
            }
            dev->prev_adv_us = now;
            dev->adv_count++;
            dev->adv_len = d->length_data > 31 ? 31 : d->length_data;
            memcpy(dev->adv, d->data, dev->adv_len);
            dev_capture_name(dev, d->data, d->length_data);
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scanning = false;
        s_op_rc = event->disc_complete.reason;
        xSemaphoreGive(s_op_sem);
        return 0;

    default:
        return 0;
    }
}

static int on_conn_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        s_op_rc = event->connect.status;
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_connected = false;
        }
        xSemaphoreGive(s_op_sem);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        s_last_disc_reason = event->disconnect.reason;
        s_connected = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_discovered = false;
        printf("DISCONNECT reason=%d (0x%04x)\n", event->disconnect.reason,
               event->disconnect.reason);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        printf("CONN_UPDATE status=%d\n", event->conn_update.status);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE_REQ: {
        const struct ble_gap_upd_params *p = event->conn_update_req.peer_params;
        s_peer_req_itvl_min = p->itvl_min;
        s_peer_req_itvl_max = p->itvl_max;
        s_peer_req_latency  = p->latency;
        s_peer_req_timeout  = p->supervision_timeout;
        s_peer_req_count++;
        printf("PEER_PARAM_REQ itvl_min=%u itvl_max=%u latency=%u timeout=%u\n",
               p->itvl_min, p->itvl_max, p->latency, p->supervision_timeout);
        return 0;   /* accept what the peer asked for */
    }

    case BLE_GAP_EVENT_MTU:
        printf("MTU conn=%u cid=%u value=%u\n", event->mtu.conn_handle,
               event->mtu.channel_id, event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        printf("ENC_CHANGE status=%d\n", event->enc_change.status);
        s_op_rc = event->enc_change.status;
        xSemaphoreGive(s_op_sem);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        /* Only the NimBLE host task delivers this, so one static buffer is
         * enough and keeps 512 bytes off that task's stack. */
        static uint8_t buf[BLEX_MAX_ATT_LEN];
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        if (os_mbuf_copydata(event->notify_rx.om, 0, len, buf) == 0) {
            notify_sink(event->notify_rx.attr_handle, buf, len);
        }
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* Nothing is bonded on purpose; let the peer start over. */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        return 0;
    }
}

/* --------------------------------------------------------------- lifecycle */

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) {
        s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
    }
    uint8_t addr[6] = {0};
    ble_hs_id_copy_addr(s_own_addr_type, addr, NULL);
    printf("BLE_READY own_addr=%02x:%02x:%02x:%02x:%02x:%02x type=%u\n",
           addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], s_own_addr_type);
    s_ready = true;
    xSemaphoreGive(s_sync_sem);
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset, reason %d", reason);
    s_ready = false;
    s_connected = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t blex_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    s_api_mtx = xSemaphoreCreateMutex();
    s_op_sem = xSemaphoreCreateBinary();
    s_sync_sem = xSemaphoreCreateBinary();
    if (s_api_mtx == NULL || s_op_sem == NULL || s_sync_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_rec_buf = malloc(REC_BUF_SIZE);
    if (s_rec_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    err = nimble_port_init();
    if (err != ESP_OK) {
        return err;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    /* No bonding and no IO: everything we need is readable without pairing,
     * and staying unbonded keeps reconnects reproducible. */
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;

    /* No ble_svc_gap_init() here: the GAP service is peripheral-side GATT and
     * this build has the peripheral role disabled. A pure central needs none
     * of it. */
    nimble_port_freertos_init(host_task);

    /* The controller needs a moment to come up; give the caller a stack that
     * is already usable when blex_init() returns. */
    xSemaphoreTake(s_sync_sem, pdMS_TO_TICKS(5000));
    return ESP_OK;
}

bool blex_ready(void)
{
    return s_ready;
}

/* ---------------------------------------------------------------- scanning */

int blex_scan(int duration_ms, bool passive, bool filter_duplicates)
{
    if (!s_ready) {
        return BLE_HS_EAGAIN;
    }
    xSemaphoreTake(s_api_mtx, portMAX_DELAY);

    struct ble_gap_disc_params params = {
        .itvl = 0x0060,             /* 60 ms interval  */
        .window = 0x0050,           /* 50 ms window -> ~83 % duty cycle */
        .filter_policy = 0,
        .limited = 0,
        .passive = passive ? 1 : 0,
        .filter_duplicates = filter_duplicates ? 1 : 0,
    };

    xSemaphoreTake(s_op_sem, 0);    /* drop a stale completion, if any */
    s_scanning = true;
    int rc = ble_gap_disc(s_own_addr_type, duration_ms, &params, on_disc_event, NULL);
    if (rc != 0) {
        s_scanning = false;
        xSemaphoreGive(s_api_mtx);
        return rc;
    }

    if (xSemaphoreTake(s_op_sem, pdMS_TO_TICKS(duration_ms + 2000)) != pdTRUE) {
        ble_gap_disc_cancel();
        s_scanning = false;
        xSemaphoreGive(s_api_mtx);
        return BLE_HS_ETIMEOUT;
    }
    xSemaphoreGive(s_api_mtx);
    return 0;
}

void blex_scan_clear(void)
{
    s_dev_count = 0;
    memset(s_devs, 0, sizeof(s_devs));
}

int blex_dev_count(void) { return s_dev_count; }

const blex_dev_t *blex_dev(int idx)
{
    if (idx < 0 || idx >= s_dev_count) {
        return NULL;
    }
    return &s_devs[idx];
}

int blex_find_by_name(const char *substr)
{
    for (int i = 0; i < s_dev_count; i++) {
        if (s_devs[i].name[0] != '\0' && strstr(s_devs[i].name, substr) != NULL) {
            return i;
        }
    }
    return -1;
}

int blex_find_by_addr(const ble_addr_t *addr)
{
    for (int i = 0; i < s_dev_count; i++) {
        if (memcmp(s_devs[i].addr.val, addr->val, 6) == 0) {
            return i;
        }
    }
    return -1;
}

/* -------------------------------------------------------------- connection */

int blex_connect(const ble_addr_t *addr, int timeout_ms)
{
    if (!s_ready) {
        return BLE_HS_EAGAIN;
    }
    if (s_connected) {
        return BLE_HS_EALREADY;
    }
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }

    xSemaphoreTake(s_api_mtx, portMAX_DELAY);

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
    int rc = ble_gap_connect(s_own_addr_type, addr, timeout_ms, &params,
                             on_conn_event, NULL);
    if (rc != 0) {
        xSemaphoreGive(s_api_mtx);
        return rc;
    }
    if (xSemaphoreTake(s_op_sem, pdMS_TO_TICKS(timeout_ms + 2000)) != pdTRUE) {
        ble_gap_conn_cancel();
        xSemaphoreGive(s_api_mtx);
        return BLE_HS_ETIMEOUT;
    }
    rc = s_op_rc;
    xSemaphoreGive(s_api_mtx);
    return rc;
}

int blex_disconnect(void)
{
    if (!s_connected) {
        return BLE_HS_ENOTCONN;
    }
    return ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

bool blex_connected(void) { return s_connected; }

uint16_t blex_conn_handle(void) { return s_conn_handle; }

int blex_conn_desc(struct ble_gap_conn_desc *out)
{
    if (!s_connected) {
        return BLE_HS_ENOTCONN;
    }
    return ble_gap_conn_find(s_conn_handle, out);
}

static int on_mtu(uint16_t conn_handle, const struct ble_gatt_error *error,
                  uint16_t mtu, void *arg)
{
    (void)conn_handle;
    (void)arg;
    s_op_rc = error->status;
    s_op_att_status = mtu;
    xSemaphoreGive(s_op_sem);
    return 0;
}

int blex_exchange_mtu(uint16_t *out_mtu)
{
    if (!s_connected) {
        return BLE_HS_ENOTCONN;
    }
    xSemaphoreTake(s_api_mtx, portMAX_DELAY);
    xSemaphoreTake(s_op_sem, 0);
    int rc = ble_gattc_exchange_mtu(s_conn_handle, on_mtu, NULL);
    if (rc == 0) {
        if (xSemaphoreTake(s_op_sem, pdMS_TO_TICKS(5000)) != pdTRUE) {
            rc = BLE_HS_ETIMEOUT;
        } else {
            rc = s_op_rc;
            if (out_mtu != NULL) {
                *out_mtu = s_op_att_status;
            }
        }
    }
    xSemaphoreGive(s_api_mtx);
    return rc;
}

uint16_t blex_mtu(void)
{
    return s_connected ? ble_att_mtu(s_conn_handle) : 0;
}

int blex_update_params(uint16_t itvl_min, uint16_t itvl_max,
                       uint16_t latency, uint16_t timeout_10ms)
{
    if (!s_connected) {
        return BLE_HS_ENOTCONN;
    }
    struct ble_gap_upd_params p = {
        .itvl_min = itvl_min,
        .itvl_max = itvl_max,
        .latency = latency,
        .supervision_timeout = timeout_10ms,
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    return ble_gap_update_params(s_conn_handle, &p);
}

int blex_rssi(int8_t *out_rssi)
{
    if (!s_connected) {
        return BLE_HS_ENOTCONN;
    }
    return ble_gap_conn_rssi(s_conn_handle, out_rssi);
}

int blex_security(void)
{
    if (!s_connected) {
        return BLE_HS_ENOTCONN;
    }
    xSemaphoreTake(s_api_mtx, portMAX_DELAY);
    xSemaphoreTake(s_op_sem, 0);
    int rc = ble_gap_security_initiate(s_conn_handle);
    if (rc == 0) {
        if (xSemaphoreTake(s_op_sem, pdMS_TO_TICKS(10000)) != pdTRUE) {
            rc = BLE_HS_ETIMEOUT;
        } else {
            rc = s_op_rc;
        }
    }
    xSemaphoreGive(s_api_mtx);
    return rc;
}

void blex_peer_param_req(uint16_t *itvl_min, uint16_t *itvl_max,
                         uint16_t *latency, uint16_t *timeout, uint32_t *count)
{
    *itvl_min = s_peer_req_itvl_min;
    *itvl_max = s_peer_req_itvl_max;
    *latency = s_peer_req_latency;
    *timeout = s_peer_req_timeout;
    *count = s_peer_req_count;
}

int blex_last_disconnect_reason(void) { return s_last_disc_reason; }

/* -------------------------------------------------------------- discovery */

static int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg)
{
    (void)conn_handle;
    (void)arg;

    if (error->status == 0 && service != NULL) {
        if (s_svc_count < BLEX_MAX_SVCS) {
            blex_svc_t *s = &s_svcs[s_svc_count++];
            s->start_handle = service->start_handle;
            s->end_handle = service->end_handle;
            s->uuid = service->uuid;
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
        if (s_chr_count < BLEX_MAX_CHRS) {
            blex_chr_t *c = &s_chrs[s_chr_count++];
            c->svc_idx = (int16_t)s_disc_svc_idx;
            c->def_handle = chr->def_handle;
            c->val_handle = chr->val_handle;
            c->end_handle = 0;      /* filled in once the next one is known */
            c->cccd_handle = 0;
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
        if (s_dsc_count < BLEX_MAX_DSCS) {
            blex_dsc_t *d = &s_dscs[s_dsc_count++];
            d->chr_idx = (int16_t)s_disc_chr_idx;
            d->handle = dsc->handle;
            d->uuid = dsc->uuid;
        }
        /* Remember the CCCD: every later subscribe writes to this handle. */
        if (blex_uuid_is16(&dsc->uuid.u, 0x2902) &&
            s_disc_chr_idx >= 0 && s_disc_chr_idx < s_chr_count) {
            s_chrs[s_disc_chr_idx].cccd_handle = dsc->handle;
        }
        return 0;
    }
    s_op_rc = (error->status == BLE_HS_EDONE) ? 0 : error->status;
    xSemaphoreGive(s_op_sem);
    return 0;
}

static int wait_op(int timeout_ms)
{
    if (xSemaphoreTake(s_op_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return BLE_HS_ETIMEOUT;
    }
    return s_op_rc;
}

int blex_discover(void)
{
    if (!s_connected) {
        return BLE_HS_ENOTCONN;
    }
    xSemaphoreTake(s_api_mtx, portMAX_DELAY);

    s_svc_count = 0;
    s_chr_count = 0;
    s_dsc_count = 0;
    s_discovered = false;

    xSemaphoreTake(s_op_sem, 0);
    int rc = ble_gattc_disc_all_svcs(s_conn_handle, on_disc_svc, NULL);
    if (rc == 0) {
        rc = wait_op(15000);
    }
    if (rc != 0) {
        xSemaphoreGive(s_api_mtx);
        return rc;
    }

    for (int i = 0; i < s_svc_count; i++) {
        s_disc_svc_idx = i;
        int chr_first = s_chr_count;

        xSemaphoreTake(s_op_sem, 0);
        rc = ble_gattc_disc_all_chrs(s_conn_handle, s_svcs[i].start_handle,
                                     s_svcs[i].end_handle, on_disc_chr, NULL);
        if (rc == 0) {
            rc = wait_op(15000);
        }
        if (rc != 0) {
            xSemaphoreGive(s_api_mtx);
            return rc;
        }

        /* A characteristic owns every handle up to the next definition, or up
         * to the end of the service for the last one in it. */
        for (int c = chr_first; c < s_chr_count; c++) {
            s_chrs[c].end_handle = (c + 1 < s_chr_count)
                                       ? (uint16_t)(s_chrs[c + 1].def_handle - 1)
                                       : s_svcs[i].end_handle;
        }

        for (int c = chr_first; c < s_chr_count; c++) {
            if (s_chrs[c].val_handle >= s_chrs[c].end_handle) {
                continue;           /* no room for descriptors */
            }
            s_disc_chr_idx = c;
            xSemaphoreTake(s_op_sem, 0);
            /* NimBLE takes the value handle itself and adds the +1 when it
             * builds the FIND_INFO request, so passing val_handle + 1 here
             * would skip a handle and, for a one-handle gap, send an invalid
             * start > end range. */
            rc = ble_gattc_disc_all_dscs(s_conn_handle, s_chrs[c].val_handle,
                                         s_chrs[c].end_handle, on_disc_dsc, NULL);
            if (rc == 0) {
                rc = wait_op(15000);
            }
            if (rc != 0) {
                /* One characteristic refusing its descriptors is not a reason
                 * to lose the rest of the database. */
                printf("DSC_SKIP chr=%d val_handle=0x%04x rc=%d\n",
                       c, s_chrs[c].val_handle, rc);
            }
        }
    }

    s_discovered = true;
    xSemaphoreGive(s_api_mtx);
    return 0;
}

bool blex_discovered(void) { return s_discovered; }
int  blex_svc_count(void)  { return s_svc_count; }
int  blex_chr_count(void)  { return s_chr_count; }
int  blex_dsc_count(void)  { return s_dsc_count; }

const blex_svc_t *blex_svc(int idx)
{
    return (idx >= 0 && idx < s_svc_count) ? &s_svcs[idx] : NULL;
}
const blex_chr_t *blex_chr(int idx)
{
    return (idx >= 0 && idx < s_chr_count) ? &s_chrs[idx] : NULL;
}
const blex_dsc_t *blex_dsc(int idx)
{
    return (idx >= 0 && idx < s_dsc_count) ? &s_dscs[idx] : NULL;
}

int blex_chr_by_val_handle(uint16_t handle)
{
    for (int i = 0; i < s_chr_count; i++) {
        if (s_chrs[i].val_handle == handle) {
            return i;
        }
    }
    return -1;
}

/* ------------------------------------------------------------ read / write */

static int on_read(uint16_t conn_handle, const struct ble_gatt_error *error,
                   struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)arg;

    s_op_att_status = error->status;
    if (error->status == 0 && attr != NULL && attr->om != NULL) {
        uint16_t len = OS_MBUF_PKTLEN(attr->om);
        if (len > s_read_cap) {
            len = (uint16_t)s_read_cap;
        }
        if (os_mbuf_copydata(attr->om, 0, len, s_read_buf) == 0) {
            s_read_len = len;
        }
    }
    s_op_rc = error->status;
    xSemaphoreGive(s_op_sem);
    return 0;
}

int blex_read(uint16_t handle, uint8_t *out, size_t cap, size_t *out_len,
              uint16_t *out_att_status)
{
    if (!s_connected) {
        return BLE_HS_ENOTCONN;
    }
    xSemaphoreTake(s_api_mtx, portMAX_DELAY);

    s_read_buf = out;
    s_read_cap = cap;
    s_read_len = 0;

    xSemaphoreTake(s_op_sem, 0);
    int rc = ble_gattc_read(s_conn_handle, handle, on_read, NULL);
    if (rc == 0) {
        rc = wait_op(5000);
    }
    if (out_len != NULL) {
        *out_len = s_read_len;
    }
    if (out_att_status != NULL) {
        *out_att_status = s_op_att_status;
    }
    xSemaphoreGive(s_api_mtx);
    return rc;
}

static int on_write(uint16_t conn_handle, const struct ble_gatt_error *error,
                    struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;
    s_op_att_status = error->status;
    s_op_rc = error->status;
    xSemaphoreGive(s_op_sem);
    return 0;
}

int blex_write(uint16_t handle, const uint8_t *data, size_t len, bool with_rsp,
               uint16_t *out_att_status)
{
    if (!s_connected) {
        return BLE_HS_ENOTCONN;
    }
    if (!with_rsp) {
        /* Write without response completes locally; there is nothing to wait
         * for and no ATT status to report. */
        int rc = ble_gattc_write_no_rsp_flat(s_conn_handle, handle, data, (uint16_t)len);
        if (out_att_status != NULL) {
            *out_att_status = 0;
        }
        return rc;
    }

    xSemaphoreTake(s_api_mtx, portMAX_DELAY);
    xSemaphoreTake(s_op_sem, 0);
    int rc = ble_gattc_write_flat(s_conn_handle, handle, data, (uint16_t)len,
                                  on_write, NULL);
    if (rc == 0) {
        rc = wait_op(5000);
    }
    if (out_att_status != NULL) {
        *out_att_status = s_op_att_status;
    }
    xSemaphoreGive(s_api_mtx);
    return rc;
}

int blex_subscribe(uint16_t cccd_handle, uint16_t value)
{
    uint8_t val[2] = { (uint8_t)(value & 0xff), (uint8_t)(value >> 8) };
    return blex_write(cccd_handle, val, sizeof(val), true, NULL);
}

/* ---------------------------------------------------------- notifications */

void blex_notify_log(bool on) { s_nlog = on; }
bool blex_notify_log_get(void) { return s_nlog; }
void blex_notify_filter(uint16_t handle) { s_nfilter = handle; }

void blex_rec_start(void)
{
    portENTER_CRITICAL(&s_rec_mux);
    s_rec_used = 0;
    s_rec_frames = 0;
    s_rec_dropped = 0;
    portEXIT_CRITICAL(&s_rec_mux);
    s_rec_t0 = esp_timer_get_time();
    s_rec_active = true;
}

void blex_rec_stop(void) { s_rec_active = false; }
bool blex_rec_active(void) { return s_rec_active; }
size_t blex_rec_used(void) { return s_rec_used; }
size_t blex_rec_capacity(void) { return REC_BUF_SIZE; }
uint32_t blex_rec_frames(void) { return s_rec_frames; }
uint32_t blex_rec_dropped(void) { return s_rec_dropped; }

void blex_rec_clear(void)
{
    portENTER_CRITICAL(&s_rec_mux);
    s_rec_used = 0;
    s_rec_frames = 0;
    s_rec_dropped = 0;
    portEXIT_CRITICAL(&s_rec_mux);
}

void blex_rec_dump(uint32_t max_frames)
{
    bool was_active = s_rec_active;
    s_rec_active = false;           /* freeze the buffer while walking it */

    size_t off = 0;
    uint32_t n = 0;
    size_t used = s_rec_used;

    while (off + sizeof(struct rec_hdr) <= used) {
        struct rec_hdr hdr;
        memcpy(&hdr, s_rec_buf + off, sizeof(hdr));
        off += sizeof(hdr);
        if (off + hdr.len > used) {
            break;
        }
        if (max_frames != 0 && n >= max_frames) {
            break;
        }
        printf("REC n=%" PRIu32 " t_us=%" PRIu32 " h=0x%04x len=%u data=",
               n, hdr.t_us, hdr.handle, hdr.len);
        blex_print_hex(s_rec_buf + off, hdr.len);
        printf("\n");
        off += hdr.len;
        n++;
    }
    printf("REC_END frames=%" PRIu32 " printed=%" PRIu32 " dropped=%" PRIu32
           " bytes=%u/%u\n",
           s_rec_frames, n, s_rec_dropped, (unsigned)used, (unsigned)REC_BUF_SIZE);

    s_rec_active = was_active;
}

void blex_nstat_reset(void)
{
    s_nstat_count = 0;
    memset(s_nstat, 0, sizeof(s_nstat));
}

int blex_nstat_count(void) { return s_nstat_count; }

const blex_nstat_t *blex_nstat(int idx)
{
    return (idx >= 0 && idx < s_nstat_count) ? &s_nstat[idx] : NULL;
}

/* Used by cmd_ble.c to print the byte sets without exporting the bit layout. */
void blex_nstat_print_bytes(const blex_nstat_t *st)
{
    printf(" byte0={");
    bitset_print(st->b0_seen);
    printf("} byte1={");
    bitset_print(st->b1_seen);
    printf("}");
}
