/*
 * Console commands driving ble_explorer.
 *
 * Output is written as tagged, single-line records ("SCAN ...", "CHR ...",
 * "REC ...") so a script on the host can capture a session over the serial
 * link and parse it afterwards without guessing at layout.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ble_explorer.h"
#include "daly.h"

#include "esp_console.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_gatt.h"
#include "host/ble_hs.h"

#define READ_BUF_LEN 512

static uint8_t s_buf[READ_BUF_LEN];

/* ------------------------------------------------------------ UUID naming */

/* Only the entries that actually come up on these two devices, plus the SIG
 * services the modules inherit from their Bluetooth-to-serial firmware. */
static const struct {
    uint16_t uuid;
    const char *name;
} k_uuid16_names[] = {
    {0x1800, "gap"},
    {0x1801, "gatt"},
    {0x180a, "device_information"},
    {0x180f, "battery_service"},
    {0x2901, "char_user_description"},
    {0x2902, "cccd"},
    {0x2903, "server_char_config"},
    {0x2904, "char_presentation_format"},
    {0x2a00, "device_name"},
    {0x2a01, "appearance"},
    {0x2a04, "ppcp"},
    {0x2a19, "battery_level"},
    {0x2a23, "system_id"},
    {0x2a24, "model_number"},
    {0x2a25, "serial_number"},
    {0x2a26, "firmware_revision"},
    {0x2a27, "hardware_revision"},
    {0x2a28, "software_revision"},
    {0x2a29, "manufacturer_name"},
    {0x2a05, "service_changed"},
    {0xfff0, "daly_service"},
    {0xfff1, "daly_notify"},
    {0xfff2, "daly_control"},
    {0xffe0, "fardriver_service"},
    {0xffe1, "hm10_uart"},
    {0xffec, "fardriver_data"},
};

static const char *uuid_name(const ble_uuid_t *uuid)
{
    if (uuid->type != BLE_UUID_TYPE_16) {
        return "";
    }
    uint16_t v = BLE_UUID16(uuid)->value;
    for (size_t i = 0; i < sizeof(k_uuid16_names) / sizeof(k_uuid16_names[0]); i++) {
        if (k_uuid16_names[i].uuid == v) {
            return k_uuid16_names[i].name;
        }
    }
    return "";
}

static void print_uuid(const ble_uuid_t *uuid)
{
    char str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(uuid, str);
    const char *name = uuid_name(uuid);
    printf("%s%s%s%s", str, name[0] ? " (" : "", name, name[0] ? ")" : "");
}

/* ---------------------------------------------------------------- scanning */

static void print_dev_line(int idx, const blex_dev_t *d)
{
    char addr[18];
    blex_addr_str(&d->addr, addr);

    uint32_t itvl_avg = d->itvl_n ? (uint32_t)(d->itvl_sum_us / d->itvl_n) : 0;

    printf("DEV idx=%d addr=%s type=%s name=\"%s\" rssi=%d/%d/%d adv=%u rsp=%u "
           "itvl_us=%" PRIu32 "/%" PRIu32 "/%" PRIu32 " evt=",
           idx, addr, blex_addr_type_str(d->addr.type), d->name,
           d->rssi_min, d->rssi_last, d->rssi_max, d->adv_count, d->rsp_count,
           d->itvl_min_us == UINT32_MAX ? 0 : d->itvl_min_us, itvl_avg, d->itvl_max_us);

    bool first = true;
    for (int b = 0; b < 8; b++) {
        if (d->evt_mask & (1u << b)) {
            printf("%s%s", first ? "" : "|", blex_evt_type_str((uint8_t)b));
            first = false;
        }
    }
    printf(" adv_raw=");
    blex_print_hex(d->adv, d->adv_len);
    printf(" rsp_raw=");
    blex_print_hex(d->rsp, d->rsp_len);
    printf("\n");
}

static int cmd_scan(int argc, char **argv)
{
    int secs = 5;
    bool passive = false;
    bool dedup = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "passive") == 0) {
            passive = true;
        } else if (strcmp(argv[i], "dedup") == 0) {
            dedup = true;
        } else {
            secs = atoi(argv[i]);
        }
    }
    if (secs <= 0) {
        secs = 5;
    }

    /* Duplicate filtering is off by default: seeing every single advertising
     * event is what makes the interval and RSSI statistics meaningful, and it
     * is the only way to notice an address that rotates mid-scan. */
    printf("SCAN_START secs=%d mode=%s dedup=%d\n", secs,
           passive ? "passive" : "active", dedup);
    int rc = blex_scan(secs * 1000, passive, dedup);
    if (rc != 0) {
        printf("SCAN_ERR rc=%d\n", rc);
        return 1;
    }

    int n = blex_dev_count();
    for (int i = 0; i < n; i++) {
        print_dev_line(i, blex_dev(i));
    }
    printf("SCAN_END devices=%d\n", n);
    return 0;
}

static int cmd_devs(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    int n = blex_dev_count();
    for (int i = 0; i < n; i++) {
        print_dev_line(i, blex_dev(i));
    }
    printf("DEVS_END devices=%d\n", n);
    return 0;
}

static int cmd_dev(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: dev <idx>\n");
        return 1;
    }
    const blex_dev_t *d = blex_dev(atoi(argv[1]));
    if (d == NULL) {
        printf("ERR no such device\n");
        return 1;
    }
    print_dev_line(atoi(argv[1]), d);
    printf(" ADV payload:\n");
    blex_print_ad(d->adv, d->adv_len, "  AD");
    if (d->rsp_len > 0) {
        printf(" SCAN RESPONSE payload:\n");
        blex_print_ad(d->rsp, d->rsp_len, "  AD");
    }
    return 0;
}

static int cmd_scanclear(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    blex_scan_clear();
    printf("SCAN_CLEARED\n");
    return 0;
}

/* -------------------------------------------------------------- connection */

static int resolve_target(int argc, char **argv, ble_addr_t *out)
{
    if (argc < 2) {
        return -1;
    }
    if (strcmp(argv[1], "name") == 0 && argc >= 3) {
        int idx = blex_find_by_name(argv[2]);
        if (idx < 0) {
            printf("ERR no scanned device matches name \"%s\"\n", argv[2]);
            return -1;
        }
        *out = blex_dev(idx)->addr;
        return 0;
    }
    if (strchr(argv[1], ':') != NULL) {
        return blex_parse_addr(argv[1], argc >= 3 ? argv[2] : NULL, out);
    }
    const blex_dev_t *d = blex_dev(atoi(argv[1]));
    if (d == NULL) {
        printf("ERR no such device index\n");
        return -1;
    }
    *out = d->addr;
    return 0;
}

static void print_conn_info(void)
{
    struct ble_gap_conn_desc desc;
    if (blex_conn_desc(&desc) != 0) {
        printf("CONN state=disconnected last_reason=%d\n",
               blex_last_disconnect_reason());
        return;
    }

    char peer[18], own[18];
    blex_addr_str(&desc.peer_id_addr, peer);
    blex_addr_str(&desc.our_id_addr, own);

    int8_t rssi = 0;
    blex_rssi(&rssi);

    uint16_t pmin, pmax, plat, pto;
    uint32_t pcount;
    blex_peer_param_req(&pmin, &pmax, &plat, &pto, &pcount);

    printf("CONN state=connected handle=%u peer=%s peer_type=%s own=%s "
           "itvl=%u (%u.%02u ms) latency=%u timeout=%u (%u ms) mtu=%u rssi=%d "
           "encrypted=%d authenticated=%d bonded=%d key_size=%u\n",
           desc.conn_handle, peer, blex_addr_type_str(desc.peer_id_addr.type), own,
           desc.conn_itvl, desc.conn_itvl * 125 / 100, (desc.conn_itvl * 125 % 100),
           desc.conn_latency, desc.supervision_timeout, desc.supervision_timeout * 10,
           blex_mtu(), rssi,
           desc.sec_state.encrypted, desc.sec_state.authenticated,
           desc.sec_state.bonded, desc.sec_state.key_size);
    printf("CONN peer_param_requests=%" PRIu32 " itvl_min=%u itvl_max=%u "
           "latency=%u timeout=%u\n", pcount, pmin, pmax, plat, pto);
}

static int cmd_connect(int argc, char **argv)
{
    ble_addr_t addr;
    if (resolve_target(argc, argv, &addr) != 0) {
        printf("usage: connect <idx> | connect <aa:bb:cc:dd:ee:ff> [public|random]"
               " | connect name <substring>\n");
        return 1;
    }

    char s[18];
    blex_addr_str(&addr, s);
    printf("CONNECTING addr=%s type=%s\n", s, blex_addr_type_str(addr.type));

    int64_t t0 = esp_timer_get_time();
    int rc = blex_connect(&addr, 15000);
    int64_t dt = esp_timer_get_time() - t0;
    if (rc != 0) {
        printf("CONNECT_ERR rc=%d took_ms=%" PRId64 "\n", rc, dt / 1000);
        return 1;
    }
    printf("CONNECTED took_ms=%" PRId64 "\n", dt / 1000);
    print_conn_info();
    return 0;
}

static int cmd_disconnect(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    int rc = blex_disconnect();
    printf("DISCONNECT_REQ rc=%d\n", rc);
    return rc == 0 ? 0 : 1;
}

static int cmd_conn(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    print_conn_info();
    return 0;
}

static int cmd_mtu(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uint16_t mtu = 0;
    int rc = blex_exchange_mtu(&mtu);
    printf("MTU_EXCHANGE rc=%d negotiated=%u effective=%u\n", rc, mtu, blex_mtu());
    return rc == 0 ? 0 : 1;
}

static int cmd_params(int argc, char **argv)
{
    if (argc != 5) {
        printf("usage: params <itvl_min> <itvl_max> <latency> <timeout_10ms>\n"
               "       intervals in 1.25 ms units, timeout in 10 ms units\n");
        return 1;
    }
    int rc = blex_update_params((uint16_t)atoi(argv[1]), (uint16_t)atoi(argv[2]),
                                (uint16_t)atoi(argv[3]), (uint16_t)atoi(argv[4]));
    printf("PARAMS_REQ rc=%d\n", rc);
    return rc == 0 ? 0 : 1;
}

static int cmd_sec(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    int rc = blex_security();
    printf("SECURITY rc=%d\n", rc);
    print_conn_info();
    return 0;
}

/* -------------------------------------------------------------------- GATT */

static void print_gatt(void)
{
    for (int i = 0; i < blex_svc_count(); i++) {
        const blex_svc_t *s = blex_svc(i);
        printf("SVC idx=%d start=0x%04x end=0x%04x uuid=", i, s->start_handle,
               s->end_handle);
        print_uuid(&s->uuid.u);
        printf("\n");

        for (int c = 0; c < blex_chr_count(); c++) {
            const blex_chr_t *ch = blex_chr(c);
            if (ch->svc_idx != i) {
                continue;
            }
            char props[8];
            blex_props_str(ch->properties, props);
            printf("CHR idx=%d svc=%d def=0x%04x val=0x%04x end=0x%04x cccd=0x%04x "
                   "props=%s(0x%02x) uuid=",
                   c, i, ch->def_handle, ch->val_handle, ch->end_handle,
                   ch->cccd_handle, props, ch->properties);
            print_uuid(&ch->uuid.u);
            printf("\n");

            for (int d = 0; d < blex_dsc_count(); d++) {
                const blex_dsc_t *ds = blex_dsc(d);
                if (ds->chr_idx != c) {
                    continue;
                }
                printf("DSC idx=%d chr=%d handle=0x%04x uuid=", d, c, ds->handle);
                print_uuid(&ds->uuid.u);
                printf("\n");
            }
        }
    }
    printf("GATT_END svcs=%d chrs=%d dscs=%d\n", blex_svc_count(),
           blex_chr_count(), blex_dsc_count());
}

static int cmd_discover(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    int64_t t0 = esp_timer_get_time();
    int rc = blex_discover();
    int64_t dt = esp_timer_get_time() - t0;
    if (rc != 0) {
        printf("DISCOVER_ERR rc=%d\n", rc);
        return 1;
    }
    printf("DISCOVER_OK took_ms=%" PRId64 "\n", dt / 1000);
    print_gatt();
    return 0;
}

static int cmd_gatt(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!blex_discovered()) {
        printf("ERR run 'discover' first\n");
        return 1;
    }
    print_gatt();
    return 0;
}

static void print_read_result(uint16_t handle, const char *what)
{
    size_t len = 0;
    uint16_t att = 0;
    int rc = blex_read(handle, s_buf, sizeof(s_buf), &len, &att);
    printf("RD handle=0x%04x %s rc=%d att=0x%04x len=%u data=", handle, what, rc,
           att, (unsigned)len);
    blex_print_hex(s_buf, len);
    printf(" ascii=\"");
    blex_print_ascii(s_buf, len);
    printf("\"\n");
}

static int cmd_read(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: read <handle>   (decimal, or 0x-prefixed hex)\n");
        return 1;
    }
    print_read_result((uint16_t)strtol(argv[1], NULL, 0), "");
    return 0;
}

static int cmd_readall(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!blex_discovered()) {
        printf("ERR run 'discover' first\n");
        return 1;
    }

    for (int c = 0; c < blex_chr_count(); c++) {
        const blex_chr_t *ch = blex_chr(c);
        if (!(ch->properties & BLE_GATT_CHR_PROP_READ)) {
            continue;
        }
        char uuid[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&ch->uuid.u, uuid);
        char what[80];
        snprintf(what, sizeof(what), "chr=%d uuid=%s%s%s", c, uuid,
                 uuid_name(&ch->uuid.u)[0] ? "/" : "", uuid_name(&ch->uuid.u));
        print_read_result(ch->val_handle, what);
    }

    /* User descriptions and presentation formats name the vendor fields, so
     * they are worth reading even though nothing subscribes to them. */
    for (int d = 0; d < blex_dsc_count(); d++) {
        const blex_dsc_t *ds = blex_dsc(d);
        if (!blex_uuid_is16(&ds->uuid.u, 0x2901) &&
            !blex_uuid_is16(&ds->uuid.u, 0x2904)) {
            continue;
        }
        char what[48];
        snprintf(what, sizeof(what), "dsc=%d", d);
        print_read_result(ds->handle, what);
    }
    printf("READALL_END\n");
    return 0;
}

static int cmd_write(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: write <handle> <hex> [nrsp]\n");
        return 1;
    }
    uint16_t handle = (uint16_t)strtol(argv[1], NULL, 0);
    size_t len = 0;
    if (blex_parse_hex(argv[2], s_buf, sizeof(s_buf), &len) != 0 || len == 0) {
        printf("ERR bad hex payload\n");
        return 1;
    }
    bool with_rsp = !(argc >= 4 && strcmp(argv[3], "nrsp") == 0);

    uint16_t att = 0;
    int rc = blex_write(handle, s_buf, len, with_rsp, &att);
    printf("WR handle=0x%04x len=%u mode=%s rc=%d att=0x%04x data=", handle,
           (unsigned)len, with_rsp ? "rsp" : "nrsp", rc, att);
    blex_print_hex(s_buf, len);
    printf("\n");
    return rc == 0 ? 0 : 1;
}

static int subscribe_one(int c, uint16_t value)
{
    const blex_chr_t *ch = blex_chr(c);
    if (ch->cccd_handle == 0) {
        return -1;
    }
    int rc = blex_subscribe(ch->cccd_handle, value);
    printf("SUB chr=%d val_handle=0x%04x cccd=0x%04x value=0x%04x rc=%d\n", c,
           ch->val_handle, ch->cccd_handle, value, rc);
    return rc;
}

static int cmd_sub(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: sub all [value] | sub <chr_idx> [value] | sub h <cccd_handle> [value]\n"
               "       value: 0 off, 1 notify (default), 2 indicate\n");
        return 1;
    }

    if (strcmp(argv[1], "h") == 0) {
        if (argc < 3) {
            printf("usage: sub h <cccd_handle> [value]\n");
            return 1;
        }
        uint16_t handle = (uint16_t)strtol(argv[2], NULL, 0);
        uint16_t value = (argc >= 4) ? (uint16_t)strtol(argv[3], NULL, 0) : 1;
        int rc = blex_subscribe(handle, value);
        printf("SUB cccd=0x%04x value=0x%04x rc=%d\n", handle, value, rc);
        return rc == 0 ? 0 : 1;
    }

    if (strcmp(argv[1], "all") == 0) {
        if (!blex_discovered()) {
            printf("ERR run 'discover' first\n");
            return 1;
        }
        uint16_t value = (argc >= 3) ? (uint16_t)strtol(argv[2], NULL, 0) : 0;
        int n = 0;
        for (int c = 0; c < blex_chr_count(); c++) {
            const blex_chr_t *ch = blex_chr(c);
            uint16_t v = value;
            if (argc < 3) {
                /* Pick per characteristic: notify when it can, else indicate. */
                if (ch->properties & BLE_GATT_CHR_PROP_NOTIFY) {
                    v = 1;
                } else if (ch->properties & BLE_GATT_CHR_PROP_INDICATE) {
                    v = 2;
                } else {
                    continue;
                }
            }
            if (subscribe_one(c, v) == 0) {
                n++;
            }
        }
        printf("SUBALL_END subscribed=%d\n", n);
        return 0;
    }

    int c = atoi(argv[1]);
    if (blex_chr(c) == NULL) {
        printf("ERR no such characteristic index\n");
        return 1;
    }
    uint16_t value = (argc >= 3) ? (uint16_t)strtol(argv[2], NULL, 0) : 1;
    return subscribe_one(c, value) == 0 ? 0 : 1;
}

/* ----------------------------------------------------------- notifications */

static int cmd_nlog(int argc, char **argv)
{
    if (argc >= 2) {
        blex_notify_log(strcmp(argv[1], "on") == 0);
    }
    if (argc >= 3) {
        blex_notify_filter((uint16_t)strtol(argv[2], NULL, 0));
    }
    printf("NLOG state=%s\n", blex_notify_log_get() ? "on" : "off");
    return 0;
}

static int cmd_nstat(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "reset") == 0) {
        blex_nstat_reset();
        printf("NSTAT_RESET\n");
        return 0;
    }
    for (int i = 0; i < blex_nstat_count(); i++) {
        const blex_nstat_t *st = blex_nstat(i);
        int64_t span_us = st->last_us - st->first_us;
        uint32_t rate_mhz = 0;   /* frames per 1000 s, avoids float formatting */
        if (span_us > 0 && st->count > 1) {
            rate_mhz = (uint32_t)(((uint64_t)(st->count - 1) * 1000000000ULL) / (uint64_t)span_us);
        }
        printf("NSTAT h=0x%04x count=%" PRIu32 " bytes=%" PRIu32
               " len=%u..%u span_ms=%" PRId64 " rate_mHz=%" PRIu32,
               st->handle, st->count, st->bytes, st->len_min, st->len_max,
               span_us / 1000, rate_mhz);
        blex_nstat_print_bytes(st);
        printf("\n");
    }
    printf("NSTAT_END entries=%d\n", blex_nstat_count());
    return 0;
}

static int cmd_rec(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: rec start | rec stop | rec <secs> | rec dump [max] | rec clear | rec stat\n");
        return 1;
    }
    if (strcmp(argv[1], "start") == 0) {
        blex_rec_start();
        printf("REC_START\n");
        return 0;
    }
    if (strcmp(argv[1], "stop") == 0) {
        blex_rec_stop();
        printf("REC_STOP frames=%" PRIu32 " bytes=%u\n", blex_rec_frames(),
               (unsigned)blex_rec_used());
        return 0;
    }
    if (strcmp(argv[1], "dump") == 0) {
        blex_rec_dump(argc >= 3 ? (uint32_t)strtoul(argv[2], NULL, 0) : 0);
        return 0;
    }
    if (strcmp(argv[1], "clear") == 0) {
        blex_rec_clear();
        printf("REC_CLEARED\n");
        return 0;
    }
    if (strcmp(argv[1], "stat") == 0) {
        printf("REC_STAT active=%d frames=%" PRIu32 " dropped=%" PRIu32
               " bytes=%u/%u\n", blex_rec_active(), blex_rec_frames(),
               blex_rec_dropped(), (unsigned)blex_rec_used(),
               (unsigned)blex_rec_capacity());
        return 0;
    }

    /* "rec <secs>": capture silently for a while, then report. Recording into
     * RAM instead of printing live is what keeps a fast notification stream
     * from overrunning the 115200 baud console. */
    int secs = atoi(argv[1]);
    if (secs <= 0) {
        printf("ERR bad duration\n");
        return 1;
    }
    blex_rec_start();
    printf("REC_START secs=%d\n", secs);
    vTaskDelay(pdMS_TO_TICKS(secs * 1000));
    blex_rec_stop();
    printf("REC_STOP frames=%" PRIu32 " dropped=%" PRIu32 " bytes=%u/%u\n",
           blex_rec_frames(), blex_rec_dropped(), (unsigned)blex_rec_used(),
           (unsigned)blex_rec_capacity());
    return 0;
}

/* ------------------------------------------------------------ Daly helpers */

static int cmd_crc(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: crc <hex> [init]   (init defaults to 0xffff, Fardriver uses 0x7f3c)\n");
        return 1;
    }
    size_t len = 0;
    if (blex_parse_hex(argv[1], s_buf, sizeof(s_buf), &len) != 0 || len == 0) {
        printf("ERR bad hex\n");
        return 1;
    }
    uint16_t init = (argc >= 3) ? (uint16_t)strtol(argv[2], NULL, 0) : 0xFFFF;
    uint16_t crc = wf_crc16(s_buf, len, init);
    printf("CRC init=0x%04x value=0x%04x le=%02x%02x be=%02x%02x\n", init, crc,
           crc & 0xff, crc >> 8, crc >> 8, crc & 0xff);
    return 0;
}

static int daly_send(uint16_t handle, uint8_t start, uint8_t function,
                     uint16_t address, uint16_t count)
{
    uint8_t frame[DALY_REQ_LEN];
    daly_build_request(frame, start, function, address, count);

    int rc = blex_write(handle, frame, sizeof(frame), false, NULL);
    printf("DALY_TX handle=0x%04x rc=%d frame=", handle, rc);
    blex_print_hex(frame, sizeof(frame));
    printf("\n");
    return rc;
}

/* Finds the value handle of a 16-bit characteristic UUID in the discovered
 * database, or 0 if it is not there. */
static uint16_t handle_of_uuid16(uint16_t uuid16)
{
    for (int c = 0; c < blex_chr_count(); c++) {
        const blex_chr_t *ch = blex_chr(c);
        if (blex_uuid_is16(&ch->uuid.u, uuid16)) {
            return ch->val_handle;
        }
    }
    return 0;
}

/* The same for the CCCD of that characteristic, 0 when it has none. */
static uint16_t cccd_of_uuid16(uint16_t uuid16)
{
    for (int c = 0; c < blex_chr_count(); c++) {
        const blex_chr_t *ch = blex_chr(c);
        if (blex_uuid_is16(&ch->uuid.u, uuid16)) {
            return ch->cccd_handle;
        }
    }
    return 0;
}

static int cmd_daly(int argc, char **argv)
{
    if (!blex_discovered()) {
        printf("ERR run 'discover' first\n");
        return 1;
    }
    uint16_t ctrl = handle_of_uuid16(0xFFF2);
    if (ctrl == 0) {
        printf("ERR characteristic 0xfff2 (daly control) not found\n");
        return 1;
    }

    if (argc >= 2 && strcmp(argv[1], "probe") == 0) {
        /* The two protocol variants differ only in the start byte. Send the
         * status request of each and let the notification statistics show
         * which one the BMS answered: variant D2 replies starting with 0xd2,
         * variant 0x81 replies starting with 0x51. */
        printf("DALY_PROBE ctrl_handle=0x%04x\n", ctrl);

        /* Both of these are what makes the probe self-contained rather than a
         * trap. The answer to a request arrives as a notification on 0xfff1,
         * so without the subscription the probe reports frames=0 however
         * correctly the BMS replied; and without the MTU exchange the 129 byte
         * answer is truncated at 20 bytes by the peer and the remainder is
         * lost rather than fragmented across further notifications. */
        uint16_t mtu = 0;
        int rc = blex_exchange_mtu(&mtu);
        printf("DALY_PROBE mtu_rc=%d negotiated=%u effective=%u\n", rc, mtu,
               blex_mtu());
        uint16_t cccd = cccd_of_uuid16(0xFFF1);
        if (cccd == 0) {
            printf("ERR characteristic 0xfff1 (daly notify) has no CCCD\n");
            return 1;
        }
        rc = blex_subscribe(cccd, 1);
        printf("DALY_PROBE sub cccd=0x%04x rc=%d\n", cccd, rc);
        if (rc != 0) {
            printf("ERR subscribe failed, the answers would go nowhere\n");
            return 1;
        }

        blex_nstat_reset();
        blex_rec_start();

        printf("DALY_PROBE step=d2_status\n");
        daly_send(ctrl, 0xD2, 0x03, 0x0000, 0x003E);
        vTaskDelay(pdMS_TO_TICKS(1500));
        printf("DALY_PROBE step=81_cells\n");
        daly_send(ctrl, 0x81, 0x03, 0x0000, 0x0040);
        vTaskDelay(pdMS_TO_TICKS(1500));
        printf("DALY_PROBE step=81_status\n");
        daly_send(ctrl, 0x81, 0x03, 0x0041, 0x003E);
        vTaskDelay(pdMS_TO_TICKS(1500));
        printf("DALY_PROBE step=81_version\n");
        daly_send(ctrl, 0x81, 0x03, 0x0178, 0x004A);
        vTaskDelay(pdMS_TO_TICKS(1500));

        blex_rec_stop();
        printf("DALY_PROBE_END frames=%" PRIu32 "\n", blex_rec_frames());
        return 0;
    }

    if (argc == 5 && strcmp(argv[1], "read") == 0) {
        uint8_t start = (uint8_t)strtol(argv[2], NULL, 0);
        uint16_t addr = (uint16_t)strtol(argv[3], NULL, 0);
        uint16_t count = (uint16_t)strtol(argv[4], NULL, 0);
        return daly_send(ctrl, start, 0x03, addr, count) == 0 ? 0 : 1;
    }

    printf("usage: daly probe\n"
           "       daly read <start_byte> <address> <register_count>\n"
           "       start_byte is 0xd2 for the D2 protocol, 0x81 for the DL one\n");
    return 1;
}

/* --------------------------------------------------------------- one-shot */

static int cmd_probe(int argc, char **argv)
{
    int secs = 10;
    if (argc >= 2) {
        secs = atoi(argv[1]);
    }
    if (secs <= 0) {
        secs = 10;
    }
    if (!blex_connected()) {
        printf("ERR not connected\n");
        return 1;
    }

    printf("PROBE_START record_secs=%d\n", secs);
    print_conn_info();

    uint16_t mtu = 0;
    int rc = blex_exchange_mtu(&mtu);
    printf("MTU_EXCHANGE rc=%d negotiated=%u effective=%u\n", rc, mtu, blex_mtu());

    if (blex_discover() != 0) {
        printf("PROBE_ABORT discover failed\n");
        return 1;
    }
    print_gatt();

    char *readall_argv[] = {"readall"};
    cmd_readall(1, readall_argv);

    blex_nstat_reset();
    char *sub_argv[] = {"sub", "all"};
    cmd_sub(2, sub_argv);

    blex_rec_start();
    printf("PROBE_RECORDING secs=%d\n", secs);
    vTaskDelay(pdMS_TO_TICKS(secs * 1000));
    blex_rec_stop();

    cmd_nstat(1, (char *[]){"nstat"});
    print_conn_info();
    printf("PROBE_END frames=%" PRIu32 " dropped=%" PRIu32
           " bytes=%u  (run 'rec dump' for the raw frames)\n",
           blex_rec_frames(), blex_rec_dropped(), (unsigned)blex_rec_used());
    return 0;
}

/* ------------------------------------------------------------ registration */

void cmd_ble_register(void)
{
    const esp_console_cmd_t cmds[] = {
        {.command = "scan", .help = "Scan for advertisers: scan [secs] [passive] [dedup]", .func = cmd_scan},
        {.command = "devs", .help = "List the devices seen so far", .func = cmd_devs},
        {.command = "dev", .help = "Decode one device's advertising data: dev <idx>", .func = cmd_dev},
        {.command = "scanclear", .help = "Empty the scan result table", .func = cmd_scanclear},
        {.command = "connect", .help = "Connect: connect <idx>|<mac> [public|random]|name <substr>", .func = cmd_connect},
        {.command = "disconnect", .help = "Terminate the connection", .func = cmd_disconnect},
        {.command = "conn", .help = "Print connection parameters, MTU, RSSI and security state", .func = cmd_conn},
        {.command = "mtu", .help = "Run an ATT MTU exchange", .func = cmd_mtu},
        {.command = "params", .help = "Request new connection parameters: params <min> <max> <lat> <to>", .func = cmd_params},
        {.command = "sec", .help = "Initiate pairing/encryption", .func = cmd_sec},
        {.command = "discover", .help = "Walk the whole GATT database", .func = cmd_discover},
        {.command = "gatt", .help = "Reprint the last GATT discovery", .func = cmd_gatt},
        {.command = "read", .help = "Read an attribute: read <handle>", .func = cmd_read},
        {.command = "readall", .help = "Read every readable characteristic and descriptor", .func = cmd_readall},
        {.command = "write", .help = "Write an attribute: write <handle> <hex> [nrsp]", .func = cmd_write},
        {.command = "sub", .help = "Subscribe: sub all [value] | sub <chr> [value] | sub h <cccd> [value]", .func = cmd_sub},
        {.command = "nlog", .help = "Live notification logging: nlog on|off [handle]", .func = cmd_nlog},
        {.command = "nstat", .help = "Per-handle notification statistics: nstat [reset]", .func = cmd_nstat},
        {.command = "rec", .help = "Record notifications: rec start|stop|<secs>|dump [max]|clear|stat", .func = cmd_rec},
        {.command = "crc", .help = "Modbus CRC16 of a hex string: crc <hex> [init]", .func = cmd_crc},
        {.command = "daly", .help = "Daly BMS request frames: daly probe | daly read <start> <addr> <count>", .func = cmd_daly},
        {.command = "probe", .help = "Full capture on the open connection: probe [secs]", .func = cmd_probe},
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}
