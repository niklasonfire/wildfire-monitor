/*
 * ble_explorer - a host-drivable BLE central for reverse engineering the
 * Fardriver motor controller and the Daly BMS on the Wildfire motorbike.
 *
 * Everything here is deliberately protocol agnostic: it scans, connects,
 * walks the whole GATT database, reads every readable attribute, subscribes
 * to every notifiable one and records the raw frames with timestamps. The
 * console commands in cmd_ble.c drive it, so a script on the host can run a
 * full capture and parse the result offline.
 *
 * All the blocking calls below are meant to be issued from the console task.
 * They post the request to the NimBLE host and wait on a semaphore for the
 * matching callback, so only one may be in flight at a time.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "host/ble_gap.h"
#include "host/ble_uuid.h"

#define BLEX_MAX_DEVS      32
#define BLEX_MAX_SVCS      16
#define BLEX_MAX_CHRS      64
#define BLEX_MAX_DSCS      96
#define BLEX_MAX_NSTAT     32
#define BLEX_MAX_ATT_LEN   512

/* One advertiser seen during a scan. Advertising and scan response payloads
 * are kept raw; the parsed view is only ever printed, never relied upon. */
typedef struct {
    ble_addr_t addr;
    char       name[32];
    int8_t     rssi_last;
    int8_t     rssi_min;
    int8_t     rssi_max;
    uint8_t    evt_mask;      /* bit N set if BLE_HCI_ADV_RPT_EVTYPE_N seen */
    uint16_t   adv_count;
    uint16_t   rsp_count;
    int64_t    first_us;
    int64_t    last_us;
    int64_t    prev_adv_us;
    uint32_t   itvl_min_us;
    uint32_t   itvl_max_us;
    uint64_t   itvl_sum_us;
    uint32_t   itvl_n;
    uint8_t    adv_len;
    uint8_t    rsp_len;
    uint8_t    adv[31];
    uint8_t    rsp[31];
} blex_dev_t;

typedef struct {
    uint16_t       start_handle;
    uint16_t       end_handle;
    ble_uuid_any_t uuid;
} blex_svc_t;

typedef struct {
    int16_t        svc_idx;
    uint16_t       def_handle;
    uint16_t       val_handle;
    uint16_t       end_handle;    /* last handle owned by this characteristic */
    uint16_t       cccd_handle;   /* 0 when the characteristic has no CCCD */
    uint8_t        properties;
    ble_uuid_any_t uuid;
} blex_chr_t;

typedef struct {
    int16_t        chr_idx;
    uint16_t       handle;
    ble_uuid_any_t uuid;
} blex_dsc_t;

/* Per-attribute notification statistics. The two 256-bit sets record which
 * values the first and second payload byte ever took: for both target devices
 * those bytes are the frame type, so the set is effectively a list of the
 * message kinds the device emits. */
typedef struct {
    uint16_t handle;
    uint32_t count;
    uint32_t bytes;
    uint16_t len_min;
    uint16_t len_max;
    int64_t  first_us;
    int64_t  last_us;
    uint32_t b0_seen[8];
    uint32_t b1_seen[8];
} blex_nstat_t;

esp_err_t blex_init(void);
bool      blex_ready(void);

/* ---- scanning ---------------------------------------------------------- */

/* Runs a scan for duration_ms and returns once it has finished. Results
 * accumulate into the device table across calls until blex_scan_clear(). */
int  blex_scan(int duration_ms, bool passive, bool filter_duplicates);
void blex_scan_clear(void);
int  blex_dev_count(void);
const blex_dev_t *blex_dev(int idx);
int  blex_find_by_name(const char *substr);
int  blex_find_by_addr(const ble_addr_t *addr);

/* ---- connection -------------------------------------------------------- */

int  blex_connect(const ble_addr_t *addr, int timeout_ms);
int  blex_disconnect(void);
bool blex_connected(void);
uint16_t blex_conn_handle(void);
int  blex_conn_desc(struct ble_gap_conn_desc *out);
int  blex_exchange_mtu(uint16_t *out_mtu);
uint16_t blex_mtu(void);
int  blex_update_params(uint16_t itvl_min, uint16_t itvl_max,
                        uint16_t latency, uint16_t timeout_10ms);
int  blex_rssi(int8_t *out_rssi);
int  blex_security(void);
/* Last connection-parameter update the peer asked us for, 0 if it never did. */
void blex_peer_param_req(uint16_t *itvl_min, uint16_t *itvl_max,
                         uint16_t *latency, uint16_t *timeout, uint32_t *count);
int  blex_last_disconnect_reason(void);

/* ---- GATT -------------------------------------------------------------- */

int  blex_discover(void);
bool blex_discovered(void);
int  blex_svc_count(void);
int  blex_chr_count(void);
int  blex_dsc_count(void);
const blex_svc_t *blex_svc(int idx);
const blex_chr_t *blex_chr(int idx);
const blex_dsc_t *blex_dsc(int idx);
int  blex_chr_by_val_handle(uint16_t handle);

int  blex_read(uint16_t handle, uint8_t *out, size_t cap, size_t *out_len,
               uint16_t *out_att_status);
int  blex_write(uint16_t handle, const uint8_t *data, size_t len, bool with_rsp,
                uint16_t *out_att_status);
int  blex_subscribe(uint16_t cccd_handle, uint16_t value);

/* ---- notifications ----------------------------------------------------- */

void blex_notify_log(bool on);
bool blex_notify_log_get(void);
/* Only log/record this attribute handle; 0 means all of them. */
void blex_notify_filter(uint16_t handle);

void blex_rec_start(void);
void blex_rec_stop(void);
bool blex_rec_active(void);
size_t blex_rec_used(void);
size_t blex_rec_capacity(void);
uint32_t blex_rec_frames(void);
uint32_t blex_rec_dropped(void);
/* Prints the recording as "REC t_us=... h=0x.... len=... data=<hex>" lines. */
void blex_rec_dump(uint32_t max_frames);
void blex_rec_clear(void);

void blex_nstat_reset(void);
/* Prints the byte0/byte1 value sets of one statistics entry. */
void blex_nstat_print_bytes(const blex_nstat_t *st);
int  blex_nstat_count(void);
const blex_nstat_t *blex_nstat(int idx);

/* ---- formatting helpers shared with cmd_ble.c -------------------------- */

const char *blex_addr_type_str(uint8_t type);
const char *blex_evt_type_str(uint8_t evt_type);
/* Renders "aa:bb:cc:dd:ee:ff" into dst, which must hold 18 bytes. */
void blex_addr_str(const ble_addr_t *addr, char *dst);
/* Renders properties as a fixed "BRWwNIX" mask string into dst[8]. */
void blex_props_str(uint8_t props, char *dst);
void blex_print_hex(const uint8_t *data, size_t len);
void blex_print_ascii(const uint8_t *data, size_t len);
/* Decodes an AD structure list and prints one "  AD" line per element. */
void blex_print_ad(const uint8_t *data, size_t len, const char *prefix);
/* Parses "aa:bb:cc:dd:ee:ff" plus an optional "public"/"random" type. */
int  blex_parse_addr(const char *str, const char *type_str, ble_addr_t *out);
/* Parses a hex string, optionally with ':' or ' ' separators. */
int  blex_parse_hex(const char *str, uint8_t *out, size_t cap, size_t *out_len);
