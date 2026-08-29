/*
 * capture_store - the on-flash log the standalone capture writes into.
 *
 * FAT with wear levelling on the "storage" partition, one file per capture,
 * named cap0001.wfl and upwards. The sequence number is derived from what is
 * already on the filesystem, so captures survive a power cycle and never
 * overwrite each other.
 *
 * Frames arrive on the NimBLE host task at ~36 Hz per link and a FAT write can
 * block for tens of milliseconds when it hits a wear-levelling sector move, so
 * store_write() only pushes into a RAM ring and a writer task drains it to the
 * file. Anything that does not fit is counted, never blocked on.
 *
 * File layout: one wflog_hdr_t, then a stream of wflog_rec_t, each immediately
 * followed by its len payload bytes. Everything is little endian, which is
 * what the ESP32 writes natively and what struct.unpack('<') on the host
 * reads. Timestamps are milliseconds since the start of the capture; the wall
 * clock is stored once, in the header.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define STORE_MOUNT     "/data"
#define STORE_LABEL     "storage"
#define STORE_MAX_FILES 64
#define WFLOG_MAGIC     "WFCAP1"
#define WFLOG_VERSION   1

/* Record types. */
enum {
    WFREC_MCU   = 0x01,  /* notification from the Fardriver controller */
    WFREC_BMS   = 0x02,  /* notification from the Daly BMS */
    WFREC_EVENT = 0x10,  /* ASCII text: connect, disconnect, reconnect, stop */
    WFREC_TELEM = 0x11,  /* board telemetry, see wflog_telem_t */
    WFREC_IMU   = 0x12,  /* MPU6886 sample, see wflog_imu_t */
};

typedef struct __attribute__((packed)) {
    char     magic[8];      /* "WFCAP1\0" */
    uint16_t version;       /* WFLOG_VERSION */
    uint16_t hdr_len;       /* sizeof(wflog_hdr_t), so the host can skip ahead */
    uint32_t seq;           /* capture number, matches the file name */
    int64_t  unix_start;    /* UTC seconds at t_ms = 0, or 0 if the RTC is unset */
    uint32_t boot_ms;       /* esp_timer milliseconds at t_ms = 0 */
    uint32_t reserved;
    char     mcu_addr[18];  /* "aa:bb:cc:dd:ee:ff", empty if never connected */
    char     bms_addr[18];
    char     note[32];      /* free text, currently the firmware version */
} wflog_hdr_t;

typedef struct __attribute__((packed)) {
    uint8_t  type;          /* WFREC_* */
    uint8_t  len;           /* payload bytes that follow */
    uint32_t t_ms;          /* milliseconds since the start of the capture */
} wflog_rec_t;

/* Payload of a WFREC_TELEM record, written every few seconds. */
typedef struct __attribute__((packed)) {
    uint16_t batt_mv;
    int8_t   rssi_mcu;
    int8_t   rssi_bms;
    uint32_t frames_mcu;
    uint32_t frames_bms;
    uint32_t dropped;
} wflog_telem_t;

/* Payload of a WFREC_IMU record, written at IMU_LOG_HZ while recording. The
 * frames alone cannot be interpreted - this is the independent movement
 * signal a candidate field gets correlated against. Raw counts at the fixed
 * full scales: 4096 LSB/g, 16.4 LSB/dps. */
typedef struct __attribute__((packed)) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} wflog_imu_t;

typedef struct {
    int      seq;
    char     name[32];      /* "cap0001.wfl" */
    uint32_t size;          /* bytes on disk */
    int64_t  unix_start;    /* from the file header, 0 if unknown */
    uint32_t duration_ms;   /* t_ms of the last record, 0 if unknown */
} store_entry_t;

/* Mounts the partition, formatting it if it has never been used. */
esp_err_t store_init(void);
bool      store_ready(void);
esp_err_t store_space(uint64_t *out_total, uint64_t *out_free);

/* ---- writing ----------------------------------------------------------- */

/* Opens the next free capture file and starts the writer task.
 * unix_start may be 0 when the RTC has no valid time. */
esp_err_t store_begin(int64_t unix_start, const char *note,
                      uint32_t *out_seq, char *out_name, size_t name_cap);
/* Patches the peer addresses into the header of the open capture. */
void store_set_addrs(const char *mcu_addr, const char *bms_addr);
/* Queues one record. Safe from the NimBLE host task; never blocks. Returns
 * false when the ring was full, which increments the drop counter. */
bool store_write(uint8_t type, uint32_t t_ms, const void *data, uint8_t len);
/* Flushes and closes the capture. */
esp_err_t store_end(void);
bool      store_active(void);
uint32_t  store_dropped(void);
uint64_t  store_bytes(void);   /* bytes written to the current capture */

/* ---- reading ------------------------------------------------------------ */

/* Fills out[] with what is on the filesystem, lowest sequence first.
 * Returns the number of entries, or a negative esp_err_t. */
int       store_list(store_entry_t *out, int max);
int       store_count(void);
esp_err_t store_stat(int seq, store_entry_t *out);
/* Builds "/data/cap0001.wfl" for a sequence number. */
void      store_path(int seq, char *out, size_t cap);
esp_err_t store_remove(int seq);
esp_err_t store_remove_all(void);
