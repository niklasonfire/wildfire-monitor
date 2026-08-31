/*
 * wflog_format - the on-disk layout of a Capture (.wfl), and nothing else.
 *
 * Split out of capture_store.h so that both sides of ADR-0001 can describe the
 * same bytes: the Monitor, which writes Captures over ESP-IDF's FAT, and the
 * offline tools, which read them on a development machine with no ESP-IDF at
 * all. capture_store.h includes this; anything that only needs to understand
 * the file includes it directly and pays for nothing else.
 *
 * File layout: one wflog_hdr_t, then a stream of wflog_rec_t, each immediately
 * followed by its len payload bytes. Everything is little endian, which is
 * what the ESP32 writes natively and what struct.unpack('<') on the host
 * reads. Timestamps are milliseconds since the start of the Capture; the wall
 * clock is stored once, in the header.
 *
 * Plain C99: no ESP-IDF, no FreeRTOS, no allocation.
 */
#ifndef WFLOG_FORMAT_H
#define WFLOG_FORMAT_H

#include <stdint.h>

#define WFLOG_MAGIC     "WFCAP1"
#define WFLOG_VERSION   1

/* Record types. */
enum {
    WFREC_MCU   = 0x01,  /* notification from the Controller */
    WFREC_BMS   = 0x02,  /* notification from the BMS */
    WFREC_EVENT = 0x10,  /* ASCII text: connect, disconnect, reconnect, stop.
                          * A Marker is one of these whose text starts with
                          * "marker" - it is not a record type of its own. */
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
    uint32_t duration_ms;   /* t_ms of the last record, back-patched by
                             * store_end(); 0 when the capture was never
                             * closed - power cut, or rebuilt from a console
                             * dump, which does not carry it */
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

#endif /* WFLOG_FORMAT_H */
