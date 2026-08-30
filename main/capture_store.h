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
 * The file layout itself lives in wfdecode/wflog_format.h, which knows nothing
 * of ESP-IDF, so that the offline tools describe the same bytes rather than a
 * second guess at them.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "wfdecode/wflog_format.h"

#define STORE_MOUNT     "/data"
#define STORE_LABEL     "storage"
#define STORE_MAX_FILES 64

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
