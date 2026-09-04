#include "capture_store.h"

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "wear_levelling.h"

static const char *TAG = "capst";

/* The ring holds about ten seconds of both links at their combined ~72 rec/s,
 * which is far more than the writer ever needs to catch up on a wear-levelling
 * sector move. It is .bss rather than heap: the capture must not fail to start
 * because NimBLE and Wi-Fi fragmented the heap first. */
#define RING_BYTES     (12 * 1024)

#define DRAIN_BYTES    1024
#define DRAIN_PASSES   16          /* 16 KB per wakeup, more than the ring holds */
#define POLL_MS        20
#define SYNC_MS        2000        /* worst-case data loss on a power cut */
#define SYNC_BYTES     (8 * 1024)
#define MIN_FREE_BYTES (64 * 1024) /* stop recording rather than fill the disk */
#define WRITER_PRIO    4
#define WRITER_STACK   4096

#define MAX_REC_BYTES  (sizeof(wflog_rec_t) + 255)

/* --- mount state --- */
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
static bool s_ready;

/* --- the open capture --- */
static FILE *s_file;
static SemaphoreHandle_t s_file_mtx;    /* guards s_file and its position */
static SemaphoreHandle_t s_done_sem;    /* writer task signals it has finished */
static TaskHandle_t s_writer;           /* cleared by the task as it exits */
static int s_seq = -1;
static uint32_t s_boot_ms;              /* esp_timer milliseconds at t_ms = 0 */

static volatile bool s_active;          /* between store_begin and store_end */
static volatile bool s_stop_req;
static volatile bool s_full;            /* disk nearly full, records refused */
static volatile uint32_t s_last_t_ms;   /* t_ms of the newest queued record */

/* A hand-rolled ring rather than a FreeRTOS stream buffer, for two reasons.
 * Records come from two producers (the NimBLE host task for frames, the
 * capture task for events and telemetry) and a stream buffer is only safe for
 * one writer. And a zero-timeout xStreamBufferSend() writes a partial record
 * when only part of it fits, which would corrupt the stream; here a record
 * either goes in whole or is counted as a drop. The critical section is a few
 * microseconds of memcpy, so the host task never waits on flash. */
static uint8_t s_ring[RING_BYTES];
static size_t s_ring_head, s_ring_tail, s_ring_used;
static portMUX_TYPE s_ring_mux = portMUX_INITIALIZER_UNLOCKED;

/* Counters live under s_ring_mux so a 64-bit read cannot tear across tasks.
 * They survive store_end() and are only cleared by the next store_begin(). */
static uint32_t s_dropped;
static uint64_t s_bytes;

/* Only ever touched by the writer task. */
static uint8_t s_drain[DRAIN_BYTES];

/* ------------------------------------------------------------------- ring */

static bool ring_push(const uint8_t *p, size_t n)
{
    bool ok;

    portENTER_CRITICAL(&s_ring_mux);
    ok = (RING_BYTES - s_ring_used) >= n;
    if (ok) {
        size_t first = RING_BYTES - s_ring_head;
        if (first > n) {
            first = n;
        }
        memcpy(s_ring + s_ring_head, p, first);
        if (n > first) {
            memcpy(s_ring, p + first, n - first);
        }
        s_ring_head = (s_ring_head + n) % RING_BYTES;
        s_ring_used += n;
    } else {
        s_dropped++;
    }
    portEXIT_CRITICAL(&s_ring_mux);
    return ok;
}

static size_t ring_pop(uint8_t *p, size_t cap)
{
    size_t n;

    portENTER_CRITICAL(&s_ring_mux);
    n = s_ring_used < cap ? s_ring_used : cap;
    if (n > 0) {
        size_t first = RING_BYTES - s_ring_tail;
        if (first > n) {
            first = n;
        }
        memcpy(p, s_ring + s_ring_tail, first);
        if (n > first) {
            memcpy(p + first, s_ring, n - first);
        }
        s_ring_tail = (s_ring_tail + n) % RING_BYTES;
        s_ring_used -= n;
    }
    portEXIT_CRITICAL(&s_ring_mux);
    return n;
}

static void ring_reset(void)
{
    portENTER_CRITICAL(&s_ring_mux);
    s_ring_head = s_ring_tail = s_ring_used = 0;
    s_dropped = 0;
    s_bytes = 0;
    portEXIT_CRITICAL(&s_ring_mux);
}

static void count_drop(void)
{
    portENTER_CRITICAL(&s_ring_mux);
    s_dropped++;
    portEXIT_CRITICAL(&s_ring_mux);
}

static void count_bytes(size_t n)
{
    portENTER_CRITICAL(&s_ring_mux);
    s_bytes += n;
    portEXIT_CRITICAL(&s_ring_mux);
}

/* -------------------------------------------------------------- file names */

/* Returns the sequence number of "cap0001.wfl", or -1 for anything else. The
 * Wi-Fi mode lets the user drop files on the share, so foreign names and
 * subdirectories have to be ignored rather than tripping the scan up. */
static int parse_seq(const char *name)
{
    int seq = 0;

    if (strncasecmp(name, "cap", 3) != 0) {
        return -1;
    }
    for (int i = 3; i < 7; i++) {
        if (name[i] < '0' || name[i] > '9') {
            return -1;
        }
        seq = seq * 10 + (name[i] - '0');
    }
    if (strcasecmp(name + 7, ".wfl") != 0) {
        return -1;
    }
    return seq > 0 ? seq : -1;
}

void store_path(int seq, char *out, size_t cap)
{
    if (out == NULL || cap == 0) {
        return;
    }
    snprintf(out, cap, "%s/cap%04d.wfl", STORE_MOUNT, seq);
}

/* Walks the directory once. With out == NULL it only counts; otherwise it
 * fills out[] with up to max sequence numbers in readdir order. */
static int scan_dir(int *out, int max)
{
    DIR *dir = opendir(STORE_MOUNT);
    struct dirent *de;
    int n = 0;

    if (dir == NULL) {
        ESP_LOGE(TAG, "opendir %s failed: %s", STORE_MOUNT, strerror(errno));
        return -1;
    }
    while ((de = readdir(dir)) != NULL) {
        int seq = parse_seq(de->d_name);
        if (seq < 0) {
            continue;
        }
        if (out != NULL) {
            if (n >= max) {
                ESP_LOGW(TAG, "more than %d captures, listing the first %d", max, max);
                break;
            }
            out[n] = seq;
        }
        n++;
    }
    closedir(dir);
    return n;
}

static int scan_max_seq(void)
{
    DIR *dir = opendir(STORE_MOUNT);
    struct dirent *de;
    int max = 0;

    if (dir == NULL) {
        ESP_LOGE(TAG, "opendir %s failed: %s", STORE_MOUNT, strerror(errno));
        return -1;
    }
    while ((de = readdir(dir)) != NULL) {
        int seq = parse_seq(de->d_name);
        if (seq > max) {
            max = seq;
        }
    }
    closedir(dir);
    return max;
}

/* ------------------------------------------------------------------- mount */

esp_err_t store_init(void)
{
    esp_vfs_fat_mount_config_t cfg = {
        .format_if_mount_failed = true,
        .max_files = 4,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
    };
    esp_err_t err;

    if (s_ready) {
        return ESP_OK;
    }
    if (s_file_mtx == NULL) {
        s_file_mtx = xSemaphoreCreateMutex();
        s_done_sem = xSemaphoreCreateBinary();
        if (s_file_mtx == NULL || s_done_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    err = esp_vfs_fat_spiflash_mount_rw_wl(STORE_MOUNT, STORE_LABEL, &cfg, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mount %s failed: %s", STORE_LABEL, esp_err_to_name(err));
        return err;
    }
    s_ready = true;

    uint64_t total = 0, freeb = 0;
    if (store_space(&total, &freeb) == ESP_OK) {
        ESP_LOGI(TAG, "mounted %s at %s, %" PRIu64 " KB free of %" PRIu64 " KB",
                 STORE_LABEL, STORE_MOUNT, freeb / 1024, total / 1024);
    }
    return ESP_OK;
}

bool store_ready(void)
{
    return s_ready;
}

esp_err_t store_space(uint64_t *out_total, uint64_t *out_free)
{
    uint64_t total = 0, freeb = 0;
    esp_err_t err;

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    err = esp_vfs_fat_info(STORE_MOUNT, &total, &freeb);
    if (err != ESP_OK) {
        return err;
    }
    if (out_total != NULL) {
        *out_total = total;
    }
    if (out_free != NULL) {
        *out_free = freeb;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------ writer task */

/* Appends one record straight to the file. Only the writer task may call this;
 * everything else goes through the ring. */
static void write_direct(uint8_t type, const void *data, uint8_t len)
{
    wflog_rec_t rec = {
        .type = type,
        .len = len,
        .t_ms = (uint32_t)(esp_timer_get_time() / 1000) - s_boot_ms,
    };
    size_t w = 0;

    xSemaphoreTake(s_file_mtx, portMAX_DELAY);
    if (s_file != NULL) {
        w = fwrite(&rec, 1, sizeof(rec), s_file);
        if (len > 0) {
            w += fwrite(data, 1, len, s_file);
        }
    }
    xSemaphoreGive(s_file_mtx);
    count_bytes(w);
}

/* Moves everything the ring holds into the file. Returns the bytes written. */
static size_t drain_pass(void)
{
    size_t total = 0;

    for (int i = 0; i < DRAIN_PASSES; i++) {
        size_t n = ring_pop(s_drain, sizeof(s_drain));
        size_t w;

        if (n == 0) {
            break;
        }
        xSemaphoreTake(s_file_mtx, portMAX_DELAY);
        w = s_file != NULL ? fwrite(s_drain, 1, n, s_file) : 0;
        xSemaphoreGive(s_file_mtx);
        if (w != n) {
            ESP_LOGE(TAG, "short write, %u of %u bytes", (unsigned)w, (unsigned)n);
        }
        total += w;
    }
    if (total > 0) {
        count_bytes(total);
    }
    return total;
}

static void sync_file(void)
{
    xSemaphoreTake(s_file_mtx, portMAX_DELAY);
    if (s_file != NULL) {
        fflush(s_file);
        fsync(fileno(s_file));
    }
    xSemaphoreGive(s_file_mtx);
}

/* Stops accepting records once the partition is nearly full, leaving the file
 * valid and closed off with an event the host can see. */
static void check_space(void)
{
    uint64_t freeb = 0;

    if (s_full || store_space(NULL, &freeb) != ESP_OK) {
        return;
    }
    if (freeb >= MIN_FREE_BYTES) {
        return;
    }
    s_full = true;              /* producers stop here, so the ring stops growing */
    drain_pass();               /* everything already queued still belongs in the file */
    write_direct(WFREC_EVENT, "storage full", 12);
    sync_file();
    ESP_LOGW(TAG, "only %" PRIu64 " bytes free, capture %d stopped recording",
             freeb, s_seq);
}

static void writer_task(void *arg)
{
    TickType_t last_sync = xTaskGetTickCount();
    size_t unsynced = 0;

    (void)arg;
    while (1) {
        bool stopping;

        unsynced += drain_pass();
        /* Read the stop flag after the drain, then drain once more, so a record
         * queued just before store_end() still lands in the file. */
        stopping = s_stop_req;
        if (stopping) {
            unsynced += drain_pass();
        }

        TickType_t now = xTaskGetTickCount();
        if (stopping || unsynced >= SYNC_BYTES ||
            (now - last_sync) >= pdMS_TO_TICKS(SYNC_MS)) {
            if (unsynced > 0) {
                sync_file();
                unsynced = 0;
            }
            last_sync = now;
            if (!stopping) {
                check_space();   /* no point closing the file off twice */
            }
        }

        if (stopping) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }

    s_writer = NULL;
    xSemaphoreGive(s_done_sem);
    vTaskDelete(NULL);
}

/* ---------------------------------------------------------------- writing */

esp_err_t store_begin(int64_t unix_start, const char *note,
                      uint32_t *out_seq, char *out_name, size_t name_cap)
{
    char path[64];
    wflog_hdr_t hdr;
    uint64_t freeb = 0;
    int seq;

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_active || s_file != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (store_space(NULL, &freeb) == ESP_OK && freeb < MIN_FREE_BYTES) {
        ESP_LOGE(TAG, "only %" PRIu64 " bytes free, refusing to start", freeb);
        return ESP_ERR_NO_MEM;
    }

    seq = scan_max_seq();
    if (seq < 0) {
        return ESP_FAIL;
    }
    seq++;
    if (seq > 9999) {
        ESP_LOGE(TAG, "sequence numbers exhausted, delete some captures");
        return ESP_ERR_NO_MEM;
    }
    store_path(seq, path, sizeof(path));

    s_file = fopen(path, "wb");
    if (s_file == NULL) {
        ESP_LOGE(TAG, "fopen %s failed: %s", path, strerror(errno));
        return ESP_FAIL;
    }

    s_seq = seq;
    s_boot_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_full = false;
    s_stop_req = false;
    s_last_t_ms = 0;
    ring_reset();

    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.magic, WFLOG_MAGIC, sizeof(WFLOG_MAGIC));
    hdr.version = WFLOG_VERSION;
    hdr.hdr_len = sizeof(wflog_hdr_t);
    hdr.seq = (uint32_t)seq;
    hdr.unix_start = unix_start;
    hdr.boot_ms = s_boot_ms;
    hdr.duration_ms = 0;   /* patched by store_end(), see below */
    snprintf(hdr.note, sizeof(hdr.note), "%s", note != NULL ? note : "");

    if (fwrite(&hdr, 1, sizeof(hdr), s_file) != sizeof(hdr)) {
        ESP_LOGE(TAG, "header write failed on %s", path);
        fclose(s_file);
        s_file = NULL;
        unlink(path);
        return ESP_FAIL;
    }
    /* Flush the header before anything else, so a power cut one second in still
     * leaves a file the host can parse. */
    fflush(s_file);
    fsync(fileno(s_file));
    count_bytes(sizeof(hdr));

    /* Drop a stale give from a writer that finished after store_end() returned. */
    xSemaphoreTake(s_done_sem, 0);

    if (xTaskCreate(writer_task, "capwr", WRITER_STACK, NULL, WRITER_PRIO,
                    &s_writer) != pdPASS) {
        ESP_LOGE(TAG, "cannot start the writer task");
        fclose(s_file);
        s_file = NULL;
        unlink(path);
        return ESP_ERR_NO_MEM;
    }
    s_active = true;

    if (out_seq != NULL) {
        *out_seq = (uint32_t)seq;
    }
    if (out_name != NULL && name_cap > 0) {
        snprintf(out_name, name_cap, "cap%04d.wfl", seq);
    }
    ESP_LOGI(TAG, "capture %d open at %s, unix_start %" PRId64, seq, path, unix_start);
    return ESP_OK;
}

void store_set_addrs(const char *mcu_addr, const char *bms_addr)
{
    char buf[18];

    if (!s_active || s_file_mtx == NULL) {
        return;
    }
    /* The writer task owns the file position between records, so the patch has
     * to happen with the mutex held and the position put back afterwards. */
    xSemaphoreTake(s_file_mtx, portMAX_DELAY);
    if (s_file != NULL) {
        if (mcu_addr != NULL) {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "%s", mcu_addr);
            fseek(s_file, (long)offsetof(wflog_hdr_t, mcu_addr), SEEK_SET);
            fwrite(buf, 1, sizeof(buf), s_file);
        }
        if (bms_addr != NULL) {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "%s", bms_addr);
            fseek(s_file, (long)offsetof(wflog_hdr_t, bms_addr), SEEK_SET);
            fwrite(buf, 1, sizeof(buf), s_file);
        }
        fseek(s_file, 0, SEEK_END);
        fflush(s_file);
        fsync(fileno(s_file));
    }
    xSemaphoreGive(s_file_mtx);
}

bool store_write(uint8_t type, uint32_t t_ms, const void *data, uint8_t len)
{
    uint8_t buf[MAX_REC_BYTES];
    wflog_rec_t rec;

    if (!s_active) {
        return false;   /* not recording is normal, not a loss */
    }
    if (s_full) {
        count_drop();
        return false;
    }
    if (len > 0 && data == NULL) {
        count_drop();
        return false;
    }

    rec.type = type;
    rec.len = len;
    rec.t_ms = t_ms;
    memcpy(buf, &rec, sizeof(rec));
    if (len > 0) {
        memcpy(buf + sizeof(rec), data, len);
    }
    if (!ring_push(buf, sizeof(rec) + len)) {
        return false;   /* ring_push counted the drop under the same lock */
    }
    s_last_t_ms = t_ms;
    return true;
}

esp_err_t store_end(void)
{
    if (!s_active) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Stop the producers first, then let the writer finish what is queued. */
    s_active = false;
    s_stop_req = true;

    /* Waiting without a timeout is deliberate: closing the file while the
     * writer still holds the FILE* would be a use-after-free, and the writer's
     * only blocking call is a bounded flash write. The task gives the
     * semaphore exactly once per capture, so this take always pairs up. */
    xSemaphoreTake(s_done_sem, portMAX_DELAY);

    xSemaphoreTake(s_file_mtx, portMAX_DELAY);
    if (s_file != NULL) {
        /* Patching the duration in here is far cheaper than making the host
         * walk to the last record just to learn how long the ride was; it
         * stays 0 when the bike cut power mid-capture, which is the documented
         * "unknown". */
        uint32_t duration = s_last_t_ms;
        fseek(s_file, (long)offsetof(wflog_hdr_t, duration_ms), SEEK_SET);
        fwrite(&duration, 1, sizeof(duration), s_file);
        fseek(s_file, 0, SEEK_END);
        fflush(s_file);
        fsync(fileno(s_file));
        fclose(s_file);
        s_file = NULL;
    }
    xSemaphoreGive(s_file_mtx);

    ESP_LOGI(TAG, "capture %d closed, %" PRIu64 " bytes, %" PRIu32 " dropped, %"
             PRIu32 " ms", s_seq, store_bytes(), store_dropped(), s_last_t_ms);
    return ESP_OK;
}

bool store_active(void)
{
    return s_active;
}

uint32_t store_dropped(void)
{
    uint32_t n;

    portENTER_CRITICAL(&s_ring_mux);
    n = s_dropped;
    portEXIT_CRITICAL(&s_ring_mux);
    return n;
}

uint64_t store_bytes(void)
{
    uint64_t n;

    portENTER_CRITICAL(&s_ring_mux);
    n = s_bytes;
    portEXIT_CRITICAL(&s_ring_mux);
    return n;
}

/* ---------------------------------------------------------------- reading */

esp_err_t store_stat(int seq, store_entry_t *out)
{
    char path[64];
    struct stat st;
    FILE *f;

    if (out == NULL || seq <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    store_path(seq, path, sizeof(path));
    if (stat(path, &st) != 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memset(out, 0, sizeof(*out));
    out->seq = seq;
    snprintf(out->name, sizeof(out->name), "cap%04d.wfl", seq);
    out->size = (uint32_t)st.st_size;

    f = fopen(path, "rb");
    if (f != NULL) {
        wflog_hdr_t hdr;
        if (fread(&hdr, 1, sizeof(hdr), f) == sizeof(hdr) &&
            memcmp(hdr.magic, WFLOG_MAGIC, strlen(WFLOG_MAGIC)) == 0) {
            out->unix_start = hdr.unix_start;
            out->duration_ms = hdr.duration_ms;
        }
        fclose(f);
    }
    /* The open capture has not had its duration patched in yet. */
    if (s_active && seq == s_seq) {
        out->duration_ms = s_last_t_ms;
    }
    return ESP_OK;
}

int store_list(store_entry_t *out, int max)
{
    int seqs[STORE_MAX_FILES];
    int n, kept = 0;

    if (!s_ready) {
        return -ESP_ERR_INVALID_STATE;  /* the contract wants a negative code */
    }
    if (out == NULL || max <= 0) {
        return 0;
    }
    n = scan_dir(seqs, STORE_MAX_FILES);
    if (n < 0) {
        return ESP_FAIL;
    }

    /* Insertion sort: n is at most STORE_MAX_FILES and the list is nearly
     * sorted already, because readdir walks the directory in creation order. */
    for (int i = 1; i < n; i++) {
        int v = seqs[i], j = i - 1;
        while (j >= 0 && seqs[j] > v) {
            seqs[j + 1] = seqs[j];
            j--;
        }
        seqs[j + 1] = v;
    }

    for (int i = 0; i < n && kept < max; i++) {
        if (store_stat(seqs[i], &out[kept]) == ESP_OK) {
            kept++;
        }
    }
    return kept;
}

int store_count(void)
{
    if (!s_ready) {
        return -ESP_ERR_INVALID_STATE;
    }
    return scan_dir(NULL, 0);
}

esp_err_t store_remove(int seq)
{
    char path[64];

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (seq <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_active && seq == s_seq) {
        return ESP_ERR_INVALID_STATE;
    }
    store_path(seq, path, sizeof(path));
    if (unlink(path) != 0) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "removed capture %d", seq);
    return ESP_OK;
}

esp_err_t store_remove_all(void)
{
    int seqs[STORE_MAX_FILES];
    int n, removed = 0;

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_active) {
        return ESP_ERR_INVALID_STATE;   /* never delete the file being written */
    }
    n = scan_dir(seqs, STORE_MAX_FILES);
    if (n < 0) {
        return ESP_FAIL;
    }
    for (int i = 0; i < n; i++) {
        if (store_remove(seqs[i]) == ESP_OK) {
            removed++;
        }
    }
    ESP_LOGI(TAG, "removed %d of %d captures", removed, n);
    return removed == n ? ESP_OK : ESP_FAIL;
}
