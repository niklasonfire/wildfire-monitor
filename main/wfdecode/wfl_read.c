/* wfl_read - see wfl_read.h. Pure C99, no ESP-IDF, no stdio. */
#include "wfl_read.h"

#include <string.h>

#define WFL_REC_SIZE 6      /* type, len, t_ms - on disk, not sizeof() */

static uint16_t ld_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t ld_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int64_t ld_i64(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) {
        v = (v << 8) | p[i];
    }
    return (int64_t)v;
}

static void ld_str(char *dst, size_t cap, const uint8_t *p)
{
    memcpy(dst, p, cap);
    dst[cap - 1] = '\0';    /* the firmware pads with NUL, but never trust it */
}

bool wfl_open(wfl_reader_t *r, const void *buf, size_t len, wflog_hdr_t *hdr)
{
    if (r == NULL || buf == NULL) {
        return false;
    }
    const uint8_t *b = (const uint8_t *)buf;
    /* Packed, so sizeof() is the on-disk size on every target. */
    const size_t on_disk = sizeof(wflog_hdr_t);
    if (len < on_disk || memcmp(b, WFLOG_MAGIC, strlen(WFLOG_MAGIC)) != 0) {
        return false;
    }

    wflog_hdr_t h;
    memset(&h, 0, sizeof(h));
    memcpy(h.magic, b, sizeof(h.magic));
    h.magic[sizeof(h.magic) - 1] = '\0';
    h.version = ld_u16(b + 8);
    h.hdr_len = ld_u16(b + 10);
    h.seq = ld_u32(b + 12);
    h.unix_start = ld_i64(b + 16);
    h.boot_ms = ld_u32(b + 24);
    h.duration_ms = ld_u32(b + 28);
    ld_str(h.mcu_addr, sizeof(h.mcu_addr), b + 32);
    ld_str(h.bms_addr, sizeof(h.bms_addr), b + 50);
    ld_str(h.note, sizeof(h.note), b + 68);

    /* hdr_len is what the writer says its own header was, so a Capture from a
     * future firmware with extra header fields still walks correctly here. */
    size_t skip = h.hdr_len != 0 ? h.hdr_len : on_disk;
    if (skip < on_disk || skip > len) {
        return false;
    }

    r->buf = b;
    r->len = len;
    r->pos = skip;
    if (hdr != NULL) {
        *hdr = h;
    }
    return true;
}

bool wfl_next(wfl_reader_t *r, wfl_rec_t *out)
{
    if (r == NULL || out == NULL || r->buf == NULL) {
        return false;
    }
    if (r->pos + WFL_REC_SIZE > r->len) {
        return false;
    }
    const uint8_t *p = r->buf + r->pos;
    uint8_t type = p[0];
    uint8_t len = p[1];
    uint32_t t_ms = ld_u32(p + 2);
    if (r->pos + WFL_REC_SIZE + len > r->len) {
        return false;       /* truncated tail */
    }

    out->type = type;
    out->len = len;
    out->t_ms = t_ms;
    out->data = p + WFL_REC_SIZE;
    r->pos += WFL_REC_SIZE + len;
    return true;
}

bool wfl_telem(const wfl_rec_t *rec, wflog_telem_t *out)
{
    if (rec == NULL || out == NULL || rec->type != WFREC_TELEM || rec->len < 16) {
        return false;
    }
    const uint8_t *p = rec->data;
    out->batt_mv = ld_u16(p);
    out->rssi_mcu = (int8_t)p[2];
    out->rssi_bms = (int8_t)p[3];
    out->frames_mcu = ld_u32(p + 4);
    out->frames_bms = ld_u32(p + 8);
    out->dropped = ld_u32(p + 12);
    return true;
}

bool wfl_imu(const wfl_rec_t *rec, wflog_imu_t *out)
{
    if (rec == NULL || out == NULL || rec->type != WFREC_IMU || rec->len < 12) {
        return false;
    }
    const uint8_t *p = rec->data;
    out->ax = (int16_t)ld_u16(p);
    out->ay = (int16_t)ld_u16(p + 2);
    out->az = (int16_t)ld_u16(p + 4);
    out->gx = (int16_t)ld_u16(p + 6);
    out->gy = (int16_t)ld_u16(p + 8);
    out->gz = (int16_t)ld_u16(p + 10);
    return true;
}

bool wfl_is_marker(const wfl_rec_t *rec)
{
    const char *tag = "marker";
    size_t n = strlen(tag);
    if (rec == NULL || rec->type != WFREC_EVENT || rec->len < n) {
        return false;
    }
    return memcmp(rec->data, tag, n) == 0;
}
