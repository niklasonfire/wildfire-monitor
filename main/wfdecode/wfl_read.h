/*
 * wfl_read - walking a Capture file, as pure C99.
 *
 * Reads out of a buffer the caller already holds rather than off a FILE*, so
 * the same code serves the Monitor (which has the file on FAT) and the replay
 * harness (which slurps a fixture into memory). No allocation, no stdio, no
 * ESP-IDF.
 *
 * Fields are pulled out byte by byte rather than by casting the buffer to
 * wflog_hdr_t: a Capture is little endian on disk whatever the machine
 * reading it is, and the packed layout has no alignment to lean on.
 */
#ifndef WFL_READ_H
#define WFL_READ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wflog_format.h"

typedef struct {
    const uint8_t *buf;
    size_t         len;
    size_t         pos;     /* offset of the next record */
} wfl_reader_t;

/* One record, pointing into the caller's buffer. */
typedef struct {
    uint8_t        type;    /* WFREC_* */
    uint8_t        len;
    uint32_t       t_ms;
    const uint8_t *data;    /* len bytes, inside the reader's buffer */
} wfl_rec_t;

/* Validates the magic and the header length, fills hdr (may be NULL) and
 * leaves the reader positioned at the first record. False means this is not a
 * Capture, or is too short to hold a header. */
bool wfl_open(wfl_reader_t *r, const void *buf, size_t len, wflog_hdr_t *hdr);

/* Next record, or false at the end. A trailing partial record - what a bike
 * that cut power mid-write leaves behind - ends the walk as cleanly as a
 * proper end of file, because that tail is the one thing a Capture can be
 * expected to have. */
bool wfl_next(wfl_reader_t *r, wfl_rec_t *out);

/* Payload accessors, for records whose payload is a packed struct. False when
 * the record is of the wrong type or too short to hold one. */
bool wfl_telem(const wfl_rec_t *rec, wflog_telem_t *out);
bool wfl_imu(const wfl_rec_t *rec, wflog_imu_t *out);

/* True when a WFREC_EVENT record is a Marker: the rider-pressed timestamp is
 * an event whose text starts with "marker", not a record type of its own. */
bool wfl_is_marker(const wfl_rec_t *rec);

#endif /* WFL_READ_H */
