/*
 * wfdecode - what the bytes in a Capture mean, as pure C99.
 *
 * ADR-0001 keeps only raw device output in a Capture, so the decoding has to
 * exist on both sides of it: on the Monitor for the live screen, and off the
 * bike for the archive. ADR-0002 says those two must not drift, so what each
 * byte means is declared once in field-table.json and the decoding itself is
 * generated from it into wf_fields.h/.c, which this header pulls in. What is
 * left here is the part that is not offset-and-scale and so cannot be
 * generated: the frame envelopes, the checksum, and the speed calculation
 * that spans two frame types.
 *
 * Everything here is a pure function over caller-owned state: no globals, no
 * allocation, no logging, no locking, no ESP-IDF and no FreeRTOS. The Monitor
 * owns the spinlock that guards its live state and calls wf_ctrl_apply()
 * inside it; the harness owns nothing and calls the same function on the
 * stack. That is what makes the seam testable without a board.
 *
 * The field meanings, with the Confidence of each, are docs/field-table.md
 * - itself generated from the same table.
 */
#ifndef WFDECODE_H
#define WFDECODE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* wf_ctrl_live_t, wf_bms_t, the frame-type constants and the field tables all
 * come from the Field Table. */
#include "wf_fields.h"

/* ------------------------------------------------------------------- CRC */

/* Modbus CRC-16: reflected polynomial 0xA001, no final xor, appended least
 * significant byte first. Both devices use it with a different initial value,
 * which is why the seed is a parameter. Run over a whole frame including its
 * own checksum it leaves a residue of 0. */
uint16_t wf_crc16(const uint8_t *data, size_t len, uint16_t init);

/* ------------------------------------------------------ Controller frames */

#define WF_CTRL_FRAME_LEN    16         /* aa <type> <12 payload> crc_lo crc_hi */
#define WF_CTRL_PAYLOAD_LEN  12
#define WF_CTRL_LEAD         0xaa
#define WF_CTRL_CRC_INIT     0x7f3c     /* the Controller's own seed */

/* One validated frame, split into its type and its payload. */
typedef struct {
    uint8_t type;
    uint8_t payload[WF_CTRL_PAYLOAD_LEN];
} wf_ctrl_frame_t;

/* True when data[0..len) is a well-formed Controller frame, in which case out
 * is filled in. Wrong length, wrong lead byte or a bad checksum all return
 * false and leave out alone: a torn or unrelated frame is discarded silently
 * rather than decoded into a plausible-looking lie. */
bool wf_ctrl_frame_parse(const uint8_t *data, size_t len, wf_ctrl_frame_t *out);

/* Folds one parsed frame into live: the generated fields, and then the speed,
 * which is not one of them. Zero-initialise live and hand it to this frame
 * after frame. Short enough to run inside a spinlock, which is where the
 * Monitor calls it from. */
void wf_ctrl_apply(wf_ctrl_live_t *live, const wf_ctrl_frame_t *frame);

/* Not in the Field Table, and deliberately: speed needs the rpm from frame
 * type 0xb0 and the wheel geometry from type 0xaf at once, which is not an
 * offset and a scale. This is blackTeaDisp's own conversion, wheel
 * circumference and the rpm->km/h factor folded into one constant, kept
 * exactly as it reads there rather than re-derived, since a rounding
 * "improvement" here would stop matching the dashboard this bike was designed
 * to be read by. */
float wf_ctrl_speed_kmh(uint16_t rpm, uint8_t wheel_radius, uint8_t wheel_width,
                        uint8_t wheel_ratio, uint16_t rate_ratio);

/* ------------------------------------------------------------- Odometer */

/* Per ADR-0003 the Odometer Anchors distance: it is coarse and it wraps, but
 * unlike integrated speed it does not drift. The Field Table keeps the raw u16
 * count the Controller sent, because everything below has to happen in counts
 * before it happens in metres.
 *
 * The scale is WF_CTRL_ODO_METRES_PER_COUNT, generated from the Field Table.
 * It is an unverified upstream claim - Ride 1 measures it - so it is one named
 * number in one place and these three functions are all that read it. */

/* Metres covered in total, as of this reading. Wraps with the count. */
uint32_t wf_ctrl_odo_metres(uint16_t counts);

/* Counts between two readings, wrap-safe: u16 modular subtraction, so 65530 to
 * 5 is 11 counts and not a 65525-count trip backwards. The Odometer only ever
 * counts up, which is what makes that the right answer; the flip side is that
 * this cannot tell a genuine step backwards from a nearly-complete wrap, so
 * difference readings in the order they were taken and nothing else. */
uint16_t wf_ctrl_odo_delta_counts(uint16_t from, uint16_t to);

/* The same difference, in metres. At u16 the count wraps every 65536 counts,
 * ~6553 km at 100 m each; taking this difference naively in a wider type is
 * what would put one 6553 km phantom trip into the archive. */
uint32_t wf_ctrl_odo_delta_metres(uint16_t from, uint16_t to);

/* ------------------------------------------------------- BMS responses */

#define WF_BMS_LEAD       0xd2      /* the protocol variant this unit answers on */
#define WF_BMS_FUNC_READ  0x03      /* read holding registers */
#define WF_BMS_CRC_INIT   0xffff

/* Decodes a 0xd2 response: d2 03 <byte count> <registers, big endian>
 * <crc_lo> <crc_hi>. Returns false - leaving out alone - for anything else:
 * a short frame, a different lead or function byte, an inconsistent byte
 * count, a bad checksum, more registers than WF_BMS_MAX_REGS, or fewer than
 * WF_BMS_REG_NEEDED, which is where the last assigned field lives. The BMS
 * also sends a shorter, still unidentified notification; that is one of the
 * things this rejects. */
bool wf_bms_decode(const uint8_t *data, size_t len, wf_bms_t *out);

#endif /* WFDECODE_H */
