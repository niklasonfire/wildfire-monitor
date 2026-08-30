/*
 * wfdecode - what the bytes in a Capture mean, as pure C99.
 *
 * ADR-0001 keeps only raw device output in a Capture, so the decoding has to
 * exist on both sides of it: on the Monitor for the live screen, and off the
 * bike for the archive. ADR-0002 says those two must not drift. This is the
 * one copy in C - the Monitor calls it, and so does the replay harness that
 * runs a recorded Capture on a development machine.
 *
 * Everything here is a pure function over caller-owned state: no globals, no
 * allocation, no logging, no locking, no ESP-IDF and no FreeRTOS. The Monitor
 * owns the spinlock that guards its live state and calls wf_ctrl_apply()
 * inside it; the harness owns nothing and calls the same function on the
 * stack. That is what makes the seam testable without a board.
 *
 * The field meanings are docs/fardriver-fields.md, CONFIRMED/SOUND entries
 * only.
 */
#ifndef WFDECODE_H
#define WFDECODE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------- CRC */

/* Modbus CRC-16: reflected polynomial 0xA001, no final xor, appended least
 * significant byte first. Both devices use it with a different initial value,
 * which is why the seed is a parameter. Run over a whole frame including its
 * own checksum it leaves a residue of 0. */
uint16_t wf_crc16(const uint8_t *data, size_t len, uint16_t init);

/* ------------------------------------------------------------ Controller */

#define WF_CTRL_FRAME_LEN    16         /* aa <type> <12 payload> crc_lo crc_hi */
#define WF_CTRL_PAYLOAD_LEN  12
#define WF_CTRL_LEAD         0xaa
#define WF_CTRL_CRC_INIT     0x7f3c     /* the Controller's own seed */

/* Frame types this decoder knows. The Controller cycles through 55 of them. */
#define WF_CTRL_TYPE_MOTION  0xb0
#define WF_CTRL_TYPE_WHEEL   0xaf
#define WF_CTRL_TYPE_TEMP    0xb5
#define WF_CTRL_TYPE_ODO     0x94

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

/* The Controller's CONFIRMED/SOUND fields, decoded from every frame. Grouped
 * by the frame type that carries them, each with its own "seen at least once"
 * flag: the types arrive at very different rates (0xb0 at the link's full
 * ~35.5 Hz, 0xaf/0xb5/0x94 far slower), so one flag per group is what lets a
 * screen show "--" for, say, engine_temp while gear and speed are already
 * live. Zero-initialise it and hand it to wf_ctrl_apply() frame after frame. */
typedef struct {
    bool     b0_valid;             /* type 0xb0 seen at least once */
    uint8_t  gear;                 /* 0/1/2 = eco/standard/sport */
    bool     sliding_backwards;
    bool     motion;
    bool     brake_switch;
    uint16_t cur_rpm;

    bool     af_valid;             /* type 0xaf seen at least once */
    uint8_t  wheel_ratio;
    uint8_t  wheel_radius;
    uint8_t  avg_speed_kmh;
    uint8_t  wheel_width;
    uint16_t rate_ratio;

    bool     speed_valid;          /* b0_valid && af_valid && rate_ratio != 0 */
    float    cur_speed_kmh;

    bool     b5_valid;             /* type 0xb5 seen at least once */
    int16_t  engine_temp;

    bool     odo_valid;            /* type 0x94 seen at least once */
    uint16_t odometer_raw;
} wf_ctrl_live_t;

/* Folds one parsed frame into live. Types this table does not cover are
 * ignored. Short enough to run inside a spinlock, which is where the Monitor
 * calls it from. */
void wf_ctrl_apply(wf_ctrl_live_t *live, const wf_ctrl_frame_t *frame);

/* blackTeaDisp's own conversion: wheel circumference and the rpm->km/h factor
 * folded into one constant, kept exactly as it reads there rather than
 * re-derived, since a rounding "improvement" here would stop matching the
 * dashboard this bike was designed to be read by. */
float wf_ctrl_speed_kmh(uint16_t rpm, uint8_t wheel_radius, uint8_t wheel_width,
                        uint8_t wheel_ratio, uint16_t rate_ratio);

/* ------------------------------------------------------------------- BMS */

#define WF_BMS_LEAD       0xd2      /* the protocol variant this unit answers on */
#define WF_BMS_FUNC_READ  0x03      /* read holding registers */
#define WF_BMS_CRC_INIT   0xffff
#define WF_BMS_MAX_REGS   62        /* what CAP_BMS_POLL_COUNT asks for */
#define WF_BMS_MAX_CELLS  28

/* The last register this decoder reads. A response that stops short of it
 * carries none of the fields below, so it is rejected outright. */
#define WF_BMS_REG_NEEDED 56

/* One decoded 0xd2 read-holding-registers response. Register indices and
 * scales are docs/fardriver-fields.md, "Daly BMS fields". */
typedef struct {
    uint16_t reg[WF_BMS_MAX_REGS];  /* the raw registers, unassigned ones too */
    uint8_t  n_reg;

    uint16_t cell_mv[WF_BMS_MAX_CELLS];  /* reg 0-27 */
    uint16_t cell_count;                 /* reg 49 and 51, redundant */

    float    pack_v;        /* reg 40 / 10 */
    float    current_a;     /* (reg 41 - 30000) / 10, positive = discharge */
    float    soc_pct;       /* reg 42 / 10 */
    uint16_t cell_max_mv;   /* reg 43 */
    uint16_t cell_min_mv;   /* reg 44 */
    int16_t  temp_hi_c;     /* reg 45 - 40 */
    int16_t  temp_lo_c;     /* reg 46 - 40 */
    uint16_t avg_cell_mv;   /* reg 55 */
} wf_bms_t;

/* Decodes a 0xd2 response: d2 03 <byte count> <registers, big endian>
 * <crc_lo> <crc_hi>. Returns false - leaving out alone - for anything else:
 * a short frame, a different lead or function byte, an inconsistent byte
 * count, a bad checksum, more registers than this decoder knows, or fewer
 * than the assigned fields need. The BMS also sends a shorter, still
 * unidentified notification; that is one of the things this rejects. */
bool wf_bms_decode(const uint8_t *data, size_t len, wf_bms_t *out);

#endif /* WFDECODE_H */
