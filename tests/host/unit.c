/*
 * unit - the parts of the decoding no Capture we hold can exercise.
 *
 * tests/host/replay.c is the main test: it replays real recorded rides and
 * asserts what they have to produce. That is the right shape for almost
 * everything, and it is deliberately the only place ride facts live. But a
 * fixture can only assert what the bike actually did, and two things here are
 * not in any Capture we have:
 *
 *   - the Odometer wrap. `odometer_raw` reads a constant 14 through the whole
 *     of cap0007, so that fixture cannot distinguish wrap-safe differencing
 *     from the naive kind that puts a 6553 km phantom trip in the archive.
 *     Pretending it does would be worse than not testing it, so the wrap is
 *     driven here with synthesised counts instead.
 *
 *   - the power block decoded from a frame built byte by byte. cap0007 proves
 *     the eight types agree with each other and with the BMS; this proves the
 *     decoder reads the offsets the Field Table declares, including a negative
 *     line current, which that ride never produced.
 *
 * Same rules as the rest of main/wfdecode: pure C99, no board, no fixtures.
 */
#include <stdio.h>
#include <string.h>

#include "wfdecode.h"

static int failures;

#define CHECK(cond, ...)                                                      \
    do {                                                                      \
        if (!(cond)) {                                                        \
            failures++;                                                       \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);              \
            fprintf(stderr, __VA_ARGS__);                                     \
            fputc('\n', stderr);                                              \
        }                                                                     \
    } while (0)

#define CHECK_U(got, want, what)                                              \
    do {                                                                      \
        unsigned long g_ = (unsigned long)(got), w_ = (unsigned long)(want);  \
        CHECK(g_ == w_, "%s: got %lu, expected %lu", (what), g_, w_);         \
    } while (0)

/* ------------------------------------------------------------- Odometer */

static void test_odo_metres(void)
{
    CHECK_U(wf_ctrl_odo_metres(0), 0, "0 counts");
    CHECK_U(wf_ctrl_odo_metres(1), WF_CTRL_ODO_METRES_PER_COUNT, "1 count");
    /* What cap0007 actually reads, converted. */
    CHECK_U(wf_ctrl_odo_metres(14), 14u * WF_CTRL_ODO_METRES_PER_COUNT,
            "cap0007's 14 counts");
    /* The last count before the wrap. At 100 m each this is the ~6553 km the
     * Field Table's note quotes, and it has to fit in the u32 it is returned
     * in - which it does with three orders of magnitude to spare. */
    CHECK_U(wf_ctrl_odo_metres(65535), 65535u * WF_CTRL_ODO_METRES_PER_COUNT,
            "the last count before the wrap");
}

static void test_odo_delta_does_not_wrap(void)
{
    CHECK_U(wf_ctrl_odo_delta_counts(0, 0), 0, "no movement from zero");
    CHECK_U(wf_ctrl_odo_delta_counts(14, 14), 0, "no movement, parked");
    CHECK_U(wf_ctrl_odo_delta_counts(10, 25), 15, "15 counts forward");
    CHECK_U(wf_ctrl_odo_delta_metres(10, 25), 15u * WF_CTRL_ODO_METRES_PER_COUNT,
            "15 counts forward, in metres");
}

/* The whole point of the helper. A trip that straddles the u16 wrap is a short
 * trip, not a 6553 km one - and every one of these is a difference the naive
 * subtraction gets catastrophically wrong. */
static void test_odo_delta_across_the_wrap(void)
{
    CHECK_U(wf_ctrl_odo_delta_counts(65535, 0), 1, "one count over the wrap");
    CHECK_U(wf_ctrl_odo_delta_counts(65530, 5), 11, "11 counts over the wrap");
    CHECK_U(wf_ctrl_odo_delta_metres(65530, 5), 11u * WF_CTRL_ODO_METRES_PER_COUNT,
            "11 counts over the wrap, in metres");
    /* A whole lap of the counter reads as no movement, which is the one thing
     * a u16 Odometer genuinely cannot tell you. 6553 km between two readings
     * is not a case any ride produces; it is recorded here so that the limit
     * is written down rather than discovered. */
    CHECK_U(wf_ctrl_odo_delta_counts(1234, 1234), 0, "a full lap of the counter");
    /* And the flip side, also deliberate: a reading that went backwards is
     * indistinguishable from an almost-complete wrap. Callers difference
     * samples in the order they were taken; nothing here can rescue them if
     * they do not. */
    CHECK_U(wf_ctrl_odo_delta_counts(5, 4), 65535, "a step backwards");
}

/* --------------------------------------------------------- the power block */

/* A Controller frame built from a type and twelve payload bytes, checksummed
 * the way the Controller checksums it, so it goes through exactly the parser
 * a real frame goes through. */
static void make_frame(uint8_t out[WF_CTRL_FRAME_LEN], uint8_t type,
                       const uint8_t payload[WF_CTRL_PAYLOAD_LEN])
{
    out[0] = WF_CTRL_LEAD;
    out[1] = type;
    memcpy(out + 2, payload, WF_CTRL_PAYLOAD_LEN);
    uint16_t crc = wf_crc16(out, WF_CTRL_FRAME_LEN - 2, WF_CTRL_CRC_INIT);
    out[14] = (uint8_t)(crc & 0xff);
    out[15] = (uint8_t)(crc >> 8);
}

static bool apply_power(wf_ctrl_live_t *live, uint8_t type, uint16_t raw_v,
                        int16_t raw_a)
{
    uint8_t payload[WF_CTRL_PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    payload[0] = (uint8_t)(raw_v & 0xff);
    payload[1] = (uint8_t)(raw_v >> 8);
    payload[4] = (uint8_t)((uint16_t)raw_a & 0xff);
    payload[5] = (uint8_t)((uint16_t)raw_a >> 8);

    uint8_t frame[WF_CTRL_FRAME_LEN];
    make_frame(frame, type, payload);

    wf_ctrl_frame_t parsed;
    if (!wf_ctrl_frame_parse(frame, sizeof(frame), &parsed)) {
        return false;
    }
    wf_ctrl_apply(live, &parsed);
    return true;
}

/* Near enough for a float that came out of a division. */
static bool close_to(float got, double want)
{
    double d = (double)got - want;
    return d > -1e-4 && d < 1e-4;
}

/* Every one of the eight types has to decode the same two fields out of the
 * same two offsets. cap0007 shows they agree with each other on the bike; this
 * shows the decoder treats them identically, which is the claim the Field
 * Table's one-entry-eight-types row actually makes. */
static void test_power_block_decodes_at_every_type(void)
{
    for (int i = 0; i < WF_CTRL_TYPE_POWER_COUNT; i++) {
        uint8_t type = wf_ctrl_type_power[i];
        wf_ctrl_live_t live;
        memset(&live, 0, sizeof(live));

        CHECK(!live.power_valid, "type 0x%02x: valid before any frame", type);
        if (!apply_power(&live, type, 1053, 35)) {
            CHECK(false, "type 0x%02x: a frame we built ourselves did not parse",
                  type);
            continue;
        }
        CHECK(live.power_valid, "type 0x%02x: power_valid not set", type);
        CHECK(close_to(live.pack_v, 105.3),
              "type 0x%02x: pack_v %.3f, expected 105.3", type, live.pack_v);
        CHECK(close_to(live.line_current_a, 35.0 / WF_CTRL_CURRENT_LSB_PER_A),
              "type 0x%02x: line_current_a %.3f, expected %.3f", type,
              live.line_current_a, 35.0 / WF_CTRL_CURRENT_LSB_PER_A);
    }
}

/* Line current is signed and cap0007 barely goes below zero, so the sign
 * extension is asserted here rather than left to a ride that might never
 * regenerate. -1 raw is what that ride's idle actually reads. */
static void test_line_current_is_signed(void)
{
    wf_ctrl_live_t live;
    memset(&live, 0, sizeof(live));

    CHECK(apply_power(&live, wf_ctrl_type_power[0], 1050, -1),
          "a frame we built ourselves did not parse");
    CHECK(close_to(live.line_current_a, -1.0 / WF_CTRL_CURRENT_LSB_PER_A),
          "line_current_a %.3f at raw -1, expected %.3f", live.line_current_a,
          -1.0 / WF_CTRL_CURRENT_LSB_PER_A);

    CHECK(apply_power(&live, wf_ctrl_type_power[0], 1050, -400),
          "a frame we built ourselves did not parse");
    CHECK(close_to(live.line_current_a, -400.0 / WF_CTRL_CURRENT_LSB_PER_A),
          "line_current_a %.3f at raw -400, expected %.3f", live.line_current_a,
          -400.0 / WF_CTRL_CURRENT_LSB_PER_A);
}

/* A type outside the block must leave the power fields alone: the eight are a
 * list precisely because they are not an arithmetic run, and 0x82 sits between
 * two of them. */
static void test_a_type_outside_the_block_changes_nothing(void)
{
    wf_ctrl_live_t live;
    memset(&live, 0, sizeof(live));

    CHECK(apply_power(&live, 0x82, 1053, 35),
          "a frame we built ourselves did not parse");
    CHECK(!live.power_valid, "type 0x82 set power_valid");
    CHECK(close_to(live.pack_v, 0.0), "type 0x82 wrote pack_v %.3f", live.pack_v);
}

/* ------------------------------------------------------------------- main */

int main(void)
{
    test_odo_metres();
    test_odo_delta_does_not_wrap();
    test_odo_delta_across_the_wrap();
    test_power_block_decodes_at_every_type();
    test_line_current_is_signed();
    test_a_type_outside_the_block_changes_nothing();

    if (failures != 0) {
        printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
        return 1;
    }
    printf("unit: odometer wrap and the power block, all assertions hold\n");
    return 0;
}
