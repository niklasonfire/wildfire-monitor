/*
 * unit - the parts of the decoding no Capture we hold can exercise.
 *
 * tests/host/replay.c is the main test: it replays real recorded rides and
 * asserts what they have to produce. That is the right shape for almost
 * everything, and it is deliberately the only place ride facts live. But a
 * fixture can only assert what the bike actually did, and the power block
 * decoded from a frame built byte by byte is not that: cap0007 proves the
 * eight types agree with each other and with the BMS, while this proves the
 * decoder reads the offsets the Field Table declares, including a negative
 * line current, which that ride never produced.
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
    test_power_block_decodes_at_every_type();
    test_line_current_is_signed();
    test_a_type_outside_the_block_changes_nothing();

    if (failures != 0) {
        printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
        return 1;
    }
    printf("unit: the power block, all assertions hold\n");
    return 0;
}
