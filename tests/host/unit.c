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
 *   - almost everything main/wfest does. cap0007's State of Charge is pinned
 *     at 66.7 % for the whole 47 s and its current never leaves -0.25..8.75 A,
 *     so that ride is an excellent determinism and plumbing fixture and a
 *     hopeless physics one. The Anchor pulling a drifting Coulomb Count back,
 *     a gap in the BMS stream, the sign convention and the persisted state all
 *     have to be driven with synthesised streams here instead.
 *
 * Same rules as the rest of main/wfdecode and main/wfest: pure C99, no board,
 * no fixtures.
 */
#include <stdio.h>
#include <string.h>

#include "wfdecode.h"
#include "wfest.h"

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
 * list precisely because they are not an arithmetic run, and 0x82 belongs to
 * the third block, one past 0x81. */
static void test_a_type_outside_the_block_changes_nothing(void)
{
    wf_ctrl_live_t live;
    memset(&live, 0, sizeof(live));

    CHECK(apply_power(&live, 0x82, 1053, 35),
          "a frame we built ourselves did not parse");
    CHECK(!live.power_valid, "type 0x82 set power_valid");
    CHECK(close_to(live.pack_v, 0.0), "type 0x82 wrote pack_v %.3f", live.pack_v);
}

/* -------------------------------------------------------- the motion block */

/* The mistake ADR-0002 exists to prevent, caught for the third time in #13:
 * gear, the flags and rpm live in a block of eight frame types and the Field
 * Table declared one of them, so everything computed from rpm - road speed,
 * and the distance integrated from it - updated at 0.64 Hz instead of 5.17 Hz.
 *
 * cap0007 is what established the block, and tests/fixtures/cap0007.expect
 * asserts the rate it produces. This asserts the decoder's half: all eight
 * types have to write the same fields out of the same offsets, and no other
 * type may.
 */
static bool apply_motion(wf_ctrl_live_t *live, uint8_t type, uint8_t byte0,
                         uint16_t rpm)
{
    uint8_t payload[WF_CTRL_PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    payload[0] = byte0;
    payload[6] = (uint8_t)(rpm & 0xff);
    payload[7] = (uint8_t)(rpm >> 8);

    uint8_t frame[WF_CTRL_FRAME_LEN];
    make_frame(frame, type, payload);

    wf_ctrl_frame_t parsed;
    if (!wf_ctrl_frame_parse(frame, sizeof(frame), &parsed)) {
        return false;
    }
    wf_ctrl_apply(live, &parsed);
    return true;
}

static void test_motion_block_decodes_at_every_type(void)
{
    CHECK_U(WF_CTRL_TYPE_MOTION_COUNT, 8, "types in the motion block");

    for (int i = 0; i < WF_CTRL_TYPE_MOTION_COUNT; i++) {
        uint8_t type = wf_ctrl_type_motion[i];
        wf_ctrl_live_t live;
        memset(&live, 0, sizeof(live));

        CHECK(!live.motion_valid, "type 0x%02x: valid before any frame", type);
        /* 0x39: gear 2 (sport), sliding_backwards and motion both set - the
         * value cap0007 caught at t = 4.7 s, on 0x87 and 0x8e and nowhere
         * else, which is the gear change the single-type decode never saw. */
        if (!apply_motion(&live, type, 0x39, 253)) {
            CHECK(false, "type 0x%02x: a frame we built ourselves did not parse",
                  type);
            continue;
        }
        CHECK(live.motion_valid, "type 0x%02x: motion_valid not set", type);
        CHECK_U(live.cur_rpm, 253, "cur_rpm from the motion block");
        CHECK_U(live.gear, 2, "gear from the motion block");
        CHECK(live.sliding_backwards, "type 0x%02x: sliding_backwards", type);
        CHECK(live.motion, "type 0x%02x: motion", type);
    }
}

/* The third block of eight, 0x82 0x89 0x90 0x97 0x9e 0xa5 0xac 0xb2, is live
 * telemetry in cap0007 and the Field Table assigns it nothing. A frame from it
 * shaped like a motion frame must therefore change nothing at all - the eight
 * are a list because the blocks interleave, and 0x82 sits one past 0x81. */
static void test_the_undecoded_third_block_changes_nothing(void)
{
    wf_ctrl_live_t live;
    memset(&live, 0, sizeof(live));

    CHECK(apply_motion(&live, 0x82, 0x39, 253),
          "a frame we built ourselves did not parse");
    CHECK(!live.motion_valid, "type 0x82 set motion_valid");
    CHECK_U(live.cur_rpm, 0, "type 0x82 wrote cur_rpm");
}

/* ============================================================ the estimator */

#define CHECK_D(got, want, tol, what)                                         \
    do {                                                                      \
        double g_ = (double)(got), w_ = (double)(want), t_ = (double)(tol);   \
        double d_ = g_ - w_;                                                  \
        if (d_ < 0.0) { d_ = -d_; }                                           \
        CHECK(d_ <= t_, "%s: got %.4f, expected %.4f +/- %.4f",               \
              (what), g_, w_, t_);                                            \
    } while (0)

/* The synthesised streams below are the whole point of this section: a
 * Controller power sample every SAMPLE_MS and a BMS answer every POLL_MS, at
 * whatever voltage, current and State of Charge the test wants. Both mimic
 * what the real streams do - the Controller's power block at ~5.2 Hz per
 * ADR-0003, the BMS at ~1 Hz. */
#define SAMPLE_MS  200u
#define POLL_MS    1000u

static void feed_power(wf_est_t *e, uint32_t t_ms, double volts, double amps)
{
    wf_ctrl_live_t live;
    memset(&live, 0, sizeof(live));
    live.power_valid    = true;
    live.pack_v         = (float)volts;
    live.line_current_a = (float)amps;
    /* Any of the eight will do; wf_est_feed_ctrl() treats them alike, which
     * test_power_block_decodes_at_every_type() above pins on the decode side. */
    wf_est_feed_ctrl(e, t_ms, wf_ctrl_type_power[0], &live);
}

static void feed_soc(wf_est_t *e, uint32_t t_ms, double soc_pct)
{
    wf_bms_t b;
    memset(&b, 0, sizeof(b));
    b.soc_pct = (float)soc_pct;
    wf_est_feed_bms(e, t_ms, &b);
}

static double remaining(const wf_est_t *e)
{
    wf_est_out_t o;
    wf_est_get(e, &o);
    return o.remaining_wh;
}

static double soc_of(const wf_est_t *e)
{
    wf_est_out_t o;
    wf_est_get(e, &o);
    return o.soc_pct;
}

/* ---- the provisional Limp Point model ---------------------------------- */

/* The numbers in wfest.h's header comment, asserted rather than asserted-in-
 * prose. If one of the provisional constants is corrected these move, and they
 * are meant to: the failure is the notification. */
static void test_limp_point_model(void)
{
    /* The line passes through the point cap0007 measured, by construction. */
    CHECK_D(wf_est_pack_v_at_soc(WF_EST_PACK_SOC_AT_REF), WF_EST_PACK_V_AT_REF,
            1e-9, "the line through cap0007's measured point");
    CHECK_D(wf_est_pack_v_at_soc(100.0), WF_EST_PACK_V_FULL, 1e-9,
            "the line at a full Pack");

    /* 84.0 V is about 9 % on this line, so roughly a tenth of the Pack sits
     * below the Limp Point and is never offered to the rider. */
    CHECK_D(wf_est_limp_soc_pct(), 9.034, 0.01, "the Limp Point as a percentage");
    CHECK_D(wf_est_pack_v_at_soc(wf_est_limp_soc_pct()), WF_EST_LIMP_POINT_V,
            1e-9, "pack voltage at the Limp Point");

    /* Nothing left at or below the Limp Point, and never a negative. */
    CHECK_D(wf_est_energy_above_limp_wh(wf_est_limp_soc_pct()), 0.0, 1e-9,
            "energy at the Limp Point");
    CHECK_D(wf_est_energy_above_limp_wh(0.0), 0.0, 1e-9,
            "energy at a flat Pack");

    /* A full Pack, and cap0007's 66.7 %. Both quoted in wfest.h. */
    CHECK_D(wf_est_energy_above_limp_wh(100.0), 4584.0, 2.0,
            "energy above the Limp Point, full");
    CHECK_D(wf_est_energy_above_limp_wh(WF_EST_PACK_SOC_AT_REF), 2729.0, 2.0,
            "energy above the Limp Point at cap0007's 66.7 %");

    /* The cross-check wfest.h claims: this line integrated over the whole Pack
     * gives about 5.0 kWh against the ~5.2 kWh the Pack is reckoned to hold.
     * That is 0 % to 100 %, so it includes the charge below the Limp Point. */
    double v_mean_full = (wf_est_pack_v_at_soc(0.0) + WF_EST_PACK_V_FULL) / 2.0;
    CHECK_D(v_mean_full * WF_EST_RATED_CAPACITY_AH, 4956.0, 5.0,
            "the whole Pack by this line, against the ~5.2 kWh nameplate");
}

/* ---- acquisition -------------------------------------------------------- */

/* Nothing to show before the BMS has spoken: no Anchor, no persisted state, no
 * number. A screen full of confident watt-hours computed from a Pack we have
 * not measured would be worse than a dash. */
static void test_nothing_before_the_first_anchor(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(!o.valid, "valid before any input");
    CHECK(!o.anchored, "anchored before any input");

    for (uint32_t t = 0; t < 5000; t += SAMPLE_MS) {
        feed_power(&e, t, 105.0, 10.0);
    }
    wf_est_get(&e, &o);
    CHECK(!o.valid, "valid on Controller frames alone");
    CHECK_U(o.power_samples, 25, "power samples integrated before the Anchor");
}

/* The one assignment there is. */
static void test_the_first_anchor_acquires(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_soc(&e, 0, 50.0);

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.valid, "not valid after the first BMS answer");
    CHECK(o.anchored, "not anchored after the first BMS answer");
    CHECK_D(o.coulomb_ah, 0.5 * WF_EST_RATED_CAPACITY_AH, 1e-3,
            "the Coulomb Count acquired at 50 %");
    CHECK_D(o.remaining_wh, wf_est_energy_above_limp_wh(50.0), 1e-3,
            "Remaining Energy acquired at 50 %");
    CHECK(o.anchor_fresh, "the Anchor is not fresh the moment it arrived");
}

/* ---- the sign convention ------------------------------------------------ */

/* Positive line current is discharge, the Controller's own convention. The BMS
 * reads the other way on discharge and that never comes up, because the only
 * BMS number consumed anywhere is the State of Charge. */
static void test_sign_convention(void)
{
    wf_est_t drain, regen;
    wf_est_init(&drain, NULL);
    wf_est_init(&regen, NULL);
    feed_soc(&drain, 0, 50.0);
    feed_soc(&regen, 0, 50.0);
    double start = remaining(&drain);

    /* One hour at 100 A and 100 V is 100 Ah and 10 kWh, which is more than the
     * Pack holds - deliberately, so the direction is unmistakable. Ten seconds
     * of it is enough to see the sign. The sample at t = 0 only starts the
     * clock, so the ten seconds integrated are 0 to 10000 ms. */
    for (uint32_t t = 0; t <= 10000u; t += SAMPLE_MS) {
        feed_power(&drain, t, 100.0, 100.0);
        feed_power(&regen, t, 100.0, -100.0);
    }

    CHECK(remaining(&drain) < start,
          "drawing 100 A did not take Remaining Energy down");
    CHECK(remaining(&regen) > start,
          "regenerating 100 A did not put Remaining Energy back");

    /* 100 A * 100 V for 10 s is 27.78 Wh, and the Anchor pull over 10 s is
     * 3 % of the error, so the figures are checked loosely on the energy and
     * tightly on nothing but the direction. */
    wf_est_out_t o;
    wf_est_get(&drain, &o);
    CHECK_D(o.used_wh, 100.0 * 100.0 * 10.0 / 3600.0, 0.5,
            "energy drawn over 10 s at 100 A and 100 V");
    CHECK_D(o.used_ah, 100.0 * 10.0 / 3600.0, 0.01,
            "charge drawn over 10 s at 100 A");
    wf_est_get(&regen, &o);
    CHECK(o.used_wh < 0.0, "regeneration counted as energy drawn");
}

/* ---- the Coulomb Count against its Anchor ------------------------------- */

/* The acceptance criterion, driven for an hour of simulated riding.
 *
 * The current is fed in 19 % high, which is exactly the uncertainty
 * WF_CTRL_CURRENT_LSB_PER_A actually carries - upstream says 4 LSB per amp,
 * regression against the BMS says 4.77 - so this is the drift the Anchor
 * exists to absorb and not an invented one. Unanchored, an hour of it puts the
 * Coulomb Count nearly an amp-hour below the truth. Anchored, the steady-state
 * error of a first-order tracker is the drift rate times the time constant,
 * which is under a tenth of an amp-hour.
 */
static void test_the_coulomb_count_tracks_the_anchor(void)
{
    const double v = 100.0;
    const double true_a = 5.0;          /* what the Pack is really giving */
    const double seen_a = 5.0 * 1.19;   /* what the Controller's scale claims */
    const double hours = 1.0;
    const uint32_t end_ms = (uint32_t)(hours * 3600.0 * 1000.0);

    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_soc(&e, 0, 80.0);

    double worst_gap_pct = 0.0;
    for (uint32_t t = SAMPLE_MS; t <= end_ms; t += SAMPLE_MS) {
        feed_power(&e, t, v, seen_a);
        if (t % POLL_MS == 0) {
            /* The BMS, telling the truth at 0.1 % resolution. */
            double true_soc = 80.0 - (true_a * (t / 3600000.0) /
                                      WF_EST_RATED_CAPACITY_AH) * 100.0;
            feed_soc(&e, t, true_soc);
            double gap = soc_of(&e) - true_soc;
            if (gap < 0.0) {
                gap = -gap;
            }
            if (gap > worst_gap_pct) {
                worst_gap_pct = gap;
            }
        }
    }

    /* The Pack really lost 5 Ah, which is 10 % of 50 Ah. */
    double true_end_soc = 70.0;
    CHECK_D(soc_of(&e), true_end_soc, 0.5,
            "the Coulomb Count after an hour on a 19 % high current scale");
    CHECK(worst_gap_pct < 0.5,
          "the Coulomb Count wandered %.3f %% from its Anchor, which is a "
          "divergence and not a wobble", worst_gap_pct);

    /* And the same hour with the Anchor withheld after acquisition, to show
     * the assertion above is about the Anchor and not about the numbers being
     * small. The 19 % scale error alone is worth nearly 2 % of the Pack. */
    wf_est_t loose;
    wf_est_init(&loose, NULL);
    feed_soc(&loose, 0, 80.0);
    for (uint32_t t = SAMPLE_MS; t <= end_ms; t += SAMPLE_MS) {
        feed_power(&loose, t, v, seen_a);
    }
    CHECK(soc_of(&loose) < true_end_soc - 1.5,
          "the unanchored count only drifted to %.3f %%, so this test is not "
          "measuring what it claims", soc_of(&loose));
}

/* ---- a gap in the BMS stream -------------------------------------------- */

/* A hard re-anchor on the first answer after a gap is the step the acceptance
 * criteria forbid, so the gap is made as unkind as it can be: two minutes of
 * silence during which the Controller reports a current that is wildly wrong,
 * then the BMS returns and says nothing has changed. The accumulated error is
 * over 300 Wh. The step across the return has to be a rounding error next to
 * it, and the error still has to be gone by the end of the ride.
 */
static void test_a_bms_gap_produces_no_step(void)
{
    const double v = 100.0;
    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_soc(&e, 0, 80.0);

    uint32_t t = SAMPLE_MS;
    /* A minute of both streams, resting. */
    for (; t <= 60000u; t += SAMPLE_MS) {
        feed_power(&e, t, v, 0.0);
        if (t % POLL_MS == 0) {
            feed_soc(&e, t, 80.0);
        }
    }

    /* Two minutes with the BMS gone and 100 A of nonsense on the Controller. */
    uint32_t gap_end = t + 120000u;
    for (; t <= gap_end; t += SAMPLE_MS) {
        feed_power(&e, t, v, 100.0);
    }

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(!o.anchor_fresh, "the Anchor still counts as fresh after 2 minutes");
    CHECK(o.anchor_age_ms > WF_EST_ANCHOR_STALE_MS,
          "the Anchor's age was not carried through the gap");

    double drift_wh = wf_est_energy_above_limp_wh(80.0) - remaining(&e);
    CHECK(drift_wh > 200.0,
          "the gap only moved Remaining Energy by %.1f Wh, so this test is not "
          "measuring what it claims", drift_wh);

    /* The BMS comes back. */
    double before = remaining(&e);
    feed_soc(&e, t, 80.0);
    CHECK_D(remaining(&e), before, 1e-9,
            "a BMS answer moved Remaining Energy by itself");
    feed_power(&e, t + SAMPLE_MS, v, 0.0);
    double step = remaining(&e) - before;
    if (step < 0.0) {
        step = -step;
    }
    CHECK(step < 1.0,
          "Remaining Energy stepped %.2f Wh when the BMS came back, against "
          "%.1f Wh of accumulated error - that is a re-anchor, not a pull",
          step, drift_wh);

    /* Graceful is not the same as inert: half an hour later the error is gone.
     * At rest, so nothing but the Anchor is moving the figure. */
    uint32_t settle_end = t + 1800000u;
    for (t += SAMPLE_MS; t <= settle_end; t += SAMPLE_MS) {
        feed_power(&e, t, v, 0.0);
        if (t % POLL_MS == 0) {
            feed_soc(&e, t, 80.0);
        }
    }
    CHECK_D(remaining(&e), wf_est_energy_above_limp_wh(80.0), 2.0,
            "Remaining Energy half an hour after the gap closed");
}

/* ---- what drives an integration step ------------------------------------ */

/* Only the power block's eight frame types carry a fresh pack voltage and line
 * current, so only they may take an energy step. Otherwise the integration
 * interval would be set by whichever of the other 47 types happened to arrive,
 * and the curve would depend on arrival timing rather than on the recorded
 * stream.
 *
 * 0x83 is the interleaved noise because the Field Table assigns it nothing at
 * all, which makes the assertion the strongest one available: the whole state,
 * memcmp'd, not just the energy. The motion block would not do - it carries
 * rpm, so it moves distance, and moving distance is its job. */
static void test_only_the_power_block_integrates(void)
{
    wf_est_t plain, noisy;
    wf_est_init(&plain, NULL);
    wf_est_init(&noisy, NULL);
    feed_soc(&plain, 0, 60.0);
    feed_soc(&noisy, 0, 60.0);

    wf_ctrl_live_t live;
    memset(&live, 0, sizeof(live));
    live.power_valid    = true;
    live.pack_v         = 100.0f;
    live.line_current_a = 20.0f;

    for (uint32_t t = SAMPLE_MS; t <= 60000u; t += SAMPLE_MS) {
        wf_est_feed_ctrl(&plain, t, wf_ctrl_type_power[0], &live);
        /* The same ride, with the 35 Hz of everything else interleaved. */
        for (uint32_t k = 1; k < 7; k++) {
            wf_est_feed_ctrl(&noisy, t - SAMPLE_MS + k * 28u, 0x83, &live);
        }
        wf_est_feed_ctrl(&noisy, t, wf_ctrl_type_power[0], &live);
    }

    CHECK(memcmp(&plain, &noisy, sizeof(plain)) == 0,
          "interleaving non-power frames changed the estimate: %.6f vs %.6f Wh",
          remaining(&plain), remaining(&noisy));
}

/* A stalled link must not book the whole stall at the last current it saw. */
static void test_a_long_gap_is_clamped(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_soc(&e, 0, 60.0);
    feed_power(&e, SAMPLE_MS, 100.0, 50.0);

    wf_est_out_t o;
    /* Ten minutes later, one frame. At 50 A that would be 8.3 Ah if it were
     * believed; WF_EST_DT_MAX_MS caps it at 2 s, which is 0.028 Ah. */
    feed_power(&e, SAMPLE_MS + 600000u, 100.0, 50.0);
    wf_est_get(&e, &o);
    CHECK_D(o.used_ah, 50.0 * (WF_EST_DT_MAX_MS / 1000.0) / 3600.0, 1e-6,
            "charge booked for a ten-minute stall");
}

/* ---- distance ----------------------------------------------------------- */

/* cap0007 reads a constant Odometer of 14 and never exceeds 5.64 km/h, so it
 * cannot exercise a wrap, a link drop or any real distance at all. All of that
 * is driven here, the same way the Odometer's own wrap is above.
 *
 * The synthesised streams below stand in for the two halves of the fusion: a
 * motion frame every SAMPLE_MS carrying a road speed, and an Odometer frame at
 * ODO_MS carrying a raw u16 count. Both mimic the real rates - the motion
 * block at ~5.2 Hz and the Odometer's single type at ~0.65 Hz. */
#define ODO_MS 1550u        /* one 55-type cycle, which is what 0x94 gets */

static void feed_speed(wf_est_t *e, uint32_t t_ms, double kmh)
{
    wf_ctrl_live_t live;
    memset(&live, 0, sizeof(live));
    live.motion_valid  = true;
    live.speed_valid   = true;
    live.cur_speed_kmh = (float)kmh;
    wf_est_feed_ctrl(e, t_ms, wf_ctrl_type_motion[0], &live);
}

/* A motion frame with no road speed in it: the wheel geometry from type 0xaf
 * has not arrived, so wf_ctrl_apply() has left speed_valid false. */
static void feed_no_speed(wf_est_t *e, uint32_t t_ms)
{
    wf_ctrl_live_t live;
    memset(&live, 0, sizeof(live));
    live.motion_valid = true;
    wf_est_feed_ctrl(e, t_ms, wf_ctrl_type_motion[0], &live);
}

static void feed_odo(wf_est_t *e, uint32_t t_ms, uint16_t counts)
{
    wf_ctrl_live_t live;
    memset(&live, 0, sizeof(live));
    live.odo_valid    = true;
    live.odometer_raw = counts;
    wf_est_feed_ctrl(e, t_ms, WF_CTRL_TYPE_ODO, &live);
}

static double distance(const wf_est_t *e)
{
    wf_est_out_t o;
    wf_est_get(e, &o);
    return o.distance_m;
}

/* Nothing to show before either source has spoken. A confident 0 m from a link
 * that has never come up is the same mistake as a confident watt-hour figure
 * from a Pack nobody has measured. */
static void test_no_distance_before_either_source(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(!o.distance_valid, "distance valid before any input");
    CHECK(!o.odo_anchored, "odo anchored before any input");

    /* A BMS answer is not a distance source. */
    feed_soc(&e, 0, 50.0);
    wf_est_get(&e, &o);
    CHECK(!o.distance_valid, "a BMS answer made distance valid");
}

/* The resolution half of ADR-0003, on its own: with no Odometer at all,
 * distance is the integral of road speed and nothing else, and it steps once
 * per motion frame rather than once per Odometer tick. */
static void test_distance_integrates_speed(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);

    /* 36 km/h is 10 m/s. Sixty seconds of it is 600 m. The first frame only
     * starts the clock, so the integrated span is 0 to 60000 ms. */
    for (uint32_t t = 0; t <= 60000u; t += SAMPLE_MS) {
        feed_speed(&e, t, 36.0);
    }

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.distance_valid, "distance not valid after a minute of riding");
    CHECK(!o.odo_anchored, "distance anchored with no Odometer in the stream");
    CHECK_D(o.distance_m, 600.0, 1e-6, "600 m at 36 km/h for a minute");
    /* Once per motion frame: 300 steps in the minute, the first frame having
     * only started the clock. That is the criterion "updated at the rate of
     * the Controller's stream rather than at Odometer ticks", as a number. */
    CHECK_U(o.distance_samples, 300, "distance steps over the minute");
}

/* The Anchor half. Road speed is fed 20 % high - it is rpm times a wheel
 * circumference and a gearing constant, none of them measured - and the
 * Odometer tells the truth. Unanchored, ten minutes of that is 400 m of
 * invented distance. Anchored, the answer has to end up inside the Odometer's
 * own hundred-metre quantisation, and it has to get there without ever
 * stepping backwards or visibly at a tick. */
static void test_the_odometer_corrects_drift_without_a_step(void)
{
    const double true_mps  = 10.0;              /* 36 km/h */
    const double seen_kmh  = 36.0 * 1.2;        /* what the Controller claims */
    const uint32_t end_ms  = 600000u;           /* ten minutes */

    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_odo(&e, 0, 1000);                      /* acquisition, at zero */

    double prev = 0.0;
    double worst_back = 0.0;                    /* largest backwards move */
    double worst_step = 0.0;                    /* largest single step */
    double worst_error = 0.0;                   /* furthest from the truth */
    for (uint32_t t = SAMPLE_MS; t <= end_ms; t += SAMPLE_MS) {
        feed_speed(&e, t, seen_kmh);
        double now = distance(&e);
        double step = now - prev;
        if (step < worst_back) {
            worst_back = step;
        }
        if (step > worst_step) {
            worst_step = step;
        }
        prev = now;

        double true_m = true_mps * (t / 1000.0);
        double err = now - true_m;
        if (err < 0.0) {
            err = -err;
        }
        if (err > worst_error) {
            worst_error = err;
        }
        if (t % ODO_MS < SAMPLE_MS) {
            /* The Odometer, counting real metres at a hundred to the count. */
            feed_odo(&e, t, (uint16_t)(1000u + (unsigned)(true_m /
                        WF_CTRL_ODO_METRES_PER_COUNT)));
        }
    }

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.odo_anchored, "the Odometer never acquired");

    /* The criterion, against the ground rather than against the Odometer.
     * The Odometer's own account is the truth floored to a whole count, so it
     * is itself up to a count low and is the wrong thing to be tight against;
     * what has to hold is that the fused figure is inside one Odometer count
     * of the metres the bike really covered, throughout, and not only at the
     * end. */
    CHECK(worst_error < WF_CTRL_ODO_METRES_PER_COUNT,
          "distance was %.0f m from the truth at its worst, more than the "
          "Odometer's own %d m quantisation", worst_error,
          WF_CTRL_ODO_METRES_PER_COUNT);
    CHECK_D(o.odo_distance_m, 6000.0, 1e-9, "the Odometer's own account");

    /* Never backwards, at all, ever. */
    CHECK_D(worst_back, 0.0, 0.0, "the largest backwards move in distance");

    /* And no step visible at a tick. A free-running step at 43.2 km/h is
     * 2.4 m; if the Anchor were assigned rather than pulled, the step at the
     * tick that closed a 400 m error would be hundreds of times that. */
    CHECK(worst_step < 3.0,
          "distance stepped %.3f m in one frame, against the 2.4 m free "
          "running at this speed - that is a re-anchor, not a pull",
          worst_step);

    /* And the assertion that the test is measuring something: the same ten
     * minutes with the Odometer withheld invents 20 % of the ride. */
    wf_est_t loose;
    wf_est_init(&loose, NULL);
    for (uint32_t t = 0; t <= end_ms; t += SAMPLE_MS) {
        feed_speed(&loose, t, seen_kmh);
    }
    CHECK(distance(&loose) > 7000.0,
          "the unanchored distance only reached %.0f m, so this test is not "
          "measuring what it claims", distance(&loose));
}

/* The acceptance criterion, and the one cap0007 cannot touch: an Odometer that
 * wraps must change nothing at all.
 *
 * Two identical rides, differing only in where the counter started - one well
 * clear of the wrap, one six counts short of it, so the second crosses 65535
 * to 0 partway through. The Anchor is built from differences and never from an
 * absolute reading, so the two must not merely be close: they have to be the
 * same double, bit for bit. */
static void test_an_odometer_wrap_changes_nothing(void)
{
    wf_est_t plain, wrapped;
    wf_est_init(&plain, NULL);
    wf_est_init(&wrapped, NULL);

    unsigned tick = 0;
    for (uint32_t t = 0; t <= 300000u; t += SAMPLE_MS) {
        feed_speed(&plain, t, 36.0);
        feed_speed(&wrapped, t, 36.0);
        if (t % ODO_MS < SAMPLE_MS) {
            feed_odo(&plain,   t, (uint16_t)(1000u + tick));
            feed_odo(&wrapped, t, (uint16_t)(65530u + tick));
            tick++;
        }
    }
    CHECK(tick > 10, "the ride was too short to reach the wrap");

    CHECK(memcmp(&plain.distance_m, &wrapped.distance_m,
                 sizeof(plain.distance_m)) == 0,
          "the wrapping ride produced %.9f m against %.9f m",
          wrapped.distance_m, plain.distance_m);
    CHECK(memcmp(&plain.odo_distance_m, &wrapped.odo_distance_m,
                 sizeof(plain.odo_distance_m)) == 0,
          "the wrapping ride's Odometer account is %.9f m against %.9f m",
          wrapped.odo_distance_m, plain.odo_distance_m);
}

/* A reading that goes backwards is what the wrap-safe subtraction cannot tell
 * from a nearly-complete wrap, so it is credited as 65535 counts - 6553 km.
 * The estimator refuses anything past half the counter's span and re-bases
 * instead, because nothing on this bike covers 3276 km between two frames. */
static void test_an_odometer_that_jumps_is_not_believed(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_odo(&e, 0, 1000);
    feed_odo(&e, ODO_MS, 1005);             /* 500 m, believed */
    feed_odo(&e, 2 * ODO_MS, 5);            /* a leap backwards */
    feed_odo(&e, 3 * ODO_MS, 10);           /* 500 m from the new base */

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK_D(o.odo_distance_m, 1000.0, 1e-9,
            "the Odometer account after a reading that went backwards");
}

/* The Controller link drops for two minutes while the bike keeps riding, then
 * comes back. Nothing may be lost - the metres covered during the drop are in
 * the Odometer, which is exactly what a non-drifting Anchor is for - and
 * nothing may be duplicated, which is what the dt clamp is for: two minutes at
 * the last speed seen would be 1.2 km of invented distance. */
static void test_a_controller_link_drop_loses_no_distance(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_odo(&e, 0, 1000);

    uint32_t t = SAMPLE_MS;
    /* A minute at 36 km/h: 600 m, six counts. */
    for (; t <= 60000u; t += SAMPLE_MS) {
        feed_speed(&e, t, 36.0);
        if (t % ODO_MS < SAMPLE_MS) {
            feed_odo(&e, t, (uint16_t)(1000u + (t / 10000u)));
        }
    }

    /* The link goes. The bike rides on for two minutes at the same speed -
     * another 1200 m, twelve counts - and the Monitor hears none of it. */
    uint32_t back_ms = t + 120000u;

    /* The link returns, and the first Odometer reading carries the whole gap:
     * 60 s + 120 s at 10 m/s is 1800 m, count 1000 + 18. */
    feed_speed(&e, back_ms, 36.0);
    feed_odo(&e, back_ms, 1018);

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK_D(o.odo_distance_m, 1800.0, 1e-9,
            "the Odometer account across the drop");
    /* Nothing duplicated: the one frame that spanned the gap booked at most
     * WF_EST_DT_MAX_MS of speed, not two minutes of it. */
    CHECK(o.distance_m < 700.0,
          "distance jumped to %.0f m on the first frame after the drop; two "
          "minutes at 10 m/s would be 1200 m of it invented", o.distance_m);

    /* Nothing lost: keep riding and the pull brings the whole gap back, to
     * within the Odometer's quantisation. Five minutes at 36 km/h is 3000 m
     * more, so 4800 m in total. */
    uint32_t end_ms = back_ms + 300000u;
    for (t = back_ms + SAMPLE_MS; t <= end_ms; t += SAMPLE_MS) {
        feed_speed(&e, t, 36.0);
        if (t % ODO_MS < SAMPLE_MS) {
            feed_odo(&e, t, (uint16_t)(1000u + 18u +
                        ((t - back_ms) / 10000u)));
        }
    }
    wf_est_get(&e, &o);
    CHECK_D(o.distance_m, o.odo_distance_m, WF_CTRL_ODO_METRES_PER_COUNT,
            "distance against the Odometer five minutes after the link came "
            "back");
    CHECK_D(o.distance_m, 4800.0, 2.0 * WF_CTRL_ODO_METRES_PER_COUNT,
            "distance against the 4800 m actually covered, drop included");
}

/* Speed needs the wheel geometry from type 0xaf, and until that has arrived
 * `speed_valid` is false. An unknown speed is not integrated as zero; the
 * Odometer carries distance on its own in that window, smoothed from a
 * hundred-metre staircase into something continuous.
 *
 * What that costs is a first-order lag: with nothing but the Anchor pulling,
 * distance trails the Odometer by roughly the time constant times the speed
 * while the bike is moving, and catches up completely when it stops. Both
 * halves are asserted, because the lag is the honest consequence of having no
 * speed to integrate and not a defect to hide. */
static void test_distance_before_the_wheel_geometry(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_odo(&e, 0, 1000);

    uint32_t t = SAMPLE_MS;
    double overshoot = 0.0;
    for (; t <= 300000u; t += SAMPLE_MS) {
        feed_no_speed(&e, t);
        if (t % ODO_MS < SAMPLE_MS) {
            feed_odo(&e, t, (uint16_t)(1000u + (t / 10000u)));
        }
        wf_est_out_t s;
        wf_est_get(&e, &s);
        if (s.distance_m - s.odo_distance_m > overshoot) {
            overshoot = s.distance_m - s.odo_distance_m;
        }
    }

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.distance_valid, "no distance at all without the wheel geometry");
    CHECK(o.odo_distance_m > 2500.0, "the Odometer account only reached %.0f m",
          o.odo_distance_m);
    /* Behind the Anchor, never in front of it: with no speed there is nothing
     * that could put distance ahead of the Odometer. */
    CHECK_D(overshoot, 0.0, 1e-9,
            "distance ran ahead of the Odometer with no speed to integrate");
    CHECK(o.distance_m < o.odo_distance_m,
          "distance %.0f m did not lag the Odometer's %.0f m while moving",
          o.distance_m, o.odo_distance_m);

    /* The bike stops. The Odometer stops with it, the Controller keeps
     * talking, and the lag closes completely. */
    double parked_odo = o.odo_distance_m;
    uint32_t end_ms = t + 600000u;
    for (; t <= end_ms; t += SAMPLE_MS) {
        feed_no_speed(&e, t);
    }
    wf_est_get(&e, &o);
    CHECK_D(o.distance_m, parked_odo, 1.0,
            "distance ten minutes after the bike stopped");
}

/* ---- determinism -------------------------------------------------------- */

/* The synthesised half of the acceptance criterion; tests/host/replay.c does
 * the same over a real recorded Capture. Same input, same state, bit for bit -
 * memcmp and not a tolerance, because a tolerance is what this criterion is
 * about not having. */
static void test_the_same_input_gives_the_same_state(void)
{
    wf_est_t a, b;
    wf_est_init(&a, NULL);
    wf_est_init(&b, NULL);

    for (int pass = 0; pass < 2; pass++) {
        wf_est_t *e = pass == 0 ? &a : &b;
        feed_soc(e, 0, 73.5);
        for (uint32_t t = SAMPLE_MS; t <= 120000u; t += SAMPLE_MS) {
            double amps = 12.0 + 30.0 * (double)((t / SAMPLE_MS) % 17u) / 17.0;
            feed_power(e, t, 104.0 - amps * 0.02, amps);
            if (t % POLL_MS == 0) {
                feed_soc(e, t, 73.5 - (double)(t / 20000u) * 0.1);
            }
        }
    }
    CHECK(memcmp(&a, &b, sizeof(a)) == 0,
          "two identical synthesised rides produced different state");
}

/* ---- persisted state ---------------------------------------------------- */

static void test_persisted_state_round_trips(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_soc(&e, 0, 42.0);
    for (uint32_t t = SAMPLE_MS; t <= 30000u; t += SAMPLE_MS) {
        feed_power(&e, t, 98.0, 25.0);
    }

    wf_est_persist_t saved;
    wf_est_save(&e, &saved);
    CHECK(saved.valid, "a ridden estimator saved as invalid");

    uint8_t blob[WF_EST_PERSIST_BYTES];
    CHECK(wf_est_persist_encode(&saved, blob, sizeof(blob)),
          "encoding the persisted state failed");
    CHECK(!wf_est_persist_encode(&saved, blob, sizeof(blob) - 1),
          "encoding into a short buffer succeeded");

    wf_est_persist_t back;
    CHECK(wf_est_persist_decode(blob, sizeof(blob), &back),
          "decoding the persisted state failed");
    CHECK_D(back.coulomb_ah, saved.coulomb_ah, 1e-6, "coulomb_ah round trip");
    CHECK_D(back.remaining_wh, saved.remaining_wh, 1e-6,
            "remaining_wh round trip");
    CHECK_D(back.rated_capacity_ah, WF_EST_RATED_CAPACITY_AH, 1e-6,
            "rated_capacity_ah round trip");

    /* Anything else is not our blob. NVS hands back whatever is in the
     * partition, including what an older build left there. */
    CHECK(!wf_est_persist_decode(blob, sizeof(blob) - 1, &back),
          "a short blob decoded");
    blob[9] ^= 0x01;
    CHECK(!wf_est_persist_decode(blob, sizeof(blob), &back),
          "a blob with a flipped bit decoded");
    blob[9] ^= 0x01;
    blob[0] = 'X';
    CHECK(!wf_est_persist_decode(blob, sizeof(blob), &back),
          "a blob with the wrong magic decoded");
}

/* A restored count bridges the gap to the first BMS answer and is not itself
 * an Anchor: the Pack may have been charged while the Monitor was off, and
 * pulling gently toward the truth from a count that is 40 % wrong would take a
 * quarter of an hour. So the first answer after a restore still acquires. */
static void test_a_restored_count_is_not_an_anchor(void)
{
    wf_est_persist_t saved = {
        .version           = WF_EST_PERSIST_VERSION,
        .valid             = true,
        .coulomb_ah        = 15.0f,
        .remaining_wh      = 750.0f,
        .rated_capacity_ah = (float)WF_EST_RATED_CAPACITY_AH,
    };

    wf_est_t e;
    wf_est_init(&e, &saved);
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.valid, "a restored estimator has nothing to show");
    CHECK(!o.anchored, "a restored count was treated as an Anchor");
    CHECK_D(o.coulomb_ah, 15.0, 1e-6, "the restored Coulomb Count");
    CHECK_D(o.remaining_wh, 750.0, 1e-6, "the restored Remaining Energy");

    /* The Pack was charged overnight. The first answer says so, and is
     * believed at once. */
    feed_soc(&e, 1000, 95.0);
    wf_est_get(&e, &o);
    CHECK(o.anchored, "the first answer after a restore did not acquire");
    CHECK_D(o.coulomb_ah, 0.95 * WF_EST_RATED_CAPACITY_AH, 1e-3,
            "the Coulomb Count after acquiring over a stale restore");

    /* A count saved against a different Rated Capacity is a count of something
     * else, so it is dropped rather than rescaled. */
    saved.rated_capacity_ah = 60.0f;
    wf_est_init(&e, &saved);
    wf_est_get(&e, &o);
    CHECK(!o.valid, "a count saved against 60 Ah was restored onto a 50 Ah Pack");

    saved.rated_capacity_ah = (float)WF_EST_RATED_CAPACITY_AH;
    saved.version = WF_EST_PERSIST_VERSION + 1;
    wf_est_init(&e, &saved);
    wf_est_get(&e, &o);
    CHECK(!o.valid, "a count from a future version was restored");
}

/* ---- distance across a power cycle -------------------------------------- */

/* The acceptance criterion. Half a ride, the Monitor loses power, the state
 * goes through NVS's bytes and comes back, and the second half carries on from
 * where the first stopped rather than from zero.
 *
 * The Odometer's count is deliberately NOT persisted. The first reading after
 * the restore acquires against the restored distance, so the bike having been
 * ridden while the Monitor was off - it runs on its own battery and can be
 * switched off under a rider who keeps going - is not credited to a Capture
 * that did not see it. */
static void test_distance_survives_a_power_cycle(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_odo(&e, 0, 1000);
    for (uint32_t t = SAMPLE_MS; t <= 120000u; t += SAMPLE_MS) {
        feed_speed(&e, t, 36.0);
        if (t % ODO_MS < SAMPLE_MS) {
            feed_odo(&e, t, (uint16_t)(1000u + (t / 10000u)));
        }
    }
    double before = distance(&e);
    CHECK(before > 1000.0, "two minutes at 36 km/h only reached %.0f m",
          before);

    /* Out through the bytes and back, exactly as est_store.c does it. */
    wf_est_persist_t saved;
    wf_est_save(&e, &saved);
    CHECK(saved.distance_valid, "a ridden distance saved as invalid");

    uint8_t blob[WF_EST_PERSIST_BYTES];
    CHECK(wf_est_persist_encode(&saved, blob, sizeof(blob)), "encode failed");
    wf_est_persist_t back;
    CHECK(wf_est_persist_decode(blob, sizeof(blob), &back), "decode failed");

    wf_est_t after;
    wf_est_init(&after, &back);
    wf_est_out_t o;
    wf_est_get(&after, &o);
    CHECK(o.distance_valid, "distance did not survive the power cycle");
    CHECK(!o.odo_anchored, "the Odometer count was persisted; it must not be");
    /* A float, so the restored value is the double rounded to single. */
    CHECK_D(o.distance_m, before, 0.05, "distance across the power cycle");

    /* The bike was ridden 5 km while the Monitor was off. The Odometer
     * acquires against the restored distance and credits none of it. */
    feed_odo(&after, 0, 1050);
    wf_est_get(&after, &o);
    CHECK_D(o.odo_distance_m, o.distance_m, 1e-9,
            "the Odometer acquired somewhere other than at the restored "
            "distance");

    /* And carries on. Another minute is 600 m more. */
    for (uint32_t t = SAMPLE_MS; t <= 60000u; t += SAMPLE_MS) {
        feed_speed(&after, t, 36.0);
        if (t % ODO_MS < SAMPLE_MS) {
            feed_odo(&after, t, (uint16_t)(1050u + (t / 10000u)));
        }
    }
    wf_est_get(&after, &o);
    CHECK_D(o.distance_m, before + 600.0, WF_CTRL_ODO_METRES_PER_COUNT,
            "distance after another minute on the far side of the restore");
}

/* ---- Consumption -------------------------------------------------------- */

/* cap0007 covers 16.7 m in 47 s at under 5.7 km/h, which is less than a fifth
 * of one Odometer count and less than a sixtieth of the window. It cannot fill
 * a window, it cannot show Consumption rising on a motorway, and it cannot
 * even clear the stationary guard - which makes it a fine test of the guard
 * and a hopeless test of everything else here. So the riding below is
 * synthesised, the same way the Odometer's wrap and the Anchor's pull are.
 *
 * The arithmetic that makes these tests readable: at a steady speed and a
 * steady draw, watt-hours per kilometre is watts divided by km/h. 105 V at
 * 10.2857 A is 1080 W, and at 36 km/h that is exactly 30 Wh/km. */
#define CONS_VOLTS 105.0

/* Amps that buy a wanted Wh/km at a wanted speed. */
static double amps_for(double wh_per_km, double kmh)
{
    return wh_per_km * kmh / CONS_VOLTS;
}

/* One tick of a steady ride: the motion frame carrying the road speed and the
 * power-block frame carrying what it is bought with. Two blocks, two clocks
 * inside the estimator, both stepped by the same stream - which is what a real
 * Controller link does. */
static void feed_ride(wf_est_t *e, uint32_t t_ms, double kmh, double amps)
{
    feed_speed(e, t_ms, kmh);
    feed_power(e, t_ms, CONS_VOLTS, amps);
}

/* Rides a given number of metres at a steady speed and a steady draw, starting
 * from t0_ms, and returns the time it ended at. The frame at t0_ms itself only
 * starts the two clocks; on a continuing ride it is a zero-length step and
 * changes nothing, so this composes. */
static uint32_t ride(wf_est_t *e, uint32_t t0_ms, double metres, double kmh,
                     double amps)
{
    double per_tick = kmh * (1000.0 / 3600.0) * ((double)SAMPLE_MS / 1000.0);
    long ticks = (long)(metres / per_tick + 0.5);
    uint32_t t = t0_ms;

    feed_ride(e, t, kmh, amps);
    for (long i = 0; i < ticks; i++) {
        t += SAMPLE_MS;
        feed_ride(e, t, kmh, amps);
    }
    return t;
}

/* The all-time average as the two totals it is the ratio of, which is what the
 * screen would show once the window has run out and is what the tests below
 * compare the windowed figure against. */
static double alltime_wh_per_km(const wf_est_out_t *o)
{
    return o->alltime_m > 0.0 ? o->alltime_wh / o->alltime_m * 1000.0 : 0.0;
}

/* The stationary-bike criterion, and it is a division and not a hypothetical:
 * cap0007's first eight seconds are exactly this. The ignition is on, the
 * Controller is streaming, current is leaving the Pack and the wheel is not
 * turning, so the denominator is zero and the quotient is not a large
 * Consumption figure but a meaningless one.
 *
 * The assertion is that no figure is produced at all - not that a large one is
 * clamped - and that the field is left at zero rather than at an infinity or a
 * NaN, which is the proof that the division never happened. */
static void test_a_standing_bike_produces_no_consumption(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);

    /* A minute of ignition-on idling: motion frames arriving at 5 Hz with a
     * road speed of zero, and 2 A of hotel load leaving the Pack. */
    for (uint32_t t = 0; t <= 60000u; t += SAMPLE_MS) {
        feed_ride(&e, t, 0.0, 2.0);
    }

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.distance_valid, "a minute of motion frames left distance invalid");
    CHECK_D(o.distance_m, 0.0, 1e-9, "a standing bike moved");
    CHECK(o.alltime_wh > 0.0,
          "a minute at 2 A drew no energy, so the guard is measuring nothing");
    CHECK_D(o.alltime_m, 0.0, 1e-9, "a standing bike accumulated distance");
    CHECK(!o.consumption_valid, "a standing bike produced a Consumption figure");
    CHECK_D(o.consumption_wh_per_km, 0.0, 0.0,
            "something divided by a standing bike's zero distance");

    /* Creeping does not clear it either: the floor is one Odometer count, and
     * below that the Monitor cannot claim to know how far it has gone. Fifty
     * metres is half of one. */
    uint32_t t = ride(&e, 60000u, 50.0, 5.0, 2.0);
    wf_est_get(&e, &o);
    CHECK(o.alltime_m > 40.0 && o.alltime_m < 60.0,
          "the creep covered %.1f m, not the 50 m it was asked for",
          o.alltime_m);
    CHECK(!o.consumption_valid,
          "50 m - half an Odometer count - produced a Consumption figure");
    CHECK_D(o.consumption_wh_per_km, 0.0, 0.0, "a figure from half a count");

    /* Past the floor there is a denominator worth dividing by, and the figure
     * appears - as the all-time average, because a hundred metres is a
     * twentieth of the window and the ring is nowhere near full. */
    ride(&e, t, 100.0, 5.0, 2.0);
    wf_est_get(&e, &o);
    CHECK(o.consumption_valid, "150 m produced no Consumption figure at all");
    CHECK(!o.consumption_windowed,
          "150 m of riding claimed to be a full %.0f m window",
          WF_EST_CONS_WINDOW_M);
    CHECK_D(o.consumption_wh_per_km, alltime_wh_per_km(&o), 1e-9,
            "the fallback figure is not the all-time average");
    CHECK(o.consumption_wh_per_km > 0.0 && o.consumption_wh_per_km < 1e5,
          "the first figure past the floor is %.1f Wh/km",
          o.consumption_wh_per_km);
}

/* The window doing its job: it has to rise on the motorway and fall in town.
 * Three kilometres, each ridden at a different steady cost, and after each one
 * the figure has to be that kilometre's cost and not the average of everything
 * so far - which is the difference between a rolling window and a trip meter,
 * and is asserted by comparing against the all-time average at every step. */
static void test_consumption_follows_the_recent_kilometre(void)
{
    const double town = 30.0, motorway = 70.0;

    wf_est_t e;
    wf_est_init(&e, NULL);

    /* Half a window in, there is still no window. */
    uint32_t t = ride(&e, 0, WF_EST_CONS_WINDOW_M / 2.0, 36.0,
                      amps_for(town, 36.0));
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(!o.consumption_windowed,
          "half a window reported itself as a full one");
    CHECK(o.consumption_valid, "half a window and no fallback either");

    /* A full one, and the figure is the town's. */
    t = ride(&e, t, WF_EST_CONS_WINDOW_M / 2.0, 36.0, amps_for(town, 36.0));
    wf_est_get(&e, &o);
    CHECK(o.consumption_windowed, "a full window did not report itself as one");
    CHECK_D(o.window_m, WF_EST_CONS_WINDOW_M, WF_EST_CONS_BUCKET_M,
            "the window's own length");
    CHECK_D(o.consumption_wh_per_km, town, 0.5, "a kilometre of town riding");

    /* Onto the motorway. One window later the figure is the motorway's, and
     * the all-time average - which is where a trip meter would still be
     * sitting - is halfway between the two, so the two cannot be confused. */
    t = ride(&e, t, WF_EST_CONS_WINDOW_M, 90.0, amps_for(motorway, 90.0));
    wf_est_get(&e, &o);
    CHECK(o.consumption_windowed, "the window emptied on the motorway");
    CHECK_D(o.consumption_wh_per_km, motorway, 0.5, "a kilometre of motorway");
    CHECK_D(alltime_wh_per_km(&o), (town + motorway) / 2.0, 0.5,
            "the all-time average after one kilometre of each");
    CHECK(o.consumption_wh_per_km - alltime_wh_per_km(&o) > 15.0,
          "the windowed figure %.1f and the all-time %.1f are too close for "
          "this test to distinguish them",
          o.consumption_wh_per_km, alltime_wh_per_km(&o));

    /* And back into town: it falls again. A trip meter cannot do this. */
    ride(&e, t, WF_EST_CONS_WINDOW_M, 36.0, amps_for(town, 36.0));
    wf_est_get(&e, &o);
    CHECK_D(o.consumption_wh_per_km, town, 0.5,
            "back to town riding after the motorway");
    CHECK(alltime_wh_per_km(&o) > town + 5.0,
          "the all-time average fell back to the town figure too, so the "
          "window is not what is being measured");
}

/* "Improves across rides rather than being overwritten by the last one." That
 * is a statement about weights, and it is why the persisted state holds two
 * running totals and not a stored mean: a mean cannot be improved by a new
 * ride without also carrying the distance it was taken over.
 *
 * Three rides of equal length at 20, 40 and 60 Wh/km. Each one moves the
 * average by its share and no more, so the sequence is 20, 30, 40 - never 40,
 * which is what a stored mean overwritten by the last ride would give. */
static void test_the_all_time_average_improves_across_rides(void)
{
    const double leg_m = 2.0 * WF_EST_CONS_WINDOW_M;
    const double cost[3] = {20.0, 40.0, 60.0};
    const double want[3] = {20.0, 30.0, 40.0};
    static const char *dist_label[3] = {
        "the all-time distance after ride 1",
        "the all-time distance after ride 2",
        "the all-time distance after ride 3",
    };
    static const char *avg_label[3] = {
        "the all-time average after ride 1",
        "the all-time average after ride 2",
        "the all-time average after ride 3",
    };

    wf_est_persist_t saved;
    memset(&saved, 0, sizeof(saved));
    bool have_saved = false;

    for (int i = 0; i < 3; i++) {
        wf_est_t e;
        wf_est_init(&e, have_saved ? &saved : NULL);

        ride(&e, 0, leg_m, 36.0, amps_for(cost[i], 36.0));

        wf_est_out_t o;
        wf_est_get(&e, &o);
        CHECK_D(o.alltime_m, leg_m * (i + 1), 1.0, dist_label[i]);
        CHECK_D(alltime_wh_per_km(&o), want[i], 0.5, avg_label[i]);

        /* Out through the bytes and back, exactly as est_store.c does it. */
        uint8_t blob[WF_EST_PERSIST_BYTES];
        wf_est_save(&e, &saved);
        CHECK(wf_est_persist_encode(&saved, blob, sizeof(blob)),
              "encode failed on ride %d", i + 1);
        CHECK(wf_est_persist_decode(blob, sizeof(blob), &saved),
              "decode failed on ride %d", i + 1);
        have_saved = true;
    }

    /* The last ride was the most expensive of the three, and the average it
     * left behind is well below it. A stored mean would read 60. */
    CHECK(saved.alltime_wh / saved.alltime_m * 1000.0 < 50.0,
          "the all-time average was overwritten by the last ride");
}

/* The power-cycle criterion: encode the state mid-ride, decode it into a fresh
 * estimator, carry on, and the figure the rider is looking at does not step.
 *
 * The trap this test is built to avoid is passing by coincidence. If the
 * all-time average happened to equal the window, a Monitor that simply forgot
 * its window and fell back would look continuous, so the persisted history
 * here is deliberately a long way from the ride: fifty kilometres at 80 Wh/km
 * against a ride at 25. A fallback would show about 78 and the assertion is
 * against a step of half a watt-hour per kilometre. */
static void test_consumption_survives_a_power_cycle(void)
{
    const double ride_cost = 25.0;

    wf_est_persist_t history;
    memset(&history, 0, sizeof(history));
    history.version    = WF_EST_PERSIST_VERSION;
    history.alltime_m  = 50000.0f;
    history.alltime_wh = 50.0f * 80.0f;

    wf_est_t e;
    wf_est_init(&e, &history);
    uint32_t t = ride(&e, 0, 1.5 * WF_EST_CONS_WINDOW_M, 36.0,
                      amps_for(ride_cost, 36.0));

    wf_est_out_t before;
    wf_est_get(&e, &before);
    CHECK(before.consumption_windowed, "the ride did not fill a window");
    CHECK_D(before.consumption_wh_per_km, ride_cost, 0.5,
            "the figure before the break");
    CHECK(alltime_wh_per_km(&before) > 60.0,
          "the persisted history is %.1f Wh/km, too close to the ride's %.1f "
          "for a fallback to be distinguishable from continuity",
          alltime_wh_per_km(&before), ride_cost);

    /* The break. Out through the bytes and back into a fresh estimator, which
     * is all a power cycle is. */
    wf_est_persist_t saved;
    wf_est_save(&e, &saved);
    CHECK(saved.window_m >= (float)WF_EST_CONS_WINDOW_M,
          "a full window was not saved");
    uint8_t blob[WF_EST_PERSIST_BYTES];
    CHECK(wf_est_persist_encode(&saved, blob, sizeof(blob)), "encode failed");
    wf_est_persist_t back;
    CHECK(wf_est_persist_decode(blob, sizeof(blob), &back), "decode failed");

    wf_est_t after;
    wf_est_init(&after, &back);
    wf_est_out_t o;
    wf_est_get(&after, &o);
    CHECK(o.consumption_valid, "no Consumption figure at all after the break");
    CHECK(o.consumption_windowed,
          "the window was lost across the break and the figure fell back to "
          "the all-time average");
    CHECK_D(o.consumption_wh_per_km, before.consumption_wh_per_km, 0.5,
            "the Consumption figure across the break");

    /* And it carries on rather than merely surviving the instant: another two
     * hundred metres at the same cost keeps it there, which is the seeded
     * window being displaced by real buckets without a step. */
    double worst = 0.0;
    double prev = o.consumption_wh_per_km;
    for (int i = 0; i < 10; i++) {
        t = ride(&after, t, 100.0, 36.0, amps_for(ride_cost, 36.0));
        wf_est_get(&after, &o);
        double step = o.consumption_wh_per_km - prev;
        if (step < 0.0) {
            step = -step;
        }
        if (step > worst) {
            worst = step;
        }
        prev = o.consumption_wh_per_km;
    }
    CHECK(worst < 0.5, "Consumption stepped by %.2f Wh/km over the kilometre "
                       "after the break", worst);
    CHECK_D(prev, ride_cost, 0.5, "the figure a kilometre past the break");

    /* The all-time average kept the history and added the ride to it, so it
     * has moved a little and not been replaced. */
    CHECK(o.alltime_m > 52000.0,
          "the all-time distance lost the history: %.0f m", o.alltime_m);
    CHECK(alltime_wh_per_km(&o) > 60.0 && alltime_wh_per_km(&o) < 80.0,
          "the all-time average after the ride is %.1f Wh/km",
          alltime_wh_per_km(&o));
}

/* Builds the 24-byte layout versions 1 and 2 wrote, by hand rather than with
 * the encoder, because the encoder only ever writes the current version. The
 * distance fields are the ones version 1 left zeroed and version 2 spent. */
static void build_old_blob(uint8_t blob[WF_EST_PERSIST_BYTES_V2],
                           uint16_t version, float distance_m)
{
    memset(blob, 0, WF_EST_PERSIST_BYTES_V2);
    blob[0] = 0x57;                 /* 'W' */
    blob[1] = 0x45;                 /* 'E' */
    blob[2] = (uint8_t)version;     /* little endian, and both fit a byte */
    blob[4] = 1;                    /* valid */
    blob[5] = version >= 2 ? 1u : 0u;
    float coulomb = 20.0f, remaining = 1000.0f;
    float rated = (float)WF_EST_RATED_CAPACITY_AH;
    memcpy(&blob[6], &coulomb, sizeof(coulomb));
    memcpy(&blob[10], &remaining, sizeof(remaining));
    memcpy(&blob[14], &rated, sizeof(rated));
    if (version >= 2) {
        memcpy(&blob[18], &distance_m, sizeof(distance_m));
    }
    uint16_t crc = wf_crc16(blob, WF_EST_PERSIST_BYTES_V2 - 2, 0xffff);
    blob[22] = (uint8_t)(crc & 0xff);
    blob[23] = (uint8_t)(crc >> 8);
}

/* Version 1 is version 2 without distance, in bytes version 1 left zeroed. A
 * blob from that build has to be migrated deliberately - charge figures kept,
 * distance starting at nothing, which is what that build actually knew - and
 * not misread as a distance of whatever those bytes happened to hold. */
static void test_a_version_1_blob_is_migrated(void)
{
    uint8_t blob[WF_EST_PERSIST_BYTES_V2];
    build_old_blob(blob, 1, 0.0f);

    wf_est_persist_t p;
    CHECK(wf_est_persist_decode(blob, sizeof(blob), &p),
          "a version 1 blob was rejected rather than migrated");
    CHECK_U(p.version, 1, "the version the blob was written at");
    CHECK(p.valid, "the version 1 charge figures were dropped");
    CHECK_D(p.coulomb_ah, 20.0, 1e-6, "the migrated Coulomb Count");
    CHECK(!p.distance_valid, "a version 1 blob claimed to carry a distance");

    wf_est_t e;
    wf_est_init(&e, &p);
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.valid, "the migrated charge figures were not restored");
    CHECK_D(o.coulomb_ah, 20.0, 1e-6, "the restored Coulomb Count");
    CHECK(!o.distance_valid, "a version 1 blob restored a distance");
}

/* Version 2 is the layout this build replaced: 24 bytes, no Consumption. It is
 * migrated rather than rejected, because the alternative is throwing away a
 * rider's Distance and charge on the firmware update that added Consumption,
 * for nothing. What it restores is no all-time average and no window - exactly
 * what a build that did not compute Consumption knew - so the first ride after
 * the update rebuilds both from scratch.
 *
 * And the flip side, which is the whole reason the length is checked against
 * the version: bytes are never reinterpreted. A 24-byte blob claiming version
 * 3 would have its Consumption read out of a CRC and past the end of the
 * record; a 40-byte blob claiming version 2 would have version 3's fields
 * silently ignored and its CRC read from the wrong place. Both are rejected. */
static void test_a_version_2_blob_is_migrated(void)
{
    uint8_t blob[WF_EST_PERSIST_BYTES_V2];
    build_old_blob(blob, 2, 4321.0f);

    wf_est_persist_t p;
    CHECK(wf_est_persist_decode(blob, sizeof(blob), &p),
          "a version 2 blob was rejected rather than migrated");
    CHECK_U(p.version, 2, "the version the blob was written at");
    CHECK(p.valid, "the version 2 charge figures were dropped");
    CHECK(p.distance_valid, "the version 2 distance was dropped");
    CHECK_D(p.distance_m, 4321.0, 1e-6, "the migrated Distance");
    CHECK_D(p.alltime_m, 0.0, 1e-9, "a version 2 blob carried an all-time "
            "distance it cannot have written");
    CHECK_D(p.alltime_wh, 0.0, 1e-9, "a version 2 blob carried an all-time "
            "energy it cannot have written");
    CHECK_D(p.window_m, 0.0, 1e-9, "a version 2 blob carried a window");

    wf_est_t e;
    wf_est_init(&e, &p);
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.valid, "the migrated charge figures were not restored");
    CHECK_D(o.distance_m, 4321.0, 1e-6, "the restored Distance");
    CHECK(!o.consumption_valid,
          "a version 2 blob produced a Consumption figure out of nothing");

    /* The length and the version have to agree. */
    blob[2] = 3;
    uint16_t crc = wf_crc16(blob, WF_EST_PERSIST_BYTES_V2 - 2, 0xffff);
    blob[22] = (uint8_t)(crc & 0xff);
    blob[23] = (uint8_t)(crc >> 8);
    CHECK(!wf_est_persist_decode(blob, sizeof(blob), &p),
          "a 24-byte blob claiming version 3 decoded");

    uint8_t wide[WF_EST_PERSIST_BYTES];
    wf_est_persist_t cur = {
        .version           = 2,
        .valid             = true,
        .rated_capacity_ah = (float)WF_EST_RATED_CAPACITY_AH,
    };
    CHECK(wf_est_persist_encode(&cur, wide, sizeof(wide)), "encode failed");
    CHECK(!wf_est_persist_decode(wide, sizeof(wide), &p),
          "a 40-byte blob claiming version 2 decoded");

    /* And a version this build has never written is not read at all, however
     * good its checksum is. */
    cur.version = WF_EST_PERSIST_VERSION + 1;
    CHECK(wf_est_persist_encode(&cur, wide, sizeof(wide)), "encode failed");
    CHECK(!wf_est_persist_decode(wide, sizeof(wide), &p),
          "a blob from a future version decoded");
}

/* ---- Range -------------------------------------------------------------- *
 *
 * cap0007 produces no Consumption figure, so it produces no Range either, and
 * tests/fixtures/cap0007.expect pins exactly that as `est_range_source 0`. Every
 * behavioural assertion about Range therefore lives here, on synthesised riding,
 * for the same reason Consumption's do.
 *
 * Range needs both halves, so these tests feed both: one BMS answer to acquire
 * an Anchor and put a Remaining Energy on the board, and enough road to produce
 * a Consumption. The Anchor is acquired and then deliberately left to go stale -
 * WF_EST_ANCHOR_STALE_MS is five seconds and these rides are minutes long - so
 * the pull stops and the numerator is the integration alone. That is what
 * "absent regeneration" means for a test: nothing puts energy back and nothing
 * pulls the figure up, so any rise in Range is Consumption falling and nothing
 * else. The Anchor's own behaviour is asserted above, on its own.
 */

/* The State of Charge cap0007 sat at all ride, so the Remaining Energy these
 * tests start from is the same 2729 Wh that fixture pins. */
#define RANGE_SOC   66.7

/* What the rider actually reads: main/ui.c formats the hero row with "%.0f",
 * so this is the figure on the screen. Asserting on it as well as on the double
 * is the difference between "the number is steady" and "the number looks
 * steady". */
static double km_shown(double range_km)
{
    return (double)(long)(range_km + 0.5);
}

/* Watches the Range curve the way replay.c watches the energy curve: sampled at
 * every frame the rider could have looked at the screen, tracking the largest
 * upward move of both the raw figure and the one on the screen.
 *
 * The one sample it does not measure a step across is the handover, where the
 * ring fills and the window takes over from the persisted all-time average.
 * Range moves there by whatever the two figures differ by - which on a real
 * ride is the whole point and is usually large - so a monotonicity assertion
 * that spanned it would be asserting that the rider's history matches their
 * riding. It is counted instead, and its size recorded, so a test can assert
 * that it happened exactly once and how far it moved. */
typedef struct {
    bool   started;
    bool   windowed;
    double prev;
    double up_max;       /* largest rise of the raw figure, in km */
    double km_up_max;    /* largest rise of the whole-kilometre figure */
    double handover_step;/* signed move across the change of source */
    long   handovers;
    long   km_ups;       /* samples where the whole-kilometre figure rose */
    double first, last;
    long   samples;
} range_watch_t;

static void range_watch(range_watch_t *w, const wf_est_t *e)
{
    wf_est_out_t o;
    wf_est_get(e, &o);
    if (!o.range_valid) {
        return;
    }
    if (!w->started) {
        w->started  = true;
        w->first    = o.range_km;
        w->prev     = o.range_km;
        w->windowed = o.consumption_windowed;
    }
    if (o.consumption_windowed != w->windowed) {
        w->handover_step = o.range_km - w->prev;
        w->handovers++;
        w->windowed = o.consumption_windowed;
    } else {
        double up = o.range_km - w->prev;
        if (up > w->up_max) {
            w->up_max = up;
        }
        double km_up = km_shown(o.range_km) - km_shown(w->prev);
        if (km_up > 0.0) {
            w->km_ups++;
        }
        if (km_up > w->km_up_max) {
            w->km_up_max = km_up;
        }
    }
    w->prev = o.range_km;
    w->last = o.range_km;
    w->samples++;
}

/* A tiny deterministic generator, so "noise" is uncorrelated between samples
 * rather than an alternating pattern that cancels itself and proves nothing.
 * Same sequence on every machine and every run - these tests are as
 * reproducible as the replay is. */
static uint32_t noise_state = 12345u;

static double noise_lsb(void)
{
    noise_state = noise_state * 1103515245u + 12345u;
    /* -1..+1, uniform, in units of whatever the caller scales it by. */
    return (double)((noise_state >> 16) & 0xffffu) / 32767.5 - 1.0;
}

/* ride(), but sampling Range at every frame and optionally dithering the line
 * current by up to `dither` amps each sample. */
static uint32_t ride_watched(wf_est_t *e, range_watch_t *w, uint32_t t0_ms,
                             double metres, double kmh, double amps,
                             double dither)
{
    double per_tick = kmh * (1000.0 / 3600.0) * ((double)SAMPLE_MS / 1000.0);
    long ticks = (long)(metres / per_tick + 0.5);
    uint32_t t = t0_ms;

    feed_ride(e, t, kmh, amps);
    range_watch(w, e);
    for (long i = 0; i < ticks; i++) {
        t += SAMPLE_MS;
        feed_ride(e, t, kmh, amps + (dither > 0.0 ? dither * noise_lsb() : 0.0));
        range_watch(w, e);
    }
    return t;
}

/* The guards, and the assertion in each case is that no division happened at
 * all: `range_km` is left at exactly zero rather than at an infinity, a NaN or
 * the 40,000 km a real energy over almost no Consumption produces.
 *
 * Four ways to have no Range, and the estimator has to decline all four. */
static void test_range_declines_the_divisions_it_cannot_make(void)
{
    /* 1. No Remaining Energy: the Controller is streaming and the BMS has never
     *    answered, so there is a Consumption and nothing to divide. A confident
     *    "0 KM" here would read as "the ride is over". */
    wf_est_t e;
    wf_est_init(&e, NULL);
    ride(&e, 0, 2.0 * WF_EST_CONS_WINDOW_M, 36.0, amps_for(30.0, 36.0));

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.consumption_valid, "the ride produced no Consumption, so this test "
                               "is measuring the wrong guard");
    CHECK(!o.valid, "an estimate without a single BMS answer");
    CHECK(!o.range_valid, "a Range with no Remaining Energy to divide");
    CHECK_D(o.range_km, 0.0, 0.0, "something divided into an absent energy");

    /* 2. No Consumption: the bike has a Remaining Energy and has covered
     *    16.7 m, which is cap0007 exactly. */
    wf_est_t idle;
    wf_est_init(&idle, NULL);
    feed_soc(&idle, 0, RANGE_SOC);
    ride(&idle, 0, 16.7, 5.0, 2.0);
    wf_est_get(&idle, &o);
    CHECK(o.valid, "the BMS answer did not put an energy on the board");
    CHECK(!o.consumption_valid, "16.7 m produced a Consumption figure");
    CHECK(!o.range_valid, "a Range from cap0007's 16.7 m");
    CHECK_D(o.range_km, 0.0, 0.0, "something divided by cap0007's metres");

    /* 3. A Consumption below the floor. A kilometre bought for 2 Wh is not a
     *    riding style, and 2729 Wh divided by it is 1364 km - the answer the
     *    guard exists to refuse. */
    wf_est_t cheap;
    wf_est_init(&cheap, NULL);
    feed_soc(&cheap, 0, RANGE_SOC);
    ride(&cheap, 0, 2.0 * WF_EST_CONS_WINDOW_M, 36.0, amps_for(2.0, 36.0));
    wf_est_get(&cheap, &o);
    CHECK(o.consumption_valid && o.consumption_windowed,
          "the cheap ride produced no windowed Consumption");
    CHECK_D(o.consumption_wh_per_km, 2.0, 0.2, "the cheap ride's Consumption");
    CHECK(!o.range_valid, "a Range from %.1f Wh/km, below the %.1f floor",
          o.consumption_wh_per_km, WF_EST_RANGE_MIN_CONS_WH_PER_KM);
    CHECK_D(o.range_km, 0.0, 0.0, "a division below the Consumption floor");

    /* 4. A Consumption at or below zero - a window in which regeneration
     *    outweighed draw, which a long descent really does produce. The
     *    comparison against the floor is what rejects it, so no separate sign
     *    test is needed and none exists. */
    wf_est_t downhill;
    wf_est_init(&downhill, NULL);
    feed_soc(&downhill, 0, RANGE_SOC);
    ride(&downhill, 0, 2.0 * WF_EST_CONS_WINDOW_M, 36.0, amps_for(-20.0, 36.0));
    wf_est_get(&downhill, &o);
    CHECK(o.consumption_wh_per_km < 0.0,
          "the descent produced %.1f Wh/km, so the negative case is untested",
          o.consumption_wh_per_km);
    CHECK(!o.range_valid, "a Range from a negative Consumption");
    CHECK_D(o.range_km, 0.0, 0.0, "a division by a negative Consumption");

    /* And clear of the floor there is a Range again, so the guard is a floor
     * and not a wall: 6 Wh/km is above it and produces a figure. */
    wf_est_t ok;
    wf_est_init(&ok, NULL);
    feed_soc(&ok, 0, RANGE_SOC);
    ride(&ok, 0, 2.0 * WF_EST_CONS_WINDOW_M, 36.0, amps_for(6.0, 36.0));
    wf_est_get(&ok, &o);
    CHECK(o.range_valid, "no Range from 6 Wh/km, which is above the floor");
    CHECK_D(o.range_km, o.remaining_wh / o.consumption_wh_per_km, 1e-9,
            "Range is not Remaining Energy over Consumption");
}

/* The criterion the whole ticket is about, as one ride: a rider who is told
 * 91 km and rides steadily reaches the Limp Point after 91 km. Nothing else
 * asserts the numerator, the denominator and the division are describing the
 * same riding.
 *
 * Three things come out of the one ride.
 *
 * Monotonicity, under steady riding, which is the reading of the criterion
 * this implementation takes. Sampled at every frame, on the double and not on
 * the rounded figure, the Range never rises - not once in forty-five thousand
 * samples. It is deliberately not asserted globally: a rider who slows down
 * can genuinely go further, and test_range_drops_on_the_motorway() below
 * asserts the same figure moving the other way.
 *
 * A kilometre off the Range per kilometre ridden, which is the identity that
 * catches a numerator and a denominator that have drifted apart.
 *
 * And the crawl, which is excluded exactly once. Remaining Energy already
 * counts down to the Limp Point, so Range reaches zero there and the kilometre
 * of crawling below it is already outside the figure. If a second subtraction
 * were ever added here the ride would end a kilometre short of the Range it
 * started with, and this test would say so. */
static void test_range_counts_down_to_the_limp_point(void)
{
    const double cost = 30.0, kmh = 36.0;

    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_soc(&e, 0, RANGE_SOC);

    range_watch_t w;
    memset(&w, 0, sizeof(w));

    /* The first kilometre, on the persisted-average path: with no history at
     * all the all-time average is this ride's own, so there is a Range from
     * a hundred metres in. */
    uint32_t t = ride_watched(&e, &w, 0, WF_EST_CONS_WINDOW_M, kmh,
                              amps_for(cost, kmh), 0.0);
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.range_valid, "no Range after the first kilometre");
    CHECK(o.consumption_windowed, "the first kilometre did not fill the window");
    CHECK_D(o.range_km, wf_est_energy_above_limp_wh(RANGE_SOC) / cost - 1.0, 1.0,
            "the Range a kilometre into the ride");

    /* The rest of it, in kilometre legs, until the Limp Point. */
    double start_km = w.first;
    long legs = 0;
    while (o.range_km > 0.0 && legs < 400) {
        t = ride_watched(&e, &w, t, WF_EST_CONS_WINDOW_M, kmh,
                         amps_for(cost, kmh), 0.0);
        wf_est_get(&e, &o);
        legs++;
    }

    CHECK(o.range_valid, "Range stopped being a figure before the Limp Point");
    CHECK(o.range_km == 0.0, "the ride ended at %.3f km, not at zero",
          o.range_km);
    CHECK(o.remaining_wh == 0.0,
          "Range reached zero with %.1f Wh still above the Limp Point",
          o.remaining_wh);

    /* The identity. The Range the rider was shown a kilometre in was 90 km and
     * the ride lasted another 90 km, to within the kilometre the legs are
     * measured in - no crawl subtracted twice, no kilometre lost. */
    CHECK(w.samples > 40000,
          "only %ld samples: the ride was too short to assert monotonicity on",
          w.samples);
    CHECK_D(o.distance_m / 1000.0, start_km + 1.0, 1.5,
            "the distance ridden against the Range promised at the start");

    /* Monotone, on the raw double, at every one of those samples. Under steady
     * riding this is exact and not a tolerance: Consumption does not move, so
     * Range is a falling energy over a constant. */
    CHECK(w.up_max == 0.0,
          "Range rose by %.6f km during a steady ride", w.up_max);
    CHECK_D(w.km_up_max, 0.0, 0.0,
            "the kilometre figure on the screen ticked up during a steady ride");

    /* The one place the curve is allowed to move upward, and it happens once:
     * the kilometre mark, where the window takes over from the all-time
     * average. This ride has no persisted history, so the two figures are the
     * same riding measured two ways and the step is the frame-interval offset
     * between the energy and distance integrals - well under a kilometre on a
     * 90 km Range, and invisible after the screen rounds it. On a ride whose
     * history is unlike its riding the same step is large and is the figure
     * being corrected, which is why it is not asserted small in general. */
    CHECK_U(w.handovers, 1, "changes of Consumption source over the ride");
    CHECK(w.handover_step > 0.0 && w.handover_step < 0.5,
          "the handover from the all-time average to the window moved Range by "
          "%.3f km", w.handover_step);
}

/* The other half of the tension, and issue #1's fourth user story: the rider
 * joins a motorway and the Range drops rather than staying optimistic until
 * they are stranded. This is the case a globally monotone Range would get
 * right and a globally monotone Range would get the previous test's rise
 * wrong, which is why the criterion is qualified rather than absolute.
 *
 * The reaction has to be visible inside the window, so it is asserted at three
 * hundred metres - well inside the kilometre - and not merely at the end. */
static void test_range_drops_on_the_motorway(void)
{
    const double town = 30.0, motorway = 70.0;

    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_soc(&e, 0, RANGE_SOC);

    uint32_t t = ride(&e, 0, 2.0 * WF_EST_CONS_WINDOW_M, 36.0,
                      amps_for(town, 36.0));
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.consumption_windowed, "two kilometres did not fill the window");
    double in_town = o.range_km;
    CHECK(in_town > 50.0, "the town Range is %.1f km, too small to fall from",
          in_town);

    /* Three hundred metres of motorway: a third of the window has been
     * replaced, and the figure has already moved a long way. */
    t = ride(&e, t, 300.0, 90.0, amps_for(motorway, 90.0));
    wf_est_get(&e, &o);
    double early = o.range_km;
    CHECK(early < in_town * 0.85,
          "300 m of motorway moved the Range from %.1f to %.1f km, which is "
          "not a reaction inside the window", in_town, early);

    /* And a full window of it, where the figure is the motorway's alone. */
    t = ride(&e, t, WF_EST_CONS_WINDOW_M, 90.0, amps_for(motorway, 90.0));
    wf_est_get(&e, &o);
    CHECK_D(o.consumption_wh_per_km, motorway, 0.5, "a kilometre of motorway");
    CHECK(o.range_km < in_town * 0.6,
          "a kilometre of motorway left the Range at %.1f km against the "
          "town's %.1f", o.range_km, in_town);

    /* Back into town, and it rises again. That is the feature and not a fault:
     * the rider really can go further at the lower cost, and a Range that
     * refused to say so would be a ratchet. */
    double lowest = o.range_km;
    ride(&e, t, 2.0 * WF_EST_CONS_WINDOW_M, 36.0, amps_for(town, 36.0));
    wf_est_get(&e, &o);
    CHECK(o.range_km > lowest * 1.5,
          "back in town the Range stayed at %.1f km against %.1f on the "
          "motorway, so it is a ratchet rather than an estimate",
          o.range_km, lowest);
}

/* "Available in the first kilometre via the persisted average" - the whole
 * reason the all-time totals are persisted at all.
 *
 * The history here is deliberately unlike the ride, so a Monitor that quietly
 * ignored it and waited for its own window would fail: fifty kilometres at
 * 80 Wh/km, against a ride that costs 30. */
static void test_range_is_there_in_the_first_kilometre(void)
{
    wf_est_persist_t history;
    memset(&history, 0, sizeof(history));
    history.version    = WF_EST_PERSIST_VERSION;
    history.alltime_m  = 50000.0f;
    history.alltime_wh = 50.0f * 80.0f;

    wf_est_t e;
    wf_est_init(&e, &history);
    feed_soc(&e, 0, RANGE_SOC);

    /* Under the Consumption floor there is no Range yet: 50 m is half an
     * Odometer count and the ride has no window. */
    uint32_t t = ride(&e, 0, 50.0, 36.0, amps_for(30.0, 36.0));
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.range_valid,
          "the persisted average did not give a Range in the first 50 m");
    CHECK(!o.consumption_windowed,
          "50 m of this ride reported itself as a full window");

    /* Two hundred metres in - a fifth of a window, and nowhere near one - the
     * Range is the persisted average's and is marked as such, which is the
     * star on the hero row in main/ui.c. */
    t = ride(&e, t, 150.0, 36.0, amps_for(30.0, 36.0));
    wf_est_get(&e, &o);
    CHECK(!o.consumption_windowed,
          "200 m of riding claimed a full %.0f m window", WF_EST_CONS_WINDOW_M);
    CHECK_D(o.range_km, o.remaining_wh / 80.0, 1.0,
            "the first-kilometre Range is not the persisted average's");
    CHECK(o.range_km < 40.0,
          "the Range came out at %.1f km, which is this ride's own cost rather "
          "than the persisted history's", o.range_km);

    /* A kilometre later the window has filled and the figure is the ride's
     * own, which is much larger because this ride is far cheaper than the
     * history. The handover is a change of source and the screen says so. */
    ride(&e, t, WF_EST_CONS_WINDOW_M, 36.0, amps_for(30.0, 36.0));
    wf_est_get(&e, &o);
    CHECK(o.consumption_windowed, "a kilometre did not fill the window");
    CHECK(o.range_km > 80.0,
          "the windowed Range is %.1f km, not the ~90 km a 30 Wh/km ride buys",
          o.range_km);
}

/* "It cannot jitter upward on noise."
 *
 * There is no filter on Range: the steadiness comes from Consumption's ring,
 * which moves once per fifty metres of road and by at most a twentieth of the
 * difference between the bucket entering the window and the one leaving it. The
 * claim being tested is that this is enough, and that nothing further is
 * needed.
 *
 * The noise is the instrument's own and not the rider's: one LSB of line
 * current, WF_CTRL_CURRENT_LSB_PER_A being 4, dithered independently onto every
 * one of the ~45000 samples. Anything larger than that is the throttle moving,
 * which is riding and which the window is supposed to follow.
 *
 * The assertion is on the figure the rider reads - main/ui.c's "%.0f" - and it
 * is that it never ticks up. The raw double may wobble by a few metres at a
 * bucket boundary, and is allowed to: a rise there is bounded well below the
 * fifty metres of Range that the same fifty metres of road takes off it. */
static void test_range_does_not_jitter_up_on_a_noisy_current(void)
{
    const double cost = 30.0, kmh = 36.0;

    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_soc(&e, 0, RANGE_SOC);

    range_watch_t w;
    memset(&w, 0, sizeof(w));
    noise_state = 12345u;

    /* Ten kilometres of it, which is two hundred bucket boundaries - every one
     * of them an opportunity for the window to step the wrong way. */
    ride_watched(&e, &w, 0, 10.0 * WF_EST_CONS_WINDOW_M, kmh,
                 amps_for(cost, kmh), 0.5 / WF_CTRL_CURRENT_LSB_PER_A);

    CHECK(w.samples > 4000, "only %ld samples of noisy riding", w.samples);
    CHECK_U(w.handovers, 1, "changes of Consumption source over ten kilometres");
    CHECK(w.last < w.first - 9.0,
          "ten kilometres of riding took the Range from %.2f km to %.2f km, "
          "which is not ten kilometres off it", w.first, w.last);

    /* The number that decides whether this works. A bucket of road takes about
     * 0.05 km off a 90 km Range; the noise moves the window by a fiftieth of a
     * percent, which is 0.02 km. So the riding outruns the noise by more than
     * two to one at every bucket boundary, and the trend the rider watches is
     * the road and not the instrument. */
    CHECK(w.up_max < WF_EST_CONS_BUCKET_M / 1000.0,
          "Range rose by %.6f km on noise alone, more than the %.3f km a "
          "bucket of road takes off it", w.up_max,
          WF_EST_CONS_BUCKET_M / 1000.0);
    CHECK(w.up_max / w.first < 0.0005,
          "the noise moved Range by %.4f %% of itself",
          100.0 * w.up_max / w.first);

    /* What the rider sees, and the honest limit of the claim, written down
     * rather than asserted away.
     *
     * Ten kilometres of this takes ten kilometres off the Range, so the
     * displayed figure counts down about ten times. Three times in the same
     * ten kilometres it ticks back up by one - the raw value happened to be
     * within 0.02 km of a rounding boundary when a bucket closed. It never
     * moves up by more than one, and it never moves up twice running.
     *
     * That could be removed, and deliberately is not. It would take a filter
     * on Range itself - a ratchet, or hysteresis wide enough to swallow half a
     * kilometre - and that filter cannot tell this apart from the rise a rider
     * earns by easing off, which is the figure moving for exactly the right
     * reason and is what the acceptance criterion above it asks for. A
     * hundredth-of-a-percent wobble is also two thousand times smaller than
     * the 19 % this figure is uncertain by while WF_CTRL_CURRENT_LSB_PER_A is
     * unsettled. The bound below is a regression guard on that trade, not a
     * claim that three is the right number. */
    CHECK(w.km_up_max <= 1.0,
          "the displayed Range jumped up by %.0f km on noise", w.km_up_max);
    CHECK(w.km_ups <= 5,
          "the displayed Range ticked up %ld times in ten kilometres of "
          "steady riding, against the three the rounding accounts for",
          w.km_ups);
}

/* "Range survives a simulated power cycle mid-ride without a jump."
 *
 * tests/host/replay.c drives the same break through a real Capture, but
 * cap0007 has no Range on either side of it, so the figure's continuity is
 * asserted here. Both halves of the quotient have to cross the break intact -
 * Remaining Energy from the persisted count, Consumption from the persisted
 * window - and the trap is the same one the Consumption test avoids: the
 * persisted history is deliberately a long way from the riding, so a Monitor
 * that forgot its window and fell back to a lifetime average would show a
 * different Range rather than the same one. */
static void test_range_survives_a_power_cycle(void)
{
    const double cost = 30.0, kmh = 36.0;

    wf_est_persist_t history;
    memset(&history, 0, sizeof(history));
    history.version    = WF_EST_PERSIST_VERSION;
    history.alltime_m  = 50000.0f;
    history.alltime_wh = 50.0f * 80.0f;

    wf_est_t e;
    wf_est_init(&e, &history);
    feed_soc(&e, 0, RANGE_SOC);
    uint32_t t = ride(&e, 0, 1.5 * WF_EST_CONS_WINDOW_M, kmh,
                      amps_for(cost, kmh));

    wf_est_out_t before;
    wf_est_get(&e, &before);
    CHECK(before.range_valid && before.consumption_windowed,
          "the ride before the break produced no windowed Range");
    CHECK(before.range_km > 80.0,
          "the Range before the break is %.1f km", before.range_km);

    /* The break: out through the persisted bytes and back into a fresh
     * estimator, which is all a power cycle is. */
    wf_est_persist_t saved, back;
    uint8_t blob[WF_EST_PERSIST_BYTES];
    wf_est_save(&e, &saved);
    CHECK(wf_est_persist_encode(&saved, blob, sizeof(blob)), "encode failed");
    CHECK(wf_est_persist_decode(blob, sizeof(blob), &back), "decode failed");

    wf_est_t after;
    wf_est_init(&after, &back);
    wf_est_out_t o;
    wf_est_get(&after, &o);
    CHECK(o.range_valid, "no Range at all on the far side of the break");
    CHECK(!o.anchored, "a restored estimator claimed to be Anchored");
    CHECK_D(o.range_km, before.range_km, 0.5,
            "the Range across the break");

    /* A jump the instant after the break would be the persisted window not
     * being believed, and a jump a kilometre later would be it not being
     * displaced smoothly. Both are watched.
     *
     * There is one rise the break genuinely does cost, and it is bounded here
     * rather than waved at. A restored estimator has no previous timestamp to
     * integrate from, so the first power-block frame after it only starts the
     * clock and its interval's energy is never booked - which is the correct
     * answer, because the Monitor was off across it. That one missing sample
     * makes the first bucket after the break slightly cheaper than the road it
     * covers, so the window dips and Range rises by the same fraction, until
     * the bucket ages out a kilometre later. tests/host/replay.c budgets the
     * same lost step in watt-hours; this is that step expressed in kilometres
     * of Range. */
    double lost_wh   = CONS_VOLTS * amps_for(cost, kmh) *
                       ((double)SAMPLE_MS / 3600000.0);
    double window_wh = cost * (WF_EST_CONS_WINDOW_M / 1000.0);
    double budget_km = before.range_km * lost_wh / window_wh;

    range_watch_t w;
    memset(&w, 0, sizeof(w));
    double prev = o.range_km;
    double worst = 0.0;
    for (int i = 0; i < 10; i++) {
        t = ride_watched(&after, &w, t, 100.0, kmh, amps_for(cost, kmh), 0.0);
        wf_est_get(&after, &o);
        double step = o.range_km - prev;
        if (step < 0.0) {
            step = -step;
        }
        if (step > 0.2 && step > worst) {
            worst = step;
        }
        prev = o.range_km;
    }
    CHECK(worst == 0.0,
          "Range stepped by %.3f km over the kilometre after the break, "
          "beyond the 0.1 km that kilometre should cost it", worst);
    CHECK_U(w.handovers, 0,
            "changes of Consumption source after the break: the window was "
            "restored, so there is nothing to hand over from");
    CHECK(w.up_max <= budget_km,
          "Range rose by %.6f km after the break, beyond the %.6f km the one "
          "integration step the break costs accounts for", w.up_max, budget_km);
    CHECK(w.km_ups <= 1 && w.km_up_max <= 1.0,
          "the displayed Range ticked up %ld times, by up to %.0f km, over the "
          "kilometre after the break", w.km_ups, w.km_up_max);
    CHECK_D(prev, before.range_km - 1.0, 0.5,
            "the Range a kilometre past the break, which should be a kilometre "
            "less than the Range before it");
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
    test_motion_block_decodes_at_every_type();
    test_the_undecoded_third_block_changes_nothing();

    test_limp_point_model();
    test_nothing_before_the_first_anchor();
    test_the_first_anchor_acquires();
    test_sign_convention();
    test_the_coulomb_count_tracks_the_anchor();
    test_a_bms_gap_produces_no_step();
    test_only_the_power_block_integrates();
    test_a_long_gap_is_clamped();

    test_no_distance_before_either_source();
    test_distance_integrates_speed();
    test_the_odometer_corrects_drift_without_a_step();
    test_an_odometer_wrap_changes_nothing();
    test_an_odometer_that_jumps_is_not_believed();
    test_a_controller_link_drop_loses_no_distance();
    test_distance_before_the_wheel_geometry();

    test_the_same_input_gives_the_same_state();
    test_persisted_state_round_trips();
    test_a_restored_count_is_not_an_anchor();
    test_distance_survives_a_power_cycle();

    test_a_standing_bike_produces_no_consumption();
    test_consumption_follows_the_recent_kilometre();
    test_the_all_time_average_improves_across_rides();
    test_consumption_survives_a_power_cycle();

    test_range_declines_the_divisions_it_cannot_make();
    test_range_counts_down_to_the_limp_point();
    test_range_drops_on_the_motorway();
    test_range_is_there_in_the_first_kilometre();
    test_range_does_not_jitter_up_on_a_noisy_current();
    test_range_survives_a_power_cycle();

    test_a_version_1_blob_is_migrated();
    test_a_version_2_blob_is_migrated();

    if (failures != 0) {
        printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
        return 1;
    }
    printf("unit: odometer wrap, the power block and the estimator, "
           "all assertions hold\n");
    return 0;
}
