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

/* Version 1 is version 2 without distance, in bytes version 1 left zeroed. A
 * blob from that build has to be migrated deliberately - charge figures kept,
 * distance starting at nothing, which is what that build actually knew - and
 * not misread as a distance of whatever those bytes happened to hold. */
static void test_a_version_1_blob_is_migrated(void)
{
    /* Built by hand rather than by the encoder, because the encoder only
     * writes the current version. This is the byte layout version 1 wrote. */
    uint8_t blob[WF_EST_PERSIST_BYTES];
    memset(blob, 0, sizeof(blob));
    blob[0] = 0x57;                 /* 'W' */
    blob[1] = 0x45;                 /* 'E' */
    blob[2] = 1;                    /* version 1, little endian */
    blob[4] = 1;                    /* valid */
    float coulomb = 20.0f, remaining = 1000.0f;
    float rated = (float)WF_EST_RATED_CAPACITY_AH;
    memcpy(&blob[6], &coulomb, sizeof(coulomb));
    memcpy(&blob[10], &remaining, sizeof(remaining));
    memcpy(&blob[14], &rated, sizeof(rated));
    uint16_t crc = wf_crc16(blob, WF_EST_PERSIST_BYTES - 2, 0xffff);
    blob[22] = (uint8_t)(crc & 0xff);
    blob[23] = (uint8_t)(crc >> 8);

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

    /* And the flip side: a version this build has never written is not read
     * at all, however good its checksum is. */
    blob[2] = WF_EST_PERSIST_VERSION + 1;
    crc = wf_crc16(blob, WF_EST_PERSIST_BYTES - 2, 0xffff);
    blob[22] = (uint8_t)(crc & 0xff);
    blob[23] = (uint8_t)(crc >> 8);
    CHECK(!wf_est_persist_decode(blob, sizeof(blob), &p),
          "a blob from a future version decoded");
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
    test_a_version_1_blob_is_migrated();

    if (failures != 0) {
        printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
        return 1;
    }
    printf("unit: odometer wrap, the power block and the estimator, "
           "all assertions hold\n");
    return 0;
}
