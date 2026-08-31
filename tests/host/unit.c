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

/* A NONSENSE FLOAT IN THE BLOB MUST NOT SURVIVE THE RESTORE, which is what
 * sane_f() is for and what the three most load-bearing figures were skipping.
 *
 * The threat is not a corrupted record - the CRC catches those. It is a record
 * whose checksum is perfectly good and whose bytes are not a number: a flash
 * cell that flipped before the CRC was taken, a struct written by a build with
 * a different layout, an arithmetic slip upstream that saved an infinity. The
 * estimator has no way to tell those apart from a real figure, so it checks
 * the value rather than the record.
 *
 * A NaN that got in would not merely be wrong once. Remaining Energy is
 * corrected by `x += (target - x) * k`, which is NaN for every k, so no BMS
 * answer can ever pull it back; Range is a quotient of it and goes with it;
 * and wf_est_save() writes it out again under a fresh valid CRC. It is the one
 * class of error in this module that a power cycle makes worse rather than
 * better.
 *
 * Built as bit patterns rather than as NAN and INFINITY so that the test says
 * what a corrupted record actually looks like, and so that nothing here needs
 * math.h. */
static float float_from_bits(uint32_t bits)
{
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

static bool is_a_number(double v)
{
    return v > -1e30 && v < 1e30;
}

/* One poisoned blob, all the way through the round trip a power cycle is. */
static void check_nonsense_is_refused(const char *what,
                                      const wf_est_persist_t *poisoned)
{
    uint8_t blob[WF_EST_PERSIST_BYTES];
    wf_est_persist_t back;
    CHECK(wf_est_persist_encode(poisoned, blob, sizeof(blob)),
          "%s: encode failed", what);
    CHECK(wf_est_persist_decode(blob, sizeof(blob), &back),
          "%s: the blob did not survive its own CRC, so this case is not "
          "testing the restore", what);

    wf_est_t e;
    wf_est_init(&e, &back);
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(is_a_number(o.coulomb_ah) && is_a_number(o.remaining_wh) &&
          is_a_number(o.distance_m) && is_a_number(o.range_km) &&
          is_a_number(o.soc_pct) && is_a_number(o.usable_frac),
          "%s: restored, and the estimator now shows coulomb %f Ah, "
          "remaining %f Wh, distance %f m, range %f km", what, o.coulomb_ah,
          o.remaining_wh, o.distance_m, o.range_km);

    /* And it does not come back on the next save. A figure the estimator
     * refused to restore must not be written out again under a fresh CRC, or
     * the record repairs itself into the same state at every power cycle. */
    wf_est_persist_t again;
    wf_est_save(&e, &again);
    CHECK(is_a_number((double)again.coulomb_ah) &&
          is_a_number((double)again.remaining_wh) &&
          is_a_number((double)again.distance_m),
          "%s: the estimator saved the nonsense straight back out", what);

    /* Riding on top of it does not resurrect it either: a ride that starts
     * from a refused restore is a ride that starts cold. */
    feed_soc(&e, 0, 80.0);
    for (uint32_t t = SAMPLE_MS; t <= 60000u; t += SAMPLE_MS) {
        feed_ride(&e, t, 40.0, amps_for(25.0, 40.0));
    }
    wf_est_get(&e, &o);
    CHECK(is_a_number(o.remaining_wh) && is_a_number(o.distance_m) &&
          (!o.range_valid || is_a_number(o.range_km)),
          "%s: a minute of riding on the refused restore produced remaining "
          "%f Wh, distance %f m, range %f km", what, o.remaining_wh,
          o.distance_m, o.range_km);
}

static void test_a_nonsense_restore_is_refused(void)
{
    const uint32_t quiet_nan = 0x7fc00000u;
    const uint32_t pos_inf   = 0x7f800000u;

    wf_est_persist_t good;
    memset(&good, 0, sizeof(good));
    good.version           = WF_EST_PERSIST_VERSION;
    good.valid             = true;
    good.coulomb_ah        = 15.0f;
    good.remaining_wh      = 750.0f;
    good.rated_capacity_ah = (float)WF_EST_RATED_CAPACITY_AH;
    good.distance_valid    = true;
    good.distance_m        = 8000.0f;
    good.alltime_m         = 50000.0f;
    good.alltime_wh        = 4000.0f;

    /* The control: the same blob with nothing wrong with it restores, so a
     * pass below cannot be the restore having failed for some other reason. */
    wf_est_t e;
    wf_est_init(&e, &good);
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.valid && o.distance_valid,
          "the un-poisoned control blob did not restore, so the cases below "
          "assert nothing");
    CHECK_D(o.remaining_wh, 750.0, 1e-6, "the control's Remaining Energy");

    wf_est_persist_t p;

    p = good;
    p.remaining_wh = float_from_bits(quiet_nan);
    check_nonsense_is_refused("a NaN Remaining Energy", &p);

    p = good;
    p.remaining_wh = float_from_bits(pos_inf);
    check_nonsense_is_refused("an infinite Remaining Energy", &p);

    p = good;
    p.coulomb_ah = float_from_bits(quiet_nan);
    check_nonsense_is_refused("a NaN Coulomb Count", &p);

    p = good;
    p.coulomb_ah = 1e31f;
    check_nonsense_is_refused("a Coulomb Count of 1e31 Ah", &p);

    p = good;
    p.distance_m = float_from_bits(pos_inf);
    check_nonsense_is_refused("an infinite Distance", &p);

    p = good;
    p.distance_m = 1e31f;
    check_nonsense_is_refused("a Distance of 1e31 m", &p);
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
          "a %d-byte blob claiming version 2 decoded", WF_EST_PERSIST_BYTES);

    /* And a version this build has never written is not read at all, however
     * good its checksum is. */
    cur.version = WF_EST_PERSIST_VERSION + 1;
    CHECK(wf_est_persist_encode(&cur, wide, sizeof(wide)), "encode failed");
    CHECK(!wf_est_persist_decode(wide, sizeof(wide), &p),
          "a blob from a future version decoded");
}

/* Version 3 is the layout Internal Resistance replaced: 40 bytes, everything
 * this build knows except a resistance and the weight behind it. Migrated for
 * the same reason version 2 was - a rider should not lose their Distance,
 * their charge and their Consumption history to the firmware update that
 * started measuring their Pack.
 *
 * Built as a prefix of the current layout rather than field by field, because
 * that is exactly what it is: bytes 0..37 are identical and only the CRC moves
 * to the end of the shorter record. If that ever stopped being true this test
 * would be the thing that noticed. */
static void build_v3_blob(uint8_t blob[WF_EST_PERSIST_BYTES],
                          const wf_est_persist_t *from)
{
    wf_est_persist_t v3 = *from;
    v3.version = 3;
    if (!wf_est_persist_encode(&v3, blob, WF_EST_PERSIST_BYTES)) {
        return;
    }
    uint16_t crc = wf_crc16(blob, WF_EST_PERSIST_BYTES_V3 - 2, 0xffff);
    blob[WF_EST_PERSIST_BYTES_V3 - 2] = (uint8_t)(crc & 0xff);
    blob[WF_EST_PERSIST_BYTES_V3 - 1] = (uint8_t)(crc >> 8);
}

static void test_a_version_3_blob_is_migrated(void)
{
    wf_est_persist_t saved;
    memset(&saved, 0, sizeof(saved));
    saved.valid             = true;
    saved.coulomb_ah        = 30.0f;
    saved.remaining_wh      = 1500.0f;
    saved.rated_capacity_ah = (float)WF_EST_RATED_CAPACITY_AH;
    saved.distance_valid    = true;
    saved.distance_m        = 8000.0f;
    saved.alltime_wh        = 4000.0f;
    saved.alltime_m         = 50000.0f;
    /* A version 3 build could not have written these two, and this is the
     * check that they are not read out of the bytes it did write. */
    saved.ir_ohm            = 0.050f;
    saved.ir_weight         = 40;

    uint8_t blob[WF_EST_PERSIST_BYTES];
    build_v3_blob(blob, &saved);

    wf_est_persist_t p;
    CHECK(wf_est_persist_decode(blob, WF_EST_PERSIST_BYTES_V3, &p),
          "a version 3 blob was rejected rather than migrated");
    CHECK_U(p.version, 3, "the version the blob was written at");
    CHECK_D(p.distance_m, 8000.0, 1e-6, "the migrated Distance");
    CHECK_D(p.alltime_m, 50000.0, 1e-6, "the migrated all-time distance");
    CHECK_D(p.ir_ohm, 0.0, 0.0,
            "a version 3 blob carried an Internal Resistance it cannot have "
            "written");
    CHECK_U(p.ir_weight, 0, "a version 3 blob carried a weight");

    wf_est_t e;
    wf_est_init(&e, &p);
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.valid, "the migrated charge figures were not restored");
    CHECK_D(o.distance_m, 8000.0, 1e-6, "the restored Distance");
    CHECK(!o.ir_valid,
          "a version 3 blob produced an Internal Resistance out of nothing");
    CHECK_D(o.sag_v, 0.0, 0.0, "Sag from a blob that carried no resistance");

    /* The length and the version have to agree, both ways round. */
    CHECK(!wf_est_persist_decode(blob, WF_EST_PERSIST_BYTES, &p),
          "a %d-byte blob claiming version 3 decoded", WF_EST_PERSIST_BYTES);
    saved.version = 4;
    CHECK(wf_est_persist_encode(&saved, blob, sizeof(blob)), "encode failed");
    CHECK(!wf_est_persist_decode(blob, WF_EST_PERSIST_BYTES_V3, &p),
          "a %d-byte blob claiming version 4 decoded", WF_EST_PERSIST_BYTES_V3);
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

/* ---- Internal Resistance and the Limp Point that moves --------------------
 *
 * cap0007 cannot say a word about any of this. Its line current runs
 * -0.25..8.75 A over 47 s of crawling, so the largest load step in the whole
 * ride is under half of WF_EST_IR_MIN_DI_A and the fixture's own expected
 * values pin exactly that: no steps, no estimate, no Sag. Writing a test that
 * appeared to exercise step detection against a ride that never launched would
 * assert nothing at all, so - as with Consumption and Range before it - the
 * physics is driven with synthesised streams here.
 *
 * The synthesis is a Pack with a known Internal Resistance seen through the
 * two instruments' real quantisation: the terminal voltage is the open-circuit
 * voltage less IR_TRUE_OHM per amp, rounded to the 0.1 V the Controller
 * reports, and the current is rounded to WF_CTRL_CURRENT_LSB_PER_A. So every
 * ratio these tests measure carries the same quantisation error a real one
 * would, and the thresholds are being tested against the thing they were
 * derived from rather than against clean arithmetic.
 */

#define IR_TRUE_OHM   0.050     /* what the synthesised Pack really is */
#define IR_OPEN_V     105.30    /* cap0007's rested Pack, near enough */
#define IR_REST_A     5.0       /* hotel load, between launches */
#define IR_SOC        66.7

/* Round half away from zero, which is what both instruments do to their own
 * readings and what makes these streams quantised rather than merely coarse. */
static double quantise(double x, double step)
{
    double n = x / step;
    long   k = (long)(n + (n >= 0.0 ? 0.5 : -0.5));
    return (double)k * step;
}

static double pack_amps(double amps)
{
    return quantise(amps, 1.0 / WF_CTRL_CURRENT_LSB_PER_A);
}

/* The terminal voltage this Pack shows at that current, as the Controller
 * would report it. `open_v` varies between launches in the tests below so the
 * quantisation error is not the same on every step - a running mean that only
 * ever sees one rounding is not being averaged, it is being repeated. */
static double pack_volts(double open_v, double amps, double ohm)
{
    return quantise(open_v - ohm * pack_amps(amps), 0.1);
}

/* One tick of a Pack under load: the power-block frame, and a BMS answer once
 * a second so the Anchor stays fresh - which is one of the acceptance rules a
 * load step has to pass. */
static void feed_pack(wf_est_t *e, uint32_t t_ms, double open_v, double amps,
                      double ohm)
{
    if (t_ms % POLL_MS < SAMPLE_MS) {
        feed_soc(e, t_ms, IR_SOC);
    }
    feed_power(e, t_ms, pack_volts(open_v, amps, ohm), pack_amps(amps));
}

/* A launch and the release at the end of it: two sharp steps, which is what a
 * rider gives the estimator for free every time they open the throttle. Held
 * for `hold_ms` in between, at `peak_a`. Returns the time it ended at. */
static uint32_t ir_launch(wf_est_t *e, uint32_t t0_ms, double open_v,
                          double peak_a, uint32_t hold_ms, double ohm)
{
    uint32_t t = t0_ms;
    feed_pack(e, t, open_v, IR_REST_A, ohm);
    for (uint32_t held = 0; held < hold_ms; held += SAMPLE_MS) {
        t += SAMPLE_MS;
        feed_pack(e, t, open_v, peak_a, ohm);
    }
    t += SAMPLE_MS;
    feed_pack(e, t, open_v, IR_REST_A, ohm);
    return t;
}

/* The same launch with the BMS silent: the Controller stream is identical and
 * the Anchor goes stale underneath it, which is the gap case. */
static uint32_t ir_launch_no_bms(wf_est_t *e, uint32_t t0_ms, double open_v,
                                 double peak_a, uint32_t hold_ms, double ohm)
{
    uint32_t t = t0_ms;
    feed_power(e, t, pack_volts(open_v, IR_REST_A, ohm), pack_amps(IR_REST_A));
    for (uint32_t held = 0; held < hold_ms; held += SAMPLE_MS) {
        t += SAMPLE_MS;
        feed_power(e, t, pack_volts(open_v, peak_a, ohm), pack_amps(peak_a));
    }
    t += SAMPLE_MS;
    feed_power(e, t, pack_volts(open_v, IR_REST_A, ohm), pack_amps(IR_REST_A));
    return t;
}

/* The estimate itself: measured from load steps, stable, and inside the
 * plausibility bound.
 *
 * Twenty launches, each at a different peak and each on a slightly different
 * open-circuit voltage, so the forty steps carry forty different roundings
 * rather than one repeated. What comes out has to be the Pack that was
 * synthesised, and - the criterion that matters for a figure on a screen - it
 * has to stop moving once it has seen a few of them. */
static void test_internal_resistance_comes_from_load_steps(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(!o.ir_valid, "an Internal Resistance before a single load step");
    CHECK_D(o.ir_ohm, 0.0, 0.0, "an estimate out of no steps at all");
    CHECK_D(o.sag_v, 0.0, 0.0, "Sag with no resistance behind it");
    CHECK_D(o.limp_point_v, WF_EST_LIMP_POINT_V, 1e-9,
            "the Limp Point with no resistance measured");

    /* The Monitor sees the BMS before the rider opens the throttle, which is
     * what a bike being switched on does. Without it the first launch would be
     * refused for want of an Anchor, and rightly. */
    feed_soc(&e, 0, IR_SOC);

    uint32_t t = 0;
    double   prev = 0.0;
    double   worst_move = 0.0;      /* largest relative move once settled */
    uint32_t settled_steps = 0;

    for (int i = 0; i < 20; i++) {
        double peak = 90.0 + 6.0 * (double)i;        /* 90 A up to 204 A */
        double open = IR_OPEN_V - 0.05 * (double)i;  /* the Pack emptying */
        uint32_t before = e.ir_steps;
        t = ir_launch(&e, t + SAMPLE_MS, open, peak, 3000u, IR_TRUE_OHM);
        wf_est_get(&e, &o);
        CHECK_U(o.ir_steps - before, 2,
                "accepted steps from one launch and its release");
        if (o.ir_weight > WF_EST_IR_CONFIDENT_SAMPLES) {
            double move = (o.ir_ohm - prev) / prev;
            if (move < 0.0) {
                move = -move;
            }
            if (move > worst_move) {
                worst_move = move;
            }
            settled_steps++;
        }
        prev = o.ir_ohm;
    }

    wf_est_get(&e, &o);
    CHECK(o.ir_valid, "twenty launches produced no Internal Resistance");
    CHECK_U(o.ir_steps, 40, "accepted steps over twenty launches");
    CHECK_U(o.ir_rejected, 0,
            "steps that were sharp enough to try and were refused");
    CHECK_U(o.ir_weight, 40, "the weight the running mean carries");

    /* The Pack that was synthesised, to within what the 0.1 V quantisation can
     * do to a ratio. */
    CHECK_D(o.ir_ohm, IR_TRUE_OHM, 0.005,
            "the Internal Resistance measured off the load steps");
    CHECK(o.ir_ohm >= WF_EST_IR_MIN_OHM && o.ir_ohm <= WF_EST_IR_MAX_OHM,
          "the estimate landed outside its own plausibility bound");

    /* Stable, which is the criterion in the ticket: not wandering with each
     * step. Past the confidence point a whole launch - two steps - may move it
     * by well under a percent, because the running mean's weight is what
     * bounds the move and the weight is growing. */
    CHECK(settled_steps > 10,
          "only %u launches landed past the confidence point, so stability is "
          "not being measured", settled_steps);
    CHECK(worst_move < 0.01,
          "a single launch moved the settled estimate by %.3f %%, which is a "
          "figure that wanders", 100.0 * worst_move);

    /* And it is attributed: issue #21 compares readings over months, and a
     * resistance with no State of Charge against it is not comparable. */
    CHECK_D(o.ir_soc_pct, IR_SOC, 1e-4,
            "the State of Charge the last accepted step was recorded at");
}

/* The rejections, one rule at a time. Every one of them has to leave the
 * estimate untouched rather than fold a weakened sample in - which is the
 * difference between a mean of measurements and a mean of noise.
 *
 * The two counters tell the two kinds of refusal apart. A pair that never beat
 * WF_EST_IR_MIN_DI_A was not a load step and is not counted at all; a pair that
 * was sharp enough to try and failed a later rule is counted, so a Pack that
 * launches hard and produces nothing but rejections can be diagnosed rather
 * than guessed at. */
static void test_bad_load_steps_are_rejected(void)
{
    wf_est_out_t o;

    /* 1. Too small to beat the quantisation. 20 A steps, just under the
     *    threshold, repeated twenty times - a mean of these would be a mean of
     *    rounding error. Not even counted as a refusal: it is not a step. */
    wf_est_t small;
    wf_est_init(&small, NULL);
    uint32_t t = 0;
    for (int i = 0; i < 20; i++) {
        t = ir_launch(&small, t + SAMPLE_MS, IR_OPEN_V,
                      IR_REST_A + WF_EST_IR_MIN_DI_A - 1.0, 1000u, IR_TRUE_OHM);
    }
    wf_est_get(&small, &o);
    CHECK(!o.ir_valid, "a %.1f A step, under the %.1f A threshold, was measured",
          WF_EST_IR_MIN_DI_A - 1.0, WF_EST_IR_MIN_DI_A);
    CHECK_U(o.ir_steps, 0, "steps accepted below the threshold");
    CHECK_U(o.ir_rejected, 0, "sub-threshold pairs counted as refused steps");

    /* And a hair over it is measured, which is what makes the line above a
     * threshold rather than a wall. */
    wf_est_t big;
    wf_est_init(&big, NULL);
    ir_launch(&big, 0, IR_OPEN_V, IR_REST_A + WF_EST_IR_MIN_DI_A + 1.0, 1000u,
              IR_TRUE_OHM);
    wf_est_get(&big, &o);
    CHECK(o.ir_valid, "a %.1f A step, over the %.1f A threshold, was refused",
          WF_EST_IR_MIN_DI_A + 1.0, WF_EST_IR_MIN_DI_A);

    /* 2. A step during a BMS gap. The Controller is launching hard and the
     *    Anchor has gone stale, so there is no State of Charge to record the
     *    reading against - and an Internal Resistance that cannot be compared
     *    with next month's is not a State of Health signal. Refused, and
     *    counted, because these really were load steps. */
    wf_est_t gap;
    wf_est_init(&gap, NULL);
    feed_soc(&gap, 0, IR_SOC);
    t = 60000u;   /* a minute later: the Anchor is long stale */
    for (int i = 0; i < 5; i++) {
        t = ir_launch_no_bms(&gap, t + SAMPLE_MS, IR_OPEN_V, 150.0, 1000u,
                             IR_TRUE_OHM);
    }
    wf_est_get(&gap, &o);
    CHECK(!o.anchor_fresh, "the Anchor was fresh, so this is not a BMS gap");
    CHECK(!o.ir_valid, "a load step during a BMS gap was measured anyway");
    CHECK_U(o.ir_steps, 0, "steps accepted while the BMS was silent");
    CHECK_U(o.ir_rejected, 10, "load steps refused for want of an Anchor");

    /* 3. Not a Pack. Two ohms would be a Sag of hundreds of volts at a launch
     *    current and a tenth of a milliohm would be superconducting; both are
     *    noise, a decode error or a stationary transient, and neither is
     *    averaged in. */
    wf_est_t stiff, soft;
    wf_est_init(&stiff, NULL);
    wf_est_init(&soft, NULL);
    uint32_t ts = 0, tf = 0;
    for (int i = 0; i < 5; i++) {
        ts = ir_launch(&stiff, ts + SAMPLE_MS, IR_OPEN_V, 150.0, 1000u, 0.0001);
        tf = ir_launch(&soft, tf + SAMPLE_MS, 200.0, 60.0, 1000u, 2.0);
    }
    wf_est_get(&stiff, &o);
    CHECK(!o.ir_valid, "a Pack of 0.1 mOhm was believed");
    CHECK_U(o.ir_steps, 0, "steps accepted from an implausibly stiff Pack");
    CHECK(o.ir_rejected > 0, "the implausibly stiff steps were not counted");
    wf_est_get(&soft, &o);
    CHECK(!o.ir_valid, "a Pack of 2 Ohm was believed");
    CHECK_U(o.ir_steps, 0, "steps accepted from an implausibly soft Pack");

    /* 4. The voltage moved the wrong way - it rose as the current rose, which
     *    no Pack does. That is a negative ratio, and negative is below the
     *    plausibility floor, so no separate sign rule is needed and none
     *    exists. */
    wf_est_t backwards;
    wf_est_init(&backwards, NULL);
    t = 0;
    for (int i = 0; i < 5; i++) {
        t = ir_launch(&backwards, t + SAMPLE_MS, IR_OPEN_V, 150.0, 1000u,
                      -IR_TRUE_OHM);
    }
    wf_est_get(&backwards, &o);
    CHECK(!o.ir_valid, "a Pack whose voltage rises under load was believed");
    CHECK(o.ir_rejected > 0, "the backwards steps were not counted as refused");

    /* 5. A pair that straddles a dropped frame. The current really did change
     *    by 145 A between these two samples, but a second apart there is no
     *    telling a step from a ramp, and a ramp measured as a step is a
     *    resistance measured against the wrong voltage. Refused before the
     *    threshold is even considered, so it is not counted either. */
    wf_est_t dropped;
    wf_est_init(&dropped, NULL);
    feed_soc(&dropped, 0, IR_SOC);
    feed_pack(&dropped, 0, IR_OPEN_V, IR_REST_A, IR_TRUE_OHM);
    feed_pack(&dropped, WF_EST_IR_MAX_DT_MS + SAMPLE_MS, IR_OPEN_V, 150.0,
              IR_TRUE_OHM);
    wf_est_get(&dropped, &o);
    CHECK(!o.ir_valid, "a step across a dropped frame was measured");
    CHECK_U(o.ir_steps, 0, "steps accepted across a gap in the stream");
    CHECK_U(o.ir_rejected, 0, "a pair too far apart counted as a refused step");

    /* 6. A ramp, which is the cost of a 5.2 Hz stream written down as a test.
     *    The current climbs from rest to 200 A over four seconds - brisk
     *    riding, 48 A per second - and no pair of consecutive samples moves by
     *    WF_EST_IR_MIN_DI_A, so the whole acceleration contributes nothing.
     *    Only a launch sharper than ~52 A/s is measurable at this rate, and
     *    pretending otherwise would mean averaging in ratios the quantisation
     *    has swamped. */
    wf_est_t ramp;
    wf_est_init(&ramp, NULL);
    feed_soc(&ramp, 0, IR_SOC);
    for (uint32_t k = 0; k <= 20; k++) {
        double a = IR_REST_A + 9.75 * (double)k;
        feed_pack(&ramp, k * SAMPLE_MS, IR_OPEN_V, a, IR_TRUE_OHM);
    }
    wf_est_get(&ramp, &o);
    CHECK(!o.ir_valid, "a ramp was measured as a load step");
    CHECK_U(o.ir_steps, 0, "steps accepted from a ramp");
}

/* ---- the Limp Point moving with load -------------------------------------
 *
 * The ticket's point: Range stops lying precisely when the rider is riding
 * hardest. Below, a Pack whose resistance is already known - restored from
 * NVS, which is what a Monitor that has ridden before has - so that the Sag
 * model can be driven on its own without twenty launches in front of every
 * assertion.
 */

/* A Monitor that already knows its Pack: an Internal Resistance at full weight
 * and no charge state, so the first BMS answer still acquires. */
static wf_est_persist_t ir_history(double ohm)
{
    wf_est_persist_t h;
    memset(&h, 0, sizeof(h));
    h.version   = WF_EST_PERSIST_VERSION;
    h.ir_ohm    = (float)ohm;
    h.ir_weight = (uint16_t)WF_EST_IR_SAMPLES_MAX;
    return h;
}

/* The model itself, and the direction the whole ticket is about. The Limp
 * Point is a voltage event under load: at 84.0 V of threshold and a Pack that
 * sags, the rider holding a hundred amps meets it with a fifth of the Pack
 * still in it, and gets that back on easing off.
 *
 * Asserted on Remaining Energy rather than on Range, because Remaining Energy
 * is the half of the quotient Sag moves and there is no Consumption in it to
 * confuse the direction. The recovery is the assertion that could not be
 * faked: energy drawn is monotone, so a figure that rises when the rider eases
 * off can only be the Limp Point coming back down. */
static void test_sag_moves_the_limp_point_with_load(void)
{
    wf_est_persist_t history = ir_history(IR_TRUE_OHM);
    wf_est_t e;
    wf_est_init(&e, &history);
    feed_soc(&e, 0, IR_SOC);

    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.ir_valid, "the restored Internal Resistance was not believed");
    CHECK_D(o.ir_ohm, IR_TRUE_OHM, 1e-6, "the restored Internal Resistance");

    /* A minute at rest. The load average is the hotel load, so the Limp Point
     * has moved by a quarter of a volt and Remaining Energy is a few tens of
     * watt-hours under what a fixed 84.0 V would have claimed. */
    uint32_t t = 0;
    for (; t <= 60000u; t += SAMPLE_MS) {
        feed_pack(&e, t, IR_OPEN_V, IR_REST_A, IR_TRUE_OHM);
    }
    wf_est_get(&e, &o);
    double rest_wh   = o.remaining_wh;
    double rest_limp = o.limp_point_v;
    CHECK_D(o.load_a, IR_REST_A, 0.01, "the load average at rest");
    CHECK_D(o.sag_v, IR_TRUE_OHM * IR_REST_A, 0.01, "Sag at the hotel load");
    CHECK_D(o.limp_point_v, WF_EST_LIMP_POINT_V + IR_TRUE_OHM * IR_REST_A, 0.01,
            "the Limp Point at rest");
    CHECK(rest_wh < wf_est_energy_above_limp_wh(IR_SOC),
          "a sagging Pack showed as much energy as a perfect one");

    /* Twenty seconds of holding 150 A, which is one long uphill pull. */
    double drawn_before = o.used_wh;
    uint32_t hard_end = t + 20000u;
    for (; t <= hard_end; t += SAMPLE_MS) {
        feed_pack(&e, t, IR_OPEN_V, 150.0, IR_TRUE_OHM);
    }
    wf_est_get(&e, &o);
    double hard_wh = o.remaining_wh;
    CHECK(o.load_a > 90.0,
          "twenty seconds at 150 A left the load average at %.1f A", o.load_a);
    CHECK(o.limp_point_v > rest_limp + 4.0,
          "the Limp Point moved from %.2f V to %.2f V under a sustained "
          "150 A, which is not Sag moving it", rest_limp, o.limp_point_v);
    /* Against the estimator's own resistance and not the synthesised one,
     * because the launch into this pull was itself a load step and the running
     * mean has already folded it in - at a sixty-fourth, which is why the two
     * are still the same number to three decimals. */
    CHECK_D(o.ir_ohm, IR_TRUE_OHM, 0.001,
            "the restored estimate after the ride's own steps landed in it");
    CHECK_D(o.limp_point_v, WF_EST_LIMP_POINT_V + o.ir_ohm * o.load_a, 1e-9,
            "the Limp Point against the model it is supposed to be");

    /* And the figure fell by far more than the energy actually drawn, which is
     * the whole point: the watt-hours are still in the Pack and are no longer
     * reachable at this throttle. */
    double drawn = o.used_wh - drawn_before;
    CHECK(drawn > 0.0 && drawn < 100.0,
          "twenty seconds at 150 A drew %.1f Wh, so the comparison below is "
          "not measuring what it claims", drawn);
    CHECK(rest_wh - hard_wh > 4.0 * drawn,
          "Remaining Energy fell %.1f Wh under a sustained load that drew "
          "%.1f Wh - the Limp Point barely moved", rest_wh - hard_wh, drawn);

    /* Easing off. Two minutes back at the hotel load, and the figure rises -
     * which no integration of current can do while current is positive. */
    uint32_t easy_end = t + 120000u;
    for (; t <= easy_end; t += SAMPLE_MS) {
        feed_pack(&e, t, IR_OPEN_V, IR_REST_A, IR_TRUE_OHM);
    }
    wf_est_get(&e, &o);
    CHECK(o.remaining_wh > hard_wh + 100.0,
          "easing off took Remaining Energy from %.1f Wh to %.1f Wh, which is "
          "not a Limp Point coming back down", hard_wh, o.remaining_wh);
    CHECK_D(o.limp_point_v, rest_limp, 0.05,
            "the Limp Point after easing off");
    /* Not all the way back: two minutes at 150 A really did leave the Pack
     * with less in it than it had. */
    CHECK(o.remaining_wh < rest_wh,
          "the Pack came back with more energy than it started with");
}

/* The risk this whole design is arranged around: a Sag-corrected Limp Point
 * puts a fast, noisy quantity into Range's numerator, which is exactly where
 * jitter would come from. #16 bought Range's steadiness by giving it no filter
 * of its own and putting the smoothing in Consumption's buckets; the answer
 * here is the same shape, one level down - the Limp Point moves with an
 * average of the load and never with the last frame.
 *
 * So both halves have to be asserted, and they are the two halves of the
 * acceptance criterion: it must move for a load the rider is actually holding,
 * and it must not twitch on one frame of it.
 *
 * The measurement is a twin: the same stream into a Monitor that knows its
 * Pack and one that does not, with the difference between their Ranges being
 * the Sag correction and nothing else. Consumption moves under a hard pull as
 * well - genuinely, and identically in both - so comparing a Range against its
 * own past would be measuring both effects at once. */
static void test_sag_moves_range_with_sustained_load_only(void)
{
    const double kmh = 36.0, town = amps_for(30.0, kmh), hard = 150.0;

    wf_est_persist_t history = ir_history(IR_TRUE_OHM);
    wf_est_t sagging, stiff;
    wf_est_init(&sagging, &history);
    wf_est_init(&stiff, NULL);
    feed_soc(&sagging, 0, RANGE_SOC);
    feed_soc(&stiff, 0, RANGE_SOC);

    /* Two kilometres of town riding to fill both windows. */
    uint32_t t = ride(&sagging, 0, 2.0 * WF_EST_CONS_WINDOW_M, kmh, town);
    ride(&stiff, 0, 2.0 * WF_EST_CONS_WINDOW_M, kmh, town);

    wf_est_out_t a, b;
    wf_est_get(&sagging, &a);
    wf_est_get(&stiff, &b);
    CHECK(a.range_valid && b.range_valid, "neither twin produced a Range");
    CHECK(a.consumption_windowed && b.consumption_windowed,
          "two kilometres did not fill the window");
    double gap0 = b.range_km - a.range_km;
    CHECK(gap0 > 1.0,
          "a Pack that sags half a volt at town load cost only %.2f km of "
          "Range, so the correction is not being applied", gap0);

    /* One frame of 150 A - a throttle stab, or a pothole in the current
     * reading. The load average moves by dt/TAU of the step, about a
     * hundredth, and the Range with it. This is the number that says the
     * figure does not twitch. */
    double before = a.range_km;
    t += SAMPLE_MS;
    feed_ride(&sagging, t, kmh, hard);
    feed_ride(&stiff, t, kmh, hard);
    wf_est_get(&sagging, &a);
    wf_est_get(&stiff, &b);
    double twitch = gap0 - (b.range_km - a.range_km);
    if (twitch < 0.0) {
        twitch = -twitch;
    }
    CHECK(twitch < 0.4,
          "one frame at %.0f A moved the Sag correction by %.3f km", hard,
          twitch);
    CHECK(before - a.range_km < 0.5,
          "one frame at %.0f A took %.3f km off the Range", hard,
          before - a.range_km);

    /* Fifteen seconds of holding it, which is a slip road. Same current, and
     * now the figure moves - by ten times what the single frame did. */
    for (int i = 0; i < 75; i++) {
        t += SAMPLE_MS;
        feed_ride(&sagging, t, kmh, hard);
        feed_ride(&stiff, t, kmh, hard);
    }
    wf_est_get(&sagging, &a);
    wf_est_get(&stiff, &b);
    double gap1 = b.range_km - a.range_km;
    CHECK(gap1 > gap0 + 2.0,
          "fifteen seconds at %.0f A moved the Sag correction from %.2f km to "
          "%.2f km, which is not a reaction to sustained load", hard, gap0,
          gap1);
    CHECK(gap1 - gap0 > 10.0 * twitch,
          "a sustained pull moved Range by %.2f km and one frame of the same "
          "current by %.3f km - the two are not far enough apart for the "
          "figure to be following riding rather than frames",
          gap1 - gap0, twitch);
    CHECK(a.range_km < before,
          "Range did not fall under a sustained hard pull");

    /* And easing off gives it back. A minute of town riding, and the
     * correction is within half a kilometre of where it was. */
    for (int i = 0; i < 300; i++) {
        t += SAMPLE_MS;
        feed_ride(&sagging, t, kmh, town);
        feed_ride(&stiff, t, kmh, town);
    }
    wf_est_get(&sagging, &a);
    wf_est_get(&stiff, &b);
    double gap2 = b.range_km - a.range_km;
    CHECK(gap2 < gap0 + 0.5,
          "a minute of easing off left the Sag correction at %.2f km against "
          "the %.2f km it costs at town load", gap2, gap0);
}

/* #16's criterion, re-run with the Sag model switched on: under steady riding
 * Range is monotone non-increasing, sampled at every frame, on the raw double,
 * and it is still exactly zero rises rather than a tolerance.
 *
 * That is not luck. Under a steady load the load average is a constant - the
 * first power sample acquires it and every later one pulls it toward itself by
 * zero - so the Sag reserve is a constant too, and a constant subtracted from
 * a falling numerator falls. The Sag model can only move the figure when the
 * riding does, which is the whole of what was wanted from it. */
static void test_range_is_monotone_with_a_sagging_pack(void)
{
    const double cost = 30.0, kmh = 36.0;

    wf_est_persist_t history = ir_history(IR_TRUE_OHM);
    wf_est_t e, stiff;
    wf_est_init(&e, &history);
    wf_est_init(&stiff, NULL);
    feed_soc(&e, 0, RANGE_SOC);
    feed_soc(&stiff, 0, RANGE_SOC);

    range_watch_t w;
    memset(&w, 0, sizeof(w));
    ride_watched(&e, &w, 0, 10.0 * WF_EST_CONS_WINDOW_M, kmh,
                 amps_for(cost, kmh), 0.0);
    ride(&stiff, 0, 10.0 * WF_EST_CONS_WINDOW_M, kmh, amps_for(cost, kmh));

    CHECK(w.samples > 4000, "only %ld samples to assert monotonicity on",
          w.samples);
    CHECK(w.up_max == 0.0,
          "Range rose by %.9f km during a steady ride on a sagging Pack",
          w.up_max);
    CHECK_D(w.km_up_max, 0.0, 0.0,
            "the kilometre figure ticked up during a steady ride");

    /* And the Sag correction really was on, so the zero above is a property of
     * the model and not of a model that did nothing. */
    wf_est_out_t a, b;
    wf_est_get(&e, &a);
    wf_est_get(&stiff, &b);
    CHECK(a.sag_v > 0.4, "the sagging twin showed %.3f V of Sag", a.sag_v);
    CHECK(b.range_km - a.range_km > 1.0,
          "the two twins ended %.3f km apart, so the Sag model was not "
          "applied", b.range_km - a.range_km);

    /* And the same ten kilometres with the instrument's own noise on it, which
     * is test_range_does_not_jitter_up_on_a_noisy_current() re-run over a Pack
     * that sags. Half an LSB of line current dithered onto every sample now
     * reaches the figure twice - through Consumption, as before, and through
     * the load average behind the Limp Point - so this is the assertion that
     * the second path did not undo what #16 bought.
     *
     * It cannot, and the reason is worth writing down: the load average moves
     * by dt/TAU of each sample's error, about a hundredth of it, so the noise
     * on the Sag reserve is a hundredth of the noise on the current, which is
     * an order of magnitude below what a bucket of road takes off the Range in
     * the same instant. */
    wf_est_t noisy;
    wf_est_init(&noisy, &history);
    feed_soc(&noisy, 0, RANGE_SOC);
    memset(&w, 0, sizeof(w));
    noise_state = 12345u;
    ride_watched(&noisy, &w, 0, 10.0 * WF_EST_CONS_WINDOW_M, kmh,
                 amps_for(cost, kmh), 0.5 / WF_CTRL_CURRENT_LSB_PER_A);

    CHECK(w.up_max < WF_EST_CONS_BUCKET_M / 1000.0,
          "on a sagging Pack, Range rose by %.6f km on noise alone, more than "
          "the %.3f km a bucket of road takes off it", w.up_max,
          WF_EST_CONS_BUCKET_M / 1000.0);
    CHECK(w.km_up_max <= 1.0,
          "the displayed Range jumped up by %.0f km on noise", w.km_up_max);
    CHECK(w.km_ups <= 5,
          "the displayed Range ticked up %ld times over ten noisy kilometres "
          "on a sagging Pack, against the three the rounding accounts for",
          w.km_ups);
}

/* "Persisted, and improves across rides."
 *
 * Two rides with a power cycle between them, and the second one starts from
 * the first one's estimate rather than from nothing: the weight crosses the
 * break, so the second ride's launches refine the number instead of
 * re-measuring it. The Pack is deliberately given a different resistance in
 * the second ride - a colder morning, a year of ageing - so a Monitor that
 * quietly discarded the history would land on the new value outright and be
 * caught. */
static void test_internal_resistance_improves_across_rides(void)
{
    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_soc(&e, 0, IR_SOC);

    uint32_t t = 0;
    for (int i = 0; i < 6; i++) {
        t = ir_launch(&e, t + SAMPLE_MS, IR_OPEN_V, 120.0 + 5.0 * i, 1000u,
                      0.040);
    }
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK_U(o.ir_steps, 12, "steps accepted over the first ride");
    CHECK_D(o.ir_ohm, 0.040, 0.003, "the first ride's Internal Resistance");
    double first = o.ir_ohm;

    /* Out through the bytes and back, which is all a power cycle is. */
    wf_est_persist_t saved, back;
    uint8_t blob[WF_EST_PERSIST_BYTES];
    wf_est_save(&e, &saved);
    CHECK_D(saved.ir_ohm, first, 1e-6, "the saved Internal Resistance");
    CHECK_U(saved.ir_weight, 12, "the saved weight");
    CHECK(wf_est_persist_encode(&saved, blob, sizeof(blob)), "encode failed");
    CHECK(wf_est_persist_decode(blob, sizeof(blob), &back), "decode failed");
    CHECK_U(back.ir_weight, 12, "the weight through the bytes");

    wf_est_t after;
    wf_est_init(&after, &back);
    wf_est_get(&after, &o);
    CHECK(o.ir_valid, "the estimate did not survive the power cycle");
    CHECK_D(o.ir_ohm, first, 1e-6, "the restored Internal Resistance");
    CHECK_U(o.ir_weight, 12, "the restored weight");
    CHECK_U(o.ir_steps, 0, "a restored estimator claimed steps it did not see");

    /* The second ride, on a Pack that now reads 60 mOhm. */
    feed_soc(&after, 0, IR_SOC);
    t = 0;
    for (int i = 0; i < 6; i++) {
        t = ir_launch(&after, t + SAMPLE_MS, IR_OPEN_V, 120.0 + 5.0 * i, 1000u,
                      0.060);
    }
    wf_est_get(&after, &o);
    CHECK_U(o.ir_weight, 24, "the weight after two rides");
    CHECK(o.ir_ohm > first,
          "the second ride's stiffer Pack did not move the estimate at all");
    CHECK(o.ir_ohm < 0.060 - 0.004,
          "the estimate jumped to %.4f Ohm, the second ride's Pack alone - the "
          "history was discarded rather than improved on", o.ir_ohm);
    /* Twelve steps at 40 mOhm and twelve at 60 mOhm, equally weighted, is a
     * running mean sitting between them. */
    CHECK_D(o.ir_ohm, 0.050, 0.004, "the mean over both rides");
}

/* ---------------------------------------------------- the weakest Cell -----
 *
 * cap0007 is a healthy Pack and it is the only real Pack data this repository
 * holds: 28 Cells, 3 to 7 mV between the lowest and the average, in all 34
 * responses. So it proves one half of the criterion for free - a healthy Pack
 * produces neither a clamp nor a warning - and every other case has to be
 * synthesised, here and in tests/host/replay.c, where the same healthy ride is
 * replayed with one Cell rewritten.
 *
 * The synthesis is a Pack as the BMS reports one: 28 Cells fanned out linearly
 * across `fan_mv`, with the lowest of them pulled a further `weak_mv` down. The
 * two knobs are the two things that have to be told apart -
 *
 *   fan_mv    what a healthy Pack does, and it widens as the Pack empties.
 *   weak_mv   what one failing Cell does, and it does not.
 *
 * - and every test below is some combination of the two.
 */
#define CELL_BASE_MV  3763      /* cap0007's own lowest Cell */

/* One BMS answer carrying a whole Cell block. The average register is the
 * truncated mean and the extreme registers are the array's own extremes, which
 * is what the real BMS reports and what tests/host/replay.c asserts on every
 * Capture - a synthesised Pack that broke those identities would be an
 * impossible one. */
static void feed_cells(wf_est_t *e, uint32_t t_ms, double soc_pct,
                       int base_mv, int fan_mv, int weak_mv)
{
    wf_bms_t b;
    memset(&b, 0, sizeof(b));
    b.soc_pct    = (float)soc_pct;
    b.cell_count = WF_EST_PACK_CELLS;

    long sum = 0;
    unsigned lo = 0xffffu, hi = 0;
    for (int i = 0; i < WF_EST_PACK_CELLS; i++) {
        /* Linear across the fan, so the second-lowest sits just above the
         * lowest and the Pack's own spread is `fan_mv` wide however wide that
         * is - which is the whole point of the low-charge test below. */
        double step = (double)(i * fan_mv) / (double)(WF_EST_PACK_CELLS - 1);
        int mv = base_mv + (int)(step + 0.5);
        if (i == 0) {
            mv -= weak_mv;
        }
        b.cell_mv[i] = (uint16_t)mv;
        sum += mv;
        if ((unsigned)mv < lo) lo = (unsigned)mv;
        if ((unsigned)mv > hi) hi = (unsigned)mv;
    }
    b.avg_cell_mv   = (uint16_t)(sum / WF_EST_PACK_CELLS);
    b.cell_min_mv   = (uint16_t)lo;
    b.cell_max_mv   = (uint16_t)hi;
    b.cell_delta_mv = (uint16_t)(hi - lo);
    wf_est_feed_bms(e, t_ms, &b);
}

/* ride_watched(), with a BMS answer carrying that Cell block every POLL_MS -
 * which is the ~1 Hz the real poll runs at. */
static uint32_t ride_pack(wf_est_t *e, range_watch_t *w, uint32_t t0_ms,
                          double metres, double kmh, double amps,
                          double soc_pct, int fan_mv, int weak_mv)
{
    double per_tick = kmh * (1000.0 / 3600.0) * ((double)SAMPLE_MS / 1000.0);
    long ticks = (long)(metres / per_tick + 0.5);
    uint32_t t = t0_ms;

    feed_cells(e, t, soc_pct, CELL_BASE_MV, fan_mv, weak_mv);
    feed_ride(e, t, kmh, amps);
    if (w != NULL) {
        range_watch(w, e);
    }
    for (long i = 0; i < ticks; i++) {
        t += SAMPLE_MS;
        if (t % POLL_MS < SAMPLE_MS) {
            feed_cells(e, t, soc_pct, CELL_BASE_MV, fan_mv, weak_mv);
        }
        feed_ride(e, t, kmh, amps);
        if (w != NULL) {
            range_watch(w, e);
        }
    }
    return t;
}

/* The ticket's first behaviour: Range takes the lower of the Pack-average
 * estimate and the minimum-Cell estimate.
 *
 * A twin, because the alternative - comparing a Range against its own past -
 * would be measuring the ride as well as the Cell. Two Monitors, the same
 * riding, the same State of Charge, and the only difference between them is
 * one Cell 100 mV under the rest of its Pack. */
static void test_the_weakest_cell_clamps_range(void)
{
    const double kmh = 36.0, cost = 30.0;
    const double amps = amps_for(cost, kmh);

    wf_est_t healthy, weak;
    wf_est_init(&healthy, NULL);
    wf_est_init(&weak, NULL);
    ride_pack(&healthy, NULL, 0, 2.0 * WF_EST_CONS_WINDOW_M, kmh, amps,
              RANGE_SOC, 8, 0);
    ride_pack(&weak, NULL, 0, 2.0 * WF_EST_CONS_WINDOW_M, kmh, amps,
              RANGE_SOC, 8, 100);

    wf_est_out_t h, k;
    wf_est_get(&healthy, &h);
    wf_est_get(&weak, &k);

    CHECK(h.range_valid && k.range_valid, "neither twin produced a Range");
    CHECK(h.cell_valid && k.cell_valid,
          "the Cell block never reached the estimator");
    CHECK_U(h.cell_rejected, 0, "Cell blocks the healthy twin refused");
    CHECK_U(k.cell_rejected, 0, "Cell blocks the weak twin refused");

    /* The healthy Pack pays nothing at all, and it is exactly nothing rather
     * than nearly nothing: cap0007's own imbalance is inside the line the Pack
     * model was fitted to, so charging for it would be counting it twice. */
    CHECK(!h.cell_clamped, "a healthy Pack's 4 mV of imbalance clamped Range");
    CHECK(!h.cell_diverged, "a healthy Pack raised the divergence warning");
    CHECK(h.cell_reserve_wh == 0.0, "a healthy Pack gave up %.6f Wh",
          h.cell_reserve_wh);
    CHECK(h.remaining_wh == h.remaining_pack_wh,
          "a healthy Pack's Range is not the Pack-average estimate: %.6f Wh "
          "against %.6f Wh", h.remaining_wh, h.remaining_pack_wh);

    /* The weak one clamps, and says so. */
    CHECK(k.cell_clamped, "a Cell 100 mV under its Pack did not clamp Range");
    CHECK(k.cell_diverged, "a Cell 100 mV under its Pack raised no warning");
    CHECK(k.cell_reserve_wh > 200.0,
          "a Cell 100 mV under its Pack cost only %.1f Wh", k.cell_reserve_wh);

    /* "Takes the lower of the two", as arithmetic rather than as prose: the
     * clamped figure is the Pack-average one less the reserve, exactly. */
    CHECK_D(k.remaining_wh, k.remaining_pack_wh - k.cell_reserve_wh, 1e-9,
            "the clamped figure against the Pack-average one less the reserve");
    CHECK(k.remaining_wh < k.remaining_pack_wh,
          "the clamp did not lower anything");

    /* And it is the model and not a fudge: the reserve is the band the spread
     * implies, run through the same function Sag's reserve goes through. */
    CHECK_D(k.cell_band_v, wf_est_cell_band_v(k.cell_spread_v), 1e-12,
            "the Cell band against the spread it came from");
    CHECK_D(k.cell_reserve_wh, wf_est_sag_reserve_wh(k.cell_band_v), 1e-9,
            "the reserve against the band it came from");

    /* What the rider sees. Both twins are riding identically, so the whole of
     * the difference in Range is the weakest Cell. */
    CHECK(k.range_km < h.range_km * 0.95,
          "the weak twin ended at %.2f km against the healthy twin's %.2f km, "
          "which is not a clamp", k.range_km, h.range_km);
    CHECK_D(k.range_km * k.consumption_wh_per_km, k.remaining_wh, 1e-6,
            "Range times Consumption against the clamped Remaining Energy");
}

/* The hard half of the ticket, and the one a fixed millivolt threshold gets
 * wrong: divergence widens on its own as the Pack empties.
 *
 * Three Packs at 15 % State of Charge, which is a Pack near the bottom of its
 * useful range:
 *
 *   healthy and fanned out   28 Cells spread over 60 mV, which is what a good
 *                            Pack looks like down there. The clamp binds -
 *                            correctly, the lowest Cell really will end the
 *                            ride first - and the warning does not.
 *   one failing Cell         the same 60 mV of total spread, but it is one
 *                            Cell that has left the other 27 behind. Same
 *                            millivolts, opposite verdict.
 *   healthy and tight        the same Pack at 66.7 %, where it has not fanned
 *                            out yet, as the control.
 *
 * The first two are the whole criterion. A threshold in millivolts cannot
 * separate them, because they have the same millivolts. */
static void test_a_healthy_pack_at_low_charge_does_not_warn(void)
{
    const double low = 15.0;

    wf_est_t fanned, failing, tight;
    wf_est_init(&fanned, NULL);
    wf_est_init(&failing, NULL);
    wf_est_init(&tight, NULL);
    /* Six seconds of answers each, so the running means have settled. */
    for (uint32_t t = 0; t <= 6000u; t += POLL_MS) {
        feed_cells(&fanned, t, low, CELL_BASE_MV, 60, 0);
        feed_cells(&failing, t, low, CELL_BASE_MV, 8, 55);
        feed_cells(&tight, t, RANGE_SOC, CELL_BASE_MV, 8, 0);
    }

    wf_est_out_t f, x, c;
    wf_est_get(&fanned, &f);
    wf_est_get(&failing, &x);
    wf_est_get(&tight, &c);

    /* The two low-charge Packs have to be genuinely comparable in millivolts,
     * or this test is not measuring the thing it claims. */
    CHECK(f.cell_spread_v > 0.02 && x.cell_spread_v > 0.02,
          "the two Packs spread %.1f mV and %.1f mV, too little for either to "
          "be near a threshold", f.cell_spread_v * 1000.0,
          x.cell_spread_v * 1000.0);
    CHECK(x.cell_spread_v < 2.5 * f.cell_spread_v,
          "the failing Pack spreads %.1f mV against the healthy one's %.1f mV "
          "- far enough apart that a millivolt threshold would have separated "
          "them, so this is not the case the warning is for",
          x.cell_spread_v * 1000.0, f.cell_spread_v * 1000.0);

    /* The healthy Pack at low charge: clamped, because its lowest Cell really
     * does end the ride before its average does, and not warned about. */
    CHECK(f.cell_clamped,
          "a Pack fanned out over 60 mV did not clamp Range at all");
    CHECK(!f.cell_diverged,
          "a healthy Pack at %.0f %% raised the divergence warning - which is "
          "the warning firing on every ride and being ignored", low);

    /* The same millivolts in one Cell, and it is the warning. */
    CHECK(x.cell_clamped, "a failing Cell did not clamp Range");
    CHECK(x.cell_diverged,
          "a Cell that has left the other 27 behind raised no warning");

    /* And the control: the same healthy Pack higher up, where it has not
     * fanned out, does neither. */
    CHECK(!c.cell_clamped && !c.cell_diverged,
          "a tight healthy Pack at %.1f %% clamped or warned", RANGE_SOC);
}

/* "A missing or implausible per-Cell reading must not clamp Range to zero."
 *
 * Five ways for the Cell registers to be worthless, and in every one of them
 * Range has to degrade to the unclamped Pack-average estimate. The assertion
 * is the same shape as the Consumption guards': no clamp happened at all,
 * rather than a clamp that happened and produced something small. */
static void test_bad_cell_readings_do_not_clamp_range(void)
{
    const double kmh = 36.0, amps = amps_for(30.0, kmh);

    /* 1. A BMS that answers with a State of Charge and nothing else - which is
     *    every synthesised stream in this file, and is also a zero-filled Cell
     *    array. No Cell block ever arrived, so there is no clamp and no
     *    invented one. */
    wf_est_t none;
    wf_est_init(&none, NULL);
    feed_soc(&none, 0, RANGE_SOC);
    ride(&none, 0, 2.0 * WF_EST_CONS_WINDOW_M, kmh, amps);

    wf_est_out_t o;
    wf_est_get(&none, &o);
    CHECK(o.range_valid && o.range_km > 50.0,
          "the unclamped ride produced %.2f km, so this test has nothing to "
          "compare against", o.range_km);
    CHECK(!o.cell_valid, "a Cell reading appeared out of a zero-filled array");
    CHECK(!o.cell_clamped && !o.cell_diverged, "a clamp with no Cell data");
    CHECK(o.cell_reserve_wh == 0.0, "%.6f Wh given up for no Cell data",
          o.cell_reserve_wh);
    CHECK(o.remaining_wh == o.remaining_pack_wh,
          "Range moved without a Cell reading behind it");
    double unclamped_km = o.range_km;

    /* 2-4. Three responses that are not a 28-Cell lithium Pack. Each is
     *      refused whole and counted, and none of them clamps anything. */
    wf_est_t bad;
    wf_est_init(&bad, NULL);
    feed_soc(&bad, 0, RANGE_SOC);

    wf_bms_t b;
    /* A zero-filled array with the right count: 0 mV is not a lithium Cell. */
    memset(&b, 0, sizeof(b));
    b.soc_pct    = (float)RANGE_SOC;
    b.cell_count = WF_EST_PACK_CELLS;
    wf_est_feed_bms(&bad, 0, &b);
    /* A 6 V "Cell", which no lithium chemistry does. */
    memset(&b, 0, sizeof(b));
    b.soc_pct    = (float)RANGE_SOC;
    b.cell_count = WF_EST_PACK_CELLS;
    for (int i = 0; i < WF_EST_PACK_CELLS; i++) {
        b.cell_mv[i] = 3763;
    }
    b.cell_mv[7]    = 6000;
    b.avg_cell_mv   = 3843;
    wf_est_feed_bms(&bad, 1000, &b);
    /* A Pack of 27 Cells: not the Pack this model is of. */
    memset(&b, 0, sizeof(b));
    b.soc_pct    = (float)RANGE_SOC;
    b.cell_count = WF_EST_PACK_CELLS - 1;
    for (int i = 0; i < WF_EST_PACK_CELLS; i++) {
        b.cell_mv[i] = 3763;
    }
    b.avg_cell_mv = 3763;
    wf_est_feed_bms(&bad, 2000, &b);

    wf_est_get(&bad, &o);
    /* Four and not three: the feed_soc() that opened this estimator is a BMS
     * answer whose Cell array is zero-filled and whose count is zero, which is
     * the first way of all to not be a 28-Cell Pack. Every synthesised stream
     * in this file is that response, which is why none of them clamps. */
    CHECK_U(o.cell_rejected, 4, "Cell blocks refused");
    CHECK_U(o.cell_samples, 0, "Cell blocks believed");
    CHECK(!o.cell_valid && !o.cell_clamped, "an implausible Pack clamped Range");

    ride(&bad, 3000, 2.0 * WF_EST_CONS_WINDOW_M, kmh, amps);
    wf_est_get(&bad, &o);
    CHECK(o.range_valid, "three refused responses left the rider with no Range");
    CHECK(o.range_km > 0.0, "three refused responses stranded Range at zero");
    /* The proof that no clamp happened is the identity and not the kilometres:
     * the two twins were Anchored at different instants, so their Ranges differ
     * in the last decimal for reasons that have nothing to do with a Cell. */
    CHECK(o.remaining_wh == o.remaining_pack_wh,
          "the refused responses moved Remaining Energy off the Pack-average "
          "estimate: %.9f Wh against %.9f Wh", o.remaining_wh,
          o.remaining_pack_wh);
    CHECK_D(o.range_km, unclamped_km, 0.01,
            "the refused responses moved Range off the unclamped estimate");

    /* 5. A Cell half a volt under its Pack, which is where a dying Cell and a
     *    slipped register map stop being distinguishable. Refused, and the
     *    good reading before it stands - which is the point: the rider keeps
     *    the clamp they had rather than losing it or being taken to zero. */
    wf_est_t drop;
    wf_est_init(&drop, NULL);
    for (uint32_t t = 0; t <= 4000u; t += POLL_MS) {
        feed_cells(&drop, t, RANGE_SOC, CELL_BASE_MV, 8, 60);
    }
    wf_est_get(&drop, &o);
    CHECK(o.cell_clamped, "the 60 mV Cell did not clamp, so the reading this "
                          "test needs to survive was never taken");
    double held_reserve = o.cell_reserve_wh;
    uint16_t held_min   = o.cell_min_mv;

    feed_cells(&drop, 5000u, RANGE_SOC, CELL_BASE_MV, 8, 900);
    wf_est_get(&drop, &o);
    CHECK_U(o.cell_rejected, 1, "responses refused for an impossible spread");
    CHECK_D(o.cell_reserve_wh, held_reserve, 1e-12,
            "an impossible spread moved the reserve");
    CHECK_U(o.cell_min_mv, held_min, "the lowest Cell the estimator believes");
    CHECK(o.remaining_wh > 0.0,
          "a Cell 900 mV low took Remaining Energy to zero, which is the "
          "rider stranded on a decode fault");

    /* And a dropped response is not even a rejection: imbalance is a fact
     *  about the Pack, not about the link, so nothing moves. */
    ride(&drop, 6000u, 500.0, kmh, amps);
    wf_est_get(&drop, &o);
    CHECK_U(o.cell_rejected, 1, "riding on with no BMS answer refused a block");
    CHECK_D(o.cell_reserve_wh, held_reserve, 1e-12,
            "a gap in the BMS stream moved the Cell reserve");
}

/* #16's and #17's criterion once more, with a weak Cell in the Pack: under
 * steady riding Range is monotone non-increasing, sampled at every frame, on
 * the raw double, and still exactly zero rises rather than a tolerance.
 *
 * It holds for the same reason the Sag model's does. The spread is a running
 * mean over BMS answers; under a steady Pack the first answer assigns it and
 * every later one pulls it toward itself by nothing, so the reserve is a
 * constant, and a constant subtracted from a falling numerator falls. */
static void test_range_is_monotone_with_a_weak_cell(void)
{
    const double kmh = 36.0, cost = 30.0;

    wf_est_t e;
    wf_est_init(&e, NULL);

    range_watch_t w;
    memset(&w, 0, sizeof(w));
    ride_pack(&e, &w, 0, 10.0 * WF_EST_CONS_WINDOW_M, kmh,
              amps_for(cost, kmh), RANGE_SOC, 8, 100);

    CHECK(w.samples > 4000, "only %ld samples to assert monotonicity on",
          w.samples);
    CHECK(w.up_max == 0.0,
          "Range rose by %.9f km during a steady ride on a Pack with a weak "
          "Cell", w.up_max);
    CHECK_D(w.km_up_max, 0.0, 0.0,
            "the kilometre figure ticked up during a steady ride");

    /* And the clamp really was on, so the zero above is a property of the
     * model and not of a model that did nothing. */
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.cell_clamped && o.cell_diverged,
          "the weak Cell was not clamping over the ten kilometres this "
          "monotonicity was asserted on");
}

/* ------------------------------------------------------------------- main */

/* ============================================ Consumption at a chosen speed
 *
 * The evaluator #19 leaves behind for #20. The archive cannot exercise the
 * fitted half of it - there is no fit, and there will not be one until the
 * calibration ride happens - so the polynomial is driven directly against
 * coefficients chosen here, and the wiring around it is driven against
 * whatever wf_fit.h currently holds. Both halves matter: a branch that only
 * ever runs one way is a branch nobody has tested, and the way it currently
 * runs is the refusal.
 */

static void test_the_fit_polynomial_is_what_it_claims(void)
{
    /* a + c*v^2, with the drag term explicit and no linear term. Numbers of
     * the right order for a light electric motorbike, chosen here so the
     * expected value can be worked out by hand: 18 + 0.0045*3600 = 34.2. */
    CHECK(close_to(wf_est_fit_eval(18.0, 0.0, 0.0045, 60.0), 34.2),
          "a + c*v^2 at 60 km/h gave %.6f, expected 34.2",
          wf_est_fit_eval(18.0, 0.0, 0.0045, 60.0));
    /* At rest the drag term vanishes and the constant is all there is, which
     * is what makes `a` the rolling-and-driveline term rather than a fitting
     * artefact. */
    CHECK(close_to(wf_est_fit_eval(18.0, 0.0, 0.0045, 0.0), 18.0),
          "the constant term is not what the curve reads at zero: %.6f",
          wf_est_fit_eval(18.0, 0.0, 0.0045, 0.0));
    /* Quadratic and not cubic, which is the distinction the whole ticket
     * turns on: doubling the speed quadruples the drag term. Power would
     * have gone up eightfold. */
    double at30 = wf_est_fit_eval(0.0, 0.0, 0.0045, 30.0);
    double at60 = wf_est_fit_eval(0.0, 0.0, 0.0045, 60.0);
    CHECK(close_to(at60, 4.0 * at30),
          "doubling speed changed the drag term by %.3fx, not 4x",
          at30 > 0.0 ? at60 / at30 : 0.0);
    /* The linear term is carried through when there is one, so a fit that
     * justified one is evaluated and not silently dropped. */
    CHECK(close_to(wf_est_fit_eval(1.0, 2.0, 3.0, 10.0), 321.0),
          "a + b*v + c*v^2 gave %.6f, expected 321",
          wf_est_fit_eval(1.0, 2.0, 3.0, 10.0));
}

static void test_the_monitor_produces_nothing_without_a_fit(void)
{
    /* Whatever wf_fit.h holds today, the two flags have to agree with it and
     * the unfitted case has to produce exactly nothing. When the calibration
     * ride lands and the header is regenerated, this test follows the header
     * rather than needing to be rewritten. */
    wf_est_fit_t f;
    wf_est_consumption_at_speed(50.0, &f);

    CHECK(f.fitted == (WF_FIT_FITTED != 0),
          "wf_fit.h says FITTED=%d and the evaluator says %d",
          WF_FIT_FITTED, (int)f.fitted);
    CHECK(f.speed_min_kmh == WF_FIT_SPEED_MIN_KMH &&
          f.speed_max_kmh == WF_FIT_SPEED_MAX_KMH,
          "the supported range did not come from wf_fit.h: %.3f-%.3f km/h",
          f.speed_min_kmh, f.speed_max_kmh);

    if (!f.fitted) {
        /* Exactly zero, not something small: the proof that no polynomial
         * was evaluated rather than that one was and came out low. The same
         * discipline `range_km` follows when there is no Range. */
        CHECK(f.wh_per_km == 0.0,
              "an unfitted archive produced %.6f Wh/km at 50 km/h",
              f.wh_per_km);
        CHECK(f.speed_min_kmh == 0.0 && f.speed_max_kmh == 0.0,
              "an unfitted archive claims to support %.1f-%.1f km/h",
              f.speed_min_kmh, f.speed_max_kmh);
        /* And no speed at all is inside a zero-width range, so a caller that
         * checks the range instead of the flag reaches the same answer. */
        wf_est_consumption_at_speed(0.0, &f);
        CHECK(f.wh_per_km == 0.0, "zero km/h produced %.6f Wh/km",
              f.wh_per_km);
    }
}

static void test_extrapolation_is_flagged_and_not_hidden(void)
{
    /* The flag is a property of the range in wf_fit.h, so this asserts the
     * rule the range implies rather than a number it does not yet hold. With
     * no fit, every speed is outside a zero-width range and nothing is
     * produced at all; with one, inside is clear and outside is flagged but
     * still answered - #20 has to be able to tell a supported suggestion from
     * an unsupported one, which needs both. */
    wf_est_fit_t lo, mid, hi;
    double span = WF_FIT_SPEED_MAX_KMH - WF_FIT_SPEED_MIN_KMH;

    wf_est_consumption_at_speed(WF_FIT_SPEED_MIN_KMH - 1.0, &lo);
    wf_est_consumption_at_speed(WF_FIT_SPEED_MIN_KMH + span / 2.0, &mid);
    wf_est_consumption_at_speed(WF_FIT_SPEED_MAX_KMH + 1.0, &hi);

    CHECK(lo.extrapolated == (WF_FIT_FITTED != 0),
          "a km/h below the fitted range was not flagged");
    CHECK(hi.extrapolated == (WF_FIT_FITTED != 0),
          "a km/h above the fitted range was not flagged");
    if (WF_FIT_FITTED && span > 0.0) {
        CHECK(!mid.extrapolated,
              "the middle of the fitted range was flagged as extrapolation");
        CHECK(mid.wh_per_km != 0.0,
              "a supported speed produced no Consumption at all");
        CHECK(hi.wh_per_km != 0.0,
              "an extrapolated speed was refused outright - it is meant to be "
              "flagged, not withheld");
    }

    /* A speed no bike reaches is outside every range this fit could ever
     * hold, so it is flagged whatever the header says next year. The
     * comparison is written the way round that rejects it - and rejects a
     * NaN with it, since a NaN compares false against both bounds and the
     * honest answer to "is this speed supported" for a speed that is not a
     * number is no. */
    wf_est_fit_t absurd;
    wf_est_consumption_at_speed(1e9, &absurd);
    CHECK(absurd.extrapolated == (WF_FIT_FITTED != 0),
          "a million km/h was not flagged as extrapolation");
}

/* ======================================================== the advice, issue #20
 *
 * "How much further would I get at 45?" - the one thing on this screen that
 * answers a question the rider did not already have the numbers for.
 *
 * Every test here is in two halves and the split is the whole of how this
 * cascade has handled a fixture that cannot exercise the physics.
 *
 * The dormancy half runs against the committed constants. WF_FIT_FITTED is 0,
 * so the advice can never appear on this build, and that is asserted rather
 * than assumed - a feature that is meant to be silent and is silent for the
 * wrong reason would look identical from the outside.
 *
 * The behaviour half runs against a curve synthesised here, because there is
 * no fitted one to run against and there will not be until the calibration
 * ride happens. That is the same reason every Range test above is driven by
 * synthesised riding: cap0007 covers 16.7 m and produces no Range either. What
 * is deliberately NOT done is putting invented coefficients into wf_fit.h to
 * make the feature demonstrable on the archive - that is the one failure mode
 * ADR-0005 exists to prevent.
 */

/* A curve the archive does not have, and numbers of the right order for a
 * light electric motorbike: 18 Wh/km of rolling and driveline, and a drag term
 * that has doubled that by 90 km/h. Supported over 20-90 km/h, which is about
 * what a calibration ride with steady-speed holds in it would cover.
 *
 * The arithmetic the tests below lean on, worked out here so the expectations
 * are checkable by hand:
 *
 *   25 km/h  20.81      35 km/h  23.51      45 km/h  27.11      55 km/h  31.61
 *   30 km/h  22.05      40 km/h  25.20      50 km/h  29.25      60 km/h  34.20
 */
static void synth_curve(wf_est_curve_t *c)
{
    memset(c, 0, sizeof(*c));
    c->fitted                 = true;
    c->a_wh_per_km            = 18.0;
    c->c_wh_per_km_per_kmh2   = 0.0045;
    c->speed_min_kmh          = 20.0;
    c->speed_max_kmh          = 90.0;
}

static double curve_wh_per_km(const wf_est_curve_t *c, double kmh)
{
    return wf_est_fit_eval(c->a_wh_per_km, c->b_wh_per_km_per_kmh,
                           c->c_wh_per_km_per_kmh2, kmh);
}

/* A draw that costs what the curve says this speed costs. The synthesised bike
 * and the synthesised fit then describe one machine, so the measured
 * Consumption the Range is built on and the fitted curve the counterfactual is
 * built on cannot silently disagree - which is the state a real Monitor with a
 * real fit would be in on a flat road. */
static double amps_on_curve(const wf_est_curve_t *c, double kmh)
{
    return amps_for(curve_wh_per_km(c, kmh), kmh);
}

/* A Range figure with no advice on it has to be a Range figure and nothing
 * else: the three advice fields are left at exactly zero, which is the proof
 * that no counterfactual was evaluated rather than that one was and came out
 * small. It is also the acceptance criterion "the default screen still shows
 * exactly one Range figure", as arithmetic - main/ui.c draws the row only when
 * `advice_valid`, so a zeroed triple is an absent row. */
static void check_no_second_number(const wf_est_out_t *o, const char *when)
{
    if (o->advice_valid) {
        return;
    }
    CHECK(o->advice_speed_kmh == 0.0 && o->advice_range_km == 0.0 &&
          o->advice_gain_km == 0.0,
          "%s: no advice, and yet %.1f km/h buying %.1f km came out of the "
          "counterfactual that was not supposed to happen", when,
          o->advice_speed_kmh, o->advice_range_km);
}

/* THE DORMANCY, and it is a real assertion and not a placeholder.
 *
 * The archive holds one 47-second parking-lot ride, the fit refused over it,
 * and so the counterfactual has no curve to evaluate. The rule that follows is
 * absolute: no ride, however low its Pack and however fast its rider, may put
 * this row on the screen on this build. */
static void test_the_advice_is_silent_without_a_fit(void)
{
    wf_est_curve_t committed;
    wf_est_curve_default(&committed);
    CHECK(committed.fitted == (WF_FIT_FITTED != 0),
          "wf_fit.h says FITTED=%d and the curve says %d", WF_FIT_FITTED,
          (int)committed.fitted);

    if (!committed.fitted) {
        /* Every speed a rider could be doing, against a Pack as low as it can
         * get, which is the most favourable case the advice will ever see. */
        for (double kmh = 5.0; kmh <= 120.0; kmh += 5.0) {
            wf_est_advice_t a;
            CHECK(!wf_est_advice_pick(&committed, kmh, 40.0, 0.05, &a),
                  "an unfitted archive advised something at %.0f km/h", kmh);
            CHECK(a.speed_kmh == 0.0 && a.range_km == 0.0,
                  "an unfitted archive produced %.1f km/h buying %.1f km",
                  a.speed_kmh, a.range_km);
            CHECK(!wf_est_advice_at(&committed, kmh, 40.0, kmh - 10.0, &a),
                  "an unfitted archive costed a counterfactual at %.0f km/h",
                  kmh - 10.0);
        }
    }

    /* And end to end, on the estimator the Monitor actually runs: a Pack down
     * to a fifth of its usable energy, a rider holding a speed the advice
     * would love to talk about, and twenty minutes of it. */
    wf_est_t e;
    wf_est_init(&e, NULL);
    feed_soc(&e, 0, 29.0);
    wf_est_curve_t c;
    synth_curve(&c);
    for (uint32_t t = SAMPLE_MS; t <= 20u * 60u * 1000u; t += SAMPLE_MS) {
        feed_ride(&e, t, 55.0, amps_on_curve(&c, 55.0));
        wf_est_out_t o;
        wf_est_get(&e, &o);
        if (o.advice_valid) {
            CHECK(false, "the advice appeared at %u ms with no fit behind it",
                  (unsigned)t);
            break;
        }
        check_no_second_number(&o, "an unfitted ride");
    }
    wf_est_out_t o;
    wf_est_get(&e, &o);
    CHECK(o.range_valid, "the dormancy ride produced no Range at all, so it "
                         "was not testing the advice's silence");
    CHECK(o.usable_frac <= WF_EST_ADVICE_LOW_FRAC,
          "the dormancy ride ended at %.3f of a usable Pack, above the %.2f "
          "the advice needs - so it never asked the question",
          o.usable_frac, WF_EST_ADVICE_LOW_FRAC);
}

/* The counterfactual itself, with no policy in it: Range times the ratio of
 * the curve at the two speeds. The ratio is what makes this honest - the
 * rider's own measured Consumption carries the hill they are on, and only the
 * *shape* of the fit is borrowed. */
static void test_the_counterfactual_is_a_ratio(void)
{
    wf_est_curve_t c;
    synth_curve(&c);

    wf_est_advice_t a;
    CHECK(wf_est_advice_at(&c, 55.0, 30.0, 45.0, &a),
          "no counterfactual for 55 -> 45 km/h inside the fitted range");
    double want = 30.0 * (curve_wh_per_km(&c, 55.0) / curve_wh_per_km(&c, 45.0));
    CHECK_D(a.range_km, want, 1e-9, "the Range 45 km/h would buy");
    CHECK_D(a.gain_km, want - 30.0, 1e-9, "the extra distance");
    CHECK_D(a.gain_frac, (want - 30.0) / 30.0, 1e-12, "the gain as a fraction");
    CHECK(a.range_km > 30.0, "slowing down bought less Range, not more");

    /* Scale-free in the Range it is applied to, which is the property that
     * makes the 19 % current scale cancel out of the *fraction* even though it
     * does not cancel out of the kilometres. */
    wf_est_advice_t half;
    CHECK(wf_est_advice_at(&c, 55.0, 15.0, 45.0, &half), "no counterfactual");
    CHECK_D(half.gain_frac, a.gain_frac, 1e-12,
            "the gain fraction moved when only the Range it was applied to did");

    /* Both ends have to be inside the fit. A rider above the fitted range gets
     * nothing, because a counterfactual with an extrapolated numerator
     * over-promises exactly as badly as one with an extrapolated denominator -
     * and over-promising is the failure this feature cannot have. */
    CHECK(!wf_est_advice_at(&c, 100.0, 30.0, 60.0, &a),
          "a cruise of 100 km/h was costed against a fit that stops at 90");
    CHECK(!wf_est_advice_at(&c, 30.0, 30.0, 15.0, &a),
          "a suggestion of 15 km/h was costed against a fit that starts at 20");
    CHECK(a.range_km == 0.0 && a.speed_kmh == 0.0,
          "a refused counterfactual still filled something in");

    /* And no Range to take a fraction of is no advice, whatever the curve. */
    CHECK(!wf_est_advice_at(&c, 55.0, 0.0, 45.0, &a),
          "a counterfactual on a Range of zero");
}

/* THE FIRST CONDITION. On a full Pack there is nothing to decide and the row
 * stays off the screen; the threshold is a fifth of the usable Pack and it is
 * pinned here to the millipoint, because a threshold nobody can see move is a
 * threshold that will be moved by accident. */
static void test_the_advice_is_hidden_on_a_full_pack(void)
{
    wf_est_curve_t c;
    synth_curve(&c);
    wf_est_advice_t a;

    CHECK(!wf_est_advice_pick(&c, 55.0, 30.0, 1.00, &a),
          "the advice spoke to a rider on a full Pack");
    CHECK(!wf_est_advice_pick(&c, 55.0, 30.0, 0.50, &a),
          "the advice spoke to a rider on half a Pack");
    CHECK(!wf_est_advice_pick(&c, 55.0, 30.0,
                              WF_EST_ADVICE_LOW_FRAC + 0.001, &a),
          "the advice spoke a thousandth above its own threshold");
    CHECK(a.speed_kmh == 0.0, "a refused pick still suggested %.1f km/h",
          a.speed_kmh);
    CHECK(wf_est_advice_pick(&c, 55.0, 30.0, WF_EST_ADVICE_LOW_FRAC, &a),
          "the advice stayed silent at exactly its own threshold");

    /* A frac that is not a number is not a low Pack. The comparison is written
     * the way round that says so, so an estimate that has gone wrong is silent
     * rather than loud. */
    double nan_frac = 0.0 / 0.0;
    CHECK(!wf_est_advice_pick(&c, 55.0, 30.0, nan_frac, &a),
          "a NaN Pack fraction was treated as a low Pack");
}

/* THE SECOND CONDITION, and the threshold the criterion asks to be stated:
 * WF_EST_ADVICE_GAIN_FRAC of the Range they have, and WF_EST_ADVICE_GAIN_KM of
 * road. A curve flat enough that backing off buys a few percent says nothing
 * at all, which is the "hidden when it would not change the decision" half. */
static void test_the_advice_is_hidden_when_slowing_buys_nothing(void)
{
    /* Almost all rolling resistance and almost no drag - a heavy bike at town
     * speeds. Slowing from 55 to 35 buys under 3 %. */
    wf_est_curve_t flat;
    memset(&flat, 0, sizeof(flat));
    flat.fitted               = true;
    flat.a_wh_per_km          = 30.0;
    flat.c_wh_per_km_per_kmh2 = 0.0005;
    flat.speed_min_kmh        = 10.0;
    flat.speed_max_kmh        = 120.0;

    wf_est_advice_t a;
    CHECK(!wf_est_advice_pick(&flat, 55.0, 30.0, 0.10, &a),
          "the advice offered %.1f km/h for %.2f km, which is %.1f %% - under "
          "the %.0f %% that changes a decision", a.speed_kmh, a.gain_km,
          100.0 * a.gain_frac, 100.0 * WF_EST_ADVICE_GAIN_FRAC);

    /* The kilometre floor, on its own, and it takes a Range short enough that
     * no rung of the ladder can clear it: a rider with 1.5 km left is offered
     * a third more by the biggest cut on offer, and a third of 1.5 km is half
     * a kilometre. Both rows render "%.0f", so they would show the same number
     * twice with an instruction between them, which reads as a fault.
     *
     * Note what the floor does NOT do, and the difference matters: on a longer
     * Range it makes the ladder walk one rung further down rather than going
     * silent, because a deeper cut buys more road. Silence is only the answer
     * when the whole ladder is under the floor. */
    wf_est_curve_t c;
    synth_curve(&c);
    CHECK(wf_est_advice_at(&c, 55.0, 1.5, 45.0, &a) &&
          a.gain_frac >= WF_EST_ADVICE_GAIN_FRAC,
          "the short-Range case is not clearing the fraction, so it is testing "
          "the wrong floor");
    CHECK(a.gain_km < WF_EST_ADVICE_GAIN_KM,
          "%.2f km is not under the %.1f km floor this case is about",
          a.gain_km, WF_EST_ADVICE_GAIN_KM);
    CHECK(!wf_est_advice_pick(&c, 55.0, 1.5, 0.10, &a),
          "the advice promised %.2f km, under the %.1f km either row can show",
          a.gain_km, WF_EST_ADVICE_GAIN_KM);
    /* The same rider with a longer Range is advised, and advised deeper than
     * the fraction alone would have suggested - which is the floor moving the
     * answer instead of removing it. */
    CHECK(wf_est_advice_pick(&c, 55.0, 4.0, 0.10, &a),
          "no advice at all for a Range the ladder can clear the floor on");
    CHECK_D(a.speed_kmh, 40.0, 0.0,
            "the rung the kilometre floor pushed the suggestion down to");
}

/* THE SUGGESTION HAS TO BE ONE THE RIDER COULD ACTUALLY HOLD, which is four
 * separate refusals wearing one criterion. */
static void test_the_suggested_speed_is_holdable(void)
{
    wf_est_curve_t c;
    synth_curve(&c);
    wf_est_advice_t a;

    /* The ordinary case: 55 km/h on a low Pack. 50 buys 8 % and is refused; 45
     * buys 17 % and is the answer - the least sacrifice that changes the
     * outcome, and a number a speedometer has a mark for. */
    CHECK(wf_est_advice_pick(&c, 55.0, 30.0, 0.15, &a),
          "no advice at 55 km/h on a Pack at 15 %%");
    CHECK_D(a.speed_kmh, 45.0, 0.0, "the suggested speed");
    CHECK(a.gain_frac >= WF_EST_ADVICE_GAIN_FRAC,
          "the suggestion buys %.1f %%, under its own threshold",
          100.0 * a.gain_frac);

    /* Round numbers, whatever the cruise is. A rider drifting at 62.3 km/h is
     * not told to hold 52.3. */
    for (double cruise = 30.0; cruise <= 90.0; cruise += 0.7) {
        if (!wf_est_advice_pick(&c, cruise, 40.0, 0.15, &a)) {
            continue;
        }
        double rungs = a.speed_kmh / WF_EST_ADVICE_STEP_KMH;
        CHECK_D(rungs, (double)(long)(rungs + 0.5), 1e-9,
                "a suggestion that is not a multiple of the ladder step");
        CHECK(a.speed_kmh >= WF_EST_ADVICE_MIN_SPEED_KMH,
              "a suggestion of %.1f km/h, under the %.0f km/h floor",
              a.speed_kmh, WF_EST_ADVICE_MIN_SPEED_KMH);
        CHECK(a.speed_kmh <= cruise - WF_EST_ADVICE_STEP_KMH,
              "a suggestion of %.1f km/h to a rider doing %.1f - not a step",
              a.speed_kmh, cruise);
        CHECK(a.speed_kmh >= cruise * (1.0 - WF_EST_ADVICE_MAX_DROP_FRAC),
              "a suggestion of %.1f km/h to a rider doing %.1f is %.0f %% off",
              a.speed_kmh, cruise, 100.0 * (1.0 - a.speed_kmh / cruise));
        /* And inside the fit, always. This is `extrapolated` doing the job it
         * was put in #19 for. */
        wf_est_fit_t f;
        wf_est_curve_at(&c, a.speed_kmh, &f);
        CHECK(f.fitted && !f.extrapolated,
              "a suggestion of %.1f km/h, outside the %.0f-%.0f km/h the fit "
              "covers", a.speed_kmh, c.speed_min_kmh, c.speed_max_kmh);
    }

    /* Above the fitted range there is nothing honest to say, and the answer is
     * silence rather than an extrapolated quadratic. */
    CHECK(!wf_est_advice_pick(&c, 110.0, 30.0, 0.15, &a),
          "a rider above the fitted range was advised %.1f km/h", a.speed_kmh);

    /* Below the ladder's own floor, likewise: a rider already at 30 km/h has
     * only 25 to give and 25 buys 11 %, so nothing is said. */
    CHECK(!wf_est_advice_pick(&c, 30.0, 30.0, 0.15, &a),
          "a rider at 30 km/h was advised %.1f km/h", a.speed_kmh);

    /* The proportional cap, which is the one that answers "slow to 25 km/h on
     * a motorway". A curve flat enough that a rider doing 100 has to come down
     * to half of it before the gain is worth having - so the advice says
     * nothing, and the thing it would otherwise have said is exactly what the
     * cap exists to refuse. */
    wf_est_curve_t heavy;
    memset(&heavy, 0, sizeof(heavy));
    heavy.fitted               = true;
    heavy.a_wh_per_km          = 45.0;
    heavy.c_wh_per_km_per_kmh2 = 0.001;
    heavy.speed_min_kmh        = 10.0;
    heavy.speed_max_kmh        = 120.0;

    wf_est_advice_t would;
    CHECK(wf_est_advice_at(&heavy, 100.0, 40.0, 50.0, &would) &&
          would.gain_frac >= WF_EST_ADVICE_GAIN_FRAC,
          "50 km/h does not clear the gain threshold here, so this case is "
          "not testing the cap");
    CHECK(50.0 < 100.0 * (1.0 - WF_EST_ADVICE_MAX_DROP_FRAC),
          "50 km/h is not past the cap, so this case tests nothing");
    CHECK(!wf_est_advice_pick(&heavy, 100.0, 40.0, 0.15, &a),
          "a rider doing 100 km/h was told to hold %.1f km/h", a.speed_kmh);
}

/* THE RIDE, and the criterion "the advice appears at the intended point and
 * not before". A Pack starting at 28 % of its usable energy, drained at a
 * steady 55 km/h until it crosses the threshold.
 *
 * The two halves asserted are "not one frame before the Pack is low" and "not
 * long after it" - the second matters as much as the first, because an advice
 * that arrives ten minutes late arrives after the junction the rider needed it
 * at. WF_EST_ADVICE_ARM_MS is ten seconds, which at 55 km/h is 153 m and about
 * five watt-hours: a thousandth of the Pack, which is what the tolerance
 * below is. */
static void test_the_advice_arrives_when_the_pack_is_low(void)
{
    wf_est_curve_t c;
    synth_curve(&c);

    wf_est_t e;
    wf_est_init(&e, NULL);
    wf_est_set_curve(&e, &c);
    feed_soc(&e, 0, 38.0);

    bool   seen = false;
    double frac_at_first = 0.0;
    double range_at_first = 0.0, advice_at_first = 0.0;
    uint32_t t_first = 0;

    for (uint32_t t = SAMPLE_MS; t <= 30u * 60u * 1000u; t += SAMPLE_MS) {
        feed_ride(&e, t, 55.0, amps_on_curve(&c, 55.0));

        wf_est_out_t o;
        wf_est_get(&e, &o);
        if (!seen) {
            /* Not before. The condition is on the Pack, so a screen showing
             * this while the Pack is above the threshold is showing it for a
             * reason nobody wrote down. */
            CHECK(!o.advice_valid || o.usable_frac <= WF_EST_ADVICE_LOW_FRAC,
                  "the advice appeared at %.3f of a usable Pack, above the "
                  "%.2f threshold", o.usable_frac, WF_EST_ADVICE_LOW_FRAC);
        }
        check_no_second_number(&o, "mid-ride");
        if (o.advice_valid && !seen) {
            seen            = true;
            t_first         = t;
            frac_at_first   = o.usable_frac;
            range_at_first  = o.range_km;
            advice_at_first = o.advice_range_km;
        }
        if (o.advice_valid) {
            /* Everything the row says, every frame it says it. */
            CHECK_D(o.advice_speed_kmh, 45.0, 0.0,
                    "the speed the advice suggests at a steady 55 km/h");
            CHECK(o.advice_range_km > o.range_km,
                  "the advice offered %.1f km against a Range of %.1f km",
                  o.advice_range_km, o.range_km);
            CHECK(o.advice_gain_km >= WF_EST_ADVICE_GAIN_KM,
                  "the advice offered %.2f km, under its own floor",
                  o.advice_gain_km);
        }
    }

    CHECK(seen, "a Pack drained past the threshold never produced the advice");
    if (!seen) {
        return;
    }
    CHECK(frac_at_first <= WF_EST_ADVICE_LOW_FRAC,
          "the advice arrived at %.4f of a usable Pack", frac_at_first);
    CHECK(frac_at_first >= WF_EST_ADVICE_LOW_FRAC - 0.005,
          "the advice arrived %.4f of a Pack late - more than the ten seconds "
          "of arming accounts for", WF_EST_ADVICE_LOW_FRAC - frac_at_first);
    CHECK(t_first > WF_EST_ADVICE_ARM_MS,
          "the advice appeared before it could have armed");
    /* And what it said, checked against the curve by hand rather than against
     * the code that produced it. */
    double want = range_at_first *
                  (curve_wh_per_km(&c, 55.0) / curve_wh_per_km(&c, 45.0));
    CHECK_D(advice_at_first, want, 1e-6,
            "the kilometres the advice offered are not the Range times the "
            "curve's own ratio");
}

/* IT MUST NOT BLINK, which is a rider-facing correctness property: a warning
 * that flickers is one that gets ignored, and this one only ever appears when
 * the rider cannot afford to ignore it.
 *
 * The test drives the two mechanisms separately, and measures what they are
 * suppressing rather than asserting a bare "it did not flicker".
 *
 *   Phase 1, eight-second oscillation between 34 and 42 km/h. The raw
 *   condition - the pick applied to this instant's speed, with no averaging
 *   and no hysteresis - toggles on every swing, because 34 km/h has nothing
 *   worth saying and 42 has. The cruise average absorbs it whole.
 *
 *   Phase 2, sixty-second swings between 30 and 45. Now the average genuinely
 *   follows and the advice genuinely comes and goes - which is right, the
 *   rider really has changed what they are doing - and what is asserted is
 *   that no episode is shorter than the dwell. The screen changes, and it
 *   never blinks.
 */
static void test_the_advice_does_not_flicker(void)
{
    wf_est_curve_t c;
    synth_curve(&c);

    wf_est_t e;
    wf_est_init(&e, NULL);
    wf_est_set_curve(&e, &c);
    feed_soc(&e, 0, 29.0);

    bool     shown = false, raw_prev = false, started = false;
    long     raw_flips = 0, flips = 0, phase1_flips = 0;
    uint32_t episode_t0 = 0;
    uint32_t shortest_shown = 0xffffffffu, shortest_hidden = 0xffffffffu;
    const uint32_t phase1_end = 8u * 60u * 1000u;
    const uint32_t ride_end   = 24u * 60u * 1000u;

    for (uint32_t t = SAMPLE_MS; t <= ride_end; t += SAMPLE_MS) {
        double kmh;
        if (t <= phase1_end) {
            kmh = ((t / 4000u) % 2u) ? 42.0 : 34.0;
        } else {
            kmh = ((t / 30000u) % 2u) ? 45.0 : 30.0;
        }
        feed_ride(&e, t, kmh, amps_on_curve(&c, kmh));

        wf_est_out_t o;
        wf_est_get(&e, &o);
        check_no_second_number(&o, "a rider whose speed is moving");
        if (!o.range_valid) {
            continue;
        }

        /* What a build with no averaging and no hysteresis in it would have
         * put on the screen this frame. */
        wf_est_advice_t a;
        bool raw = wf_est_advice_pick(&c, kmh, o.range_km, o.usable_frac, &a);
        if (started && raw != raw_prev) {
            raw_flips++;
        }
        raw_prev = raw;

        if (!started) {
            started    = true;
            shown      = o.advice_valid;
            episode_t0 = t;
            continue;
        }
        if (o.advice_valid == shown) {
            continue;
        }
        uint32_t held = t - episode_t0;
        if (shown) {
            if (held < shortest_shown) {
                shortest_shown = held;
            }
        } else if (held < shortest_hidden) {
            shortest_hidden = held;
        }
        flips++;
        if (t <= phase1_end) {
            phase1_flips++;
        }
        shown      = o.advice_valid;
        episode_t0 = t;
    }

    CHECK(raw_flips > 20,
          "the raw condition only moved %ld times, so this ride is not "
          "driving the thing the hysteresis exists for", raw_flips);
    /* Phase 1: one transition is the advice arriving and staying. Anything
     * more is the screen following the throttle. */
    CHECK(phase1_flips <= 1,
          "the advice changed %ld times while the rider's speed swung either "
          "side of the threshold every eight seconds", phase1_flips);
    CHECK(flips >= 2,
          "the advice never came and went at all over the slow swings, so the "
          "dwell below is not being tested");
    CHECK(shortest_shown >= WF_EST_ADVICE_DWELL_MS,
          "the advice was on the screen for %u ms, under the %u ms dwell",
          (unsigned)shortest_shown, (unsigned)WF_EST_ADVICE_DWELL_MS);
    CHECK(shortest_hidden >= WF_EST_ADVICE_ARM_MS,
          "the advice came back after %u ms, under the %u ms it takes to arm",
          (unsigned)shortest_hidden, (unsigned)WF_EST_ADVICE_ARM_MS);
}

/* THE ARMING IS A TIMER AND HAS TO BE RESETTABLE, which is a property the test
 * above cannot see. Its two phases each hold the flip condition steady for
 * minutes at a time, so an arming timer that never cleared would still produce
 * the same episodes; what that test proves is that the advice does not follow
 * the throttle, not that ten seconds of agreement were ever actually required.
 *
 * This one drives the case that separates them: agreement that keeps being
 * interrupted. A rider on a low Pack works their way up from a speed the
 * ladder has nothing to say about to one it does, with the throttle moving the
 * way a throttle moves, so the cruise average sits on the ladder's own
 * boundary for a couple of minutes and crosses it back and forth. Every one of
 * those agreeing runs is shorter than WF_EST_ADVICE_ARM_MS, and none of them
 * may put the row on the screen.
 *
 * The assertion is made against the condition rather than against a clock: the
 * test recomputes the same pick the estimator makes, from the same figures it
 * is shown, and requires that when the advice finally appears the condition
 * had held without interruption for the full arming time. A timer that
 * saturated instead of resetting - which is what an overflow test applied to
 * the reset does - shows the advice one frame after an interruption, and this
 * fails by 9800 ms. */
static void test_the_advice_arms_only_on_unbroken_agreement(void)
{
    wf_est_curve_t c;
    synth_curve(&c);

    wf_est_t e;
    wf_est_init(&e, NULL);
    wf_est_set_curve(&e, &c);
    feed_soc(&e, 0, 29.0);

    const uint32_t ride_end = 20u * 60u * 1000u;
    bool     seen = false, agreed_prev = false;
    uint32_t run_t0 = 0, last_disagreed = 0;
    long     runs = 0, short_runs = 0;

    for (uint32_t t = SAMPLE_MS; t <= ride_end; t += SAMPLE_MS) {
        /* A base speed climbing 30 to 46 km/h over the ride, with an
         * eight-second +-9 km/h swing on it. The cruise average follows the
         * base and jitters by about a km/h, which is what walks it across the
         * boundary repeatedly instead of once. */
        double base = 30.0 + 16.0 * (double)t / (double)ride_end;
        double kmh  = base + (((t / 4000u) % 2u) ? 9.0 : -9.0);
        /* The power frame first and the motion frame second, so that the
         * snapshot below is the state the advice was decided on rather than
         * one power block past it - the decision is taken at the end of the
         * motion block. */
        feed_power(&e, t, CONS_VOLTS, amps_on_curve(&c, kmh));
        feed_speed(&e, t, kmh);

        wf_est_out_t o;
        wf_est_get(&e, &o);
        check_no_second_number(&o, "a rider on the boundary of the ladder");

        /* The estimator's own question, asked again here from the figures the
         * rider is being shown. While the advice is hidden this is exactly
         * what advice_step() tests. */
        wf_est_advice_t a;
        bool agreed = o.range_valid && o.cruise_valid &&
                      wf_est_advice_pick(&c, o.cruise_kmh, o.range_km,
                                         o.usable_frac, &a);
        if (agreed && !agreed_prev) {
            run_t0 = t;
        }
        if (!agreed) {
            if (agreed_prev) {
                runs++;
                if (t - run_t0 < WF_EST_ADVICE_ARM_MS) {
                    short_runs++;
                }
            }
            last_disagreed = t;
        }
        agreed_prev = agreed;

        if (o.advice_valid) {
            seen = true;
            CHECK(t - last_disagreed >= WF_EST_ADVICE_ARM_MS,
                  "the advice appeared after %u ms of agreement, under the "
                  "%u ms of arming - the timer was not cleared by the %ld "
                  "runs that came before it",
                  (unsigned)(t - last_disagreed),
                  (unsigned)WF_EST_ADVICE_ARM_MS, runs);
            break;
        }
    }

    CHECK(seen, "the advice never appeared, so the arming was never completed "
                "and nothing here was tested");
    /* And the ride really did interrupt the agreement, repeatedly, before it
     * finally held. Without this the test above could pass on a ride that
     * simply agreed once and armed. */
    CHECK(short_runs >= 8,
          "only %ld agreeing runs ended before the arming completed, so this "
          "ride is not driving the reset", short_runs);
}

/* The hysteresis band on the low-Pack condition, driven directly rather than
 * inferred from a ride that never crosses back.
 *
 * The Pack recovers - a long descent, or simply a BMS that has warmed up and
 * revised its State of Charge upward, which is what is synthesised here
 * because the Anchor's pull raises Remaining Energy without touching the
 * Consumption window and so isolates the one condition being tested. The
 * advice has to hold through the whole of WF_EST_ADVICE_LOW_FRAC to
 * _LOW_FRAC_KEEP, and let go past it. A rider sitting exactly on a fifth of a
 * Pack must not be shown a row that comes and goes with the last decimal. */
static void test_the_advice_holds_through_its_own_band(void)
{
    wf_est_curve_t c;
    synth_curve(&c);

    wf_est_t e;
    wf_est_init(&e, NULL);
    wf_est_set_curve(&e, &c);
    feed_soc(&e, 0, 29.0);

    uint32_t t = SAMPLE_MS;
    bool     seen = false;
    for (; t <= 10u * 60u * 1000u; t += SAMPLE_MS) {
        feed_ride(&e, t, 55.0, amps_on_curve(&c, 55.0));
        wf_est_out_t o;
        wf_est_get(&e, &o);
        if (o.advice_valid) {
            seen = true;
            break;
        }
    }
    CHECK(seen, "the advice never appeared, so there is no band to hold");
    if (!seen) {
        return;
    }

    /* The BMS now says half a Pack. The Anchor pulls Remaining Energy up
     * through the band over the next couple of minutes. */
    long in_band = 0;
    bool gone = false;
    for (uint32_t t0 = t; t <= t0 + 10u * 60u * 1000u; t += SAMPLE_MS) {
        feed_ride(&e, t, 55.0, amps_on_curve(&c, 55.0));
        if (t % POLL_MS < SAMPLE_MS) {
            feed_soc(&e, t, 50.0);
        }
        wf_est_out_t o;
        wf_est_get(&e, &o);

        if (o.usable_frac > WF_EST_ADVICE_LOW_FRAC &&
            o.usable_frac <= WF_EST_ADVICE_LOW_FRAC_KEEP) {
            in_band++;
            CHECK(o.advice_valid,
                  "the advice let go at %.4f of a usable Pack, inside the "
                  "%.2f-%.2f band it is supposed to hold through",
                  o.usable_frac, WF_EST_ADVICE_LOW_FRAC,
                  WF_EST_ADVICE_LOW_FRAC_KEEP);
        }
        if (o.usable_frac > WF_EST_ADVICE_LOW_FRAC_KEEP + 0.02 &&
            !o.advice_valid) {
            gone = true;
            break;
        }
    }
    CHECK(in_band > 50,
          "only %ld samples fell inside the hysteresis band, so it was not "
          "really tested", in_band);
    CHECK(gone, "the advice stayed up past the far side of its own band");
}

/* The rider takes the advice. Two things then have to happen, and the second
 * is the one worth a test: the suggestion retires, because it has stopped
 * being a suggestion - and it does not retire so fast that the rider is left
 * wondering whether they imagined it. */
static void test_the_advice_retires_when_it_is_taken(void)
{
    wf_est_curve_t c;
    synth_curve(&c);

    wf_est_t e;
    wf_est_init(&e, NULL);
    wf_est_set_curve(&e, &c);
    feed_soc(&e, 0, 29.0);

    uint32_t t = SAMPLE_MS;
    bool     seen = false;
    double   took = 0.0;
    for (; t <= 10u * 60u * 1000u; t += SAMPLE_MS) {
        feed_ride(&e, t, 55.0, amps_on_curve(&c, 55.0));
        wf_est_out_t o;
        wf_est_get(&e, &o);
        if (o.advice_valid) {
            seen = true;
            took = o.advice_speed_kmh;
            break;
        }
    }
    CHECK(seen, "the advice never appeared, so nothing can be taken");
    if (!seen) {
        return;
    }
    CHECK_D(took, 45.0, 0.0, "the speed the rider was asked to hold");

    /* They ease off to the suggested speed. The cruise average follows over
     * WF_EST_ADVICE_SPEED_TAU_S, the suggestion stops being a step below what
     * they are doing, and the row goes - but not before the dwell. */
    uint32_t t_taken = t;
    bool     gone = false;
    for (; t <= t_taken + 5u * 60u * 1000u; t += SAMPLE_MS) {
        feed_ride(&e, t, took, amps_on_curve(&c, took));
        wf_est_out_t o;
        wf_est_get(&e, &o);
        if (!o.advice_valid) {
            gone = true;
            break;
        }
    }
    CHECK(gone, "the advice stayed up after the rider took it");
    CHECK(t - t_taken >= WF_EST_ADVICE_DWELL_MS,
          "the advice vanished %u ms after appearing, under the %u ms dwell",
          (unsigned)(t - t_taken), (unsigned)WF_EST_ADVICE_DWELL_MS);
}

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
    test_a_nonsense_restore_is_refused();
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

    test_internal_resistance_comes_from_load_steps();
    test_bad_load_steps_are_rejected();
    test_sag_moves_the_limp_point_with_load();
    test_sag_moves_range_with_sustained_load_only();
    test_range_is_monotone_with_a_sagging_pack();
    test_internal_resistance_improves_across_rides();

    test_the_weakest_cell_clamps_range();
    test_a_healthy_pack_at_low_charge_does_not_warn();
    test_bad_cell_readings_do_not_clamp_range();
    test_range_is_monotone_with_a_weak_cell();

    test_the_fit_polynomial_is_what_it_claims();
    test_the_monitor_produces_nothing_without_a_fit();
    test_extrapolation_is_flagged_and_not_hidden();

    test_the_advice_is_silent_without_a_fit();
    test_the_counterfactual_is_a_ratio();
    test_the_advice_is_hidden_on_a_full_pack();
    test_the_advice_is_hidden_when_slowing_buys_nothing();
    test_the_suggested_speed_is_holdable();
    test_the_advice_arrives_when_the_pack_is_low();
    test_the_advice_does_not_flicker();
    test_the_advice_arms_only_on_unbroken_agreement();
    test_the_advice_holds_through_its_own_band();
    test_the_advice_retires_when_it_is_taken();

    test_a_version_1_blob_is_migrated();
    test_a_version_2_blob_is_migrated();
    test_a_version_3_blob_is_migrated();

    if (failures != 0) {
        printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
        return 1;
    }
    printf("unit: odometer wrap, the power block and the estimator, "
           "all assertions hold\n");
    return 0;
}
