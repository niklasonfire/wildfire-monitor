/* wfest - see wfest.h. Pure C99, no ESP-IDF, no FreeRTOS, no globals, no
 * clock, no allocation.
 *
 * Two things about the arithmetic, because they are the difference between a
 * curve that replays identically and one that nearly does:
 *
 *   Everything accumulates in double, in a fixed order, with explicit casts.
 *   The decoded fields arrive as float and widening one to double is exact, so
 *   nothing depends on where a promotion happens to land. The Monitor's double
 *   is software-emulated and the host's is SSE2, and both are IEEE-754
 *   binary64 - same operations, same rounding, same answer.
 *
 *   The integration step is driven by the difference of two timestamps the
 *   caller supplied, never by anything this file observes. A step is taken on
 *   a power-block frame and on nothing else, so the sequence of steps is a
 *   property of the recorded stream alone.
 *
 * Both are compiled with -ffp-contract=off; see the Makefile and
 * main/CMakeLists.txt for why that matters.
 */
#include "wfest.h"

#include <string.h>

/* ------------------------------------------------------------- the model */

double wf_est_pack_v_at_soc(double soc_pct)
{
    return WF_EST_PACK_V_FULL + (soc_pct - 100.0) * WF_EST_PACK_V_PER_SOC_PCT;
}

double wf_est_limp_soc_pct(void)
{
    return 100.0 + (WF_EST_LIMP_POINT_V - WF_EST_PACK_V_FULL) /
                   WF_EST_PACK_V_PER_SOC_PCT;
}

double wf_est_energy_above_limp_wh(double soc_pct)
{
    double limp_soc = wf_est_limp_soc_pct();
    if (soc_pct <= limp_soc) {
        return 0.0;
    }
    /* The charge between here and the Limp Point... */
    double ah = ((soc_pct - limp_soc) / 100.0) * WF_EST_RATED_CAPACITY_AH;
    /* ...times the mean voltage it comes out at. The line is straight, so the
     * mean over the interval is the mean of its ends - which is the whole
     * reason Remaining Energy is not just a rescaled State of Charge: the last
     * amp-hour is worth 20 % less than the first. */
    double v_mean = (wf_est_pack_v_at_soc(soc_pct) + WF_EST_LIMP_POINT_V) / 2.0;
    return ah * v_mean;
}

static double anchor_coulomb_ah(double soc_pct)
{
    return (soc_pct / 100.0) * WF_EST_RATED_CAPACITY_AH;
}

/* ------------------------------------------------------------ persistence */

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_f32(uint8_t *p, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    p[0] = (uint8_t)(bits & 0xff);
    p[1] = (uint8_t)((bits >> 8) & 0xff);
    p[2] = (uint8_t)((bits >> 16) & 0xff);
    p[3] = (uint8_t)((bits >> 24) & 0xff);
}

static float get_f32(const uint8_t *p)
{
    uint32_t bits = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

#define PERSIST_MAGIC_0   0x57    /* 'W' */
#define PERSIST_MAGIC_1   0x45    /* 'E' */
/* Modbus CRC-16, same routine both devices use, seeded the standard way. The
 * blob is ours, so the seed is only ever compared against itself. */
#define PERSIST_CRC_INIT  0xffff

bool wf_est_persist_encode(const wf_est_persist_t *p, uint8_t *buf, size_t cap)
{
    if (p == NULL || buf == NULL || cap < WF_EST_PERSIST_BYTES) {
        return false;
    }
    memset(buf, 0, WF_EST_PERSIST_BYTES);
    buf[0] = PERSIST_MAGIC_0;
    buf[1] = PERSIST_MAGIC_1;
    put_u16(&buf[2], p->version);
    buf[4] = p->valid ? 1u : 0u;
    buf[5] = p->distance_valid ? 1u : 0u;   /* version 2; zero in version 1 */
    put_f32(&buf[6],  p->coulomb_ah);
    put_f32(&buf[10], p->remaining_wh);
    put_f32(&buf[14], p->rated_capacity_ah);
    put_f32(&buf[18], p->distance_m);       /* version 2; zero in version 1 */
    put_f32(&buf[22], p->alltime_wh);       /* version 3 from here down */
    put_f32(&buf[26], p->alltime_m);
    put_f32(&buf[30], p->window_wh);
    put_f32(&buf[34], p->window_m);
    put_u16(&buf[38], wf_crc16(buf, WF_EST_PERSIST_BYTES - 2, PERSIST_CRC_INIT));
    return true;
}

bool wf_est_persist_decode(const uint8_t *buf, size_t len, wf_est_persist_t *out)
{
    if (buf == NULL || out == NULL) {
        return false;
    }
    if (len != WF_EST_PERSIST_BYTES && len != WF_EST_PERSIST_BYTES_V2) {
        return false;
    }
    if (buf[0] != PERSIST_MAGIC_0 || buf[1] != PERSIST_MAGIC_1) {
        return false;
    }
    /* The length selects the layout and the version has to agree with it, so
     * no field is ever read out of bytes the writer did not write. A version
     * this build has never written is rejected outright; an older layout it
     * knows is migrated - version 1 is version 2 without distance_m, version 2
     * is version 3 without Consumption, and in both cases the migration is to
     * restore what that build actually knew and no more. */
    uint16_t version = get_u16(&buf[2]);
    if (version < WF_EST_PERSIST_VERSION_MIN ||
        version > WF_EST_PERSIST_VERSION) {
        return false;
    }
    bool v3 = len == WF_EST_PERSIST_BYTES;
    if (v3 != (version >= 3)) {
        return false;
    }
    if (get_u16(&buf[len - 2]) !=
        wf_crc16(buf, len - 2, PERSIST_CRC_INIT)) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->version           = version;
    out->valid             = buf[4] != 0;
    out->coulomb_ah        = get_f32(&buf[6]);
    out->remaining_wh      = get_f32(&buf[10]);
    out->rated_capacity_ah = get_f32(&buf[14]);
    out->distance_valid    = version >= 2 && buf[5] != 0;
    out->distance_m        = version >= 2 ? get_f32(&buf[18]) : 0.0f;
    if (v3) {
        out->alltime_wh = get_f32(&buf[22]);
        out->alltime_m  = get_f32(&buf[26]);
        out->window_wh  = get_f32(&buf[30]);
        out->window_m   = get_f32(&buf[34]);
    }
    return true;
}

/* ------------------------------------------------------------ the estimator */

/* A restored float worth reading. Written as a range rather than as v == v so
 * that it rejects a NaN, an infinity and a bit pattern from a blob that passed
 * its CRC and still holds nonsense, all in one comparison. Energy may be
 * negative - a Monitor that has seen more regeneration than draw - so this is
 * deliberately not a floor. */
static bool sane_f(float v)
{
    return v > -1e30f && v < 1e30f;
}

void wf_est_init(wf_est_t *e, const wf_est_persist_t *restored)
{
    if (e == NULL) {
        return;
    }
    memset(e, 0, sizeof(*e));

    if (restored == NULL ||
        restored->version < WF_EST_PERSIST_VERSION_MIN ||
        restored->version > WF_EST_PERSIST_VERSION) {
        return;
    }

    /* Distance first, and on its own terms: it is metres travelled, it does
     * not depend on the Pack model, and the Rated Capacity guard below has
     * nothing to say about it. The comparison is written the way round that
     * rejects a NaN as well as a negative. */
    if (restored->distance_valid && restored->distance_m >= 0.0f) {
        e->distance_m        = (double)restored->distance_m;
        e->distance_restored = true;
    }

    /* The all-time totals, on their own terms again: they are a lifetime of
     * riding and they depend on neither the Pack model nor an Anchor. Both
     * comparisons are written the way round that rejects a NaN, and both
     * totals are taken or neither is - half a ratio is not a ratio. Energy may
     * legitimately be negative for a Monitor that has seen more regeneration
     * than draw, which is why only the distance is floored. */
    if (restored->alltime_m >= 0.0f && sane_f(restored->alltime_m) &&
        sane_f(restored->alltime_wh)) {
        e->alltime_m  = (double)restored->alltime_m;
        e->alltime_wh = (double)restored->alltime_wh;
    }

    /* The window, spread evenly over the ring. It is saved only when the ring
     * was full, so a non-zero window_m means there is a whole window's worth
     * of recent riding to come back to - a coffee stop does not cost the rider
     * their Consumption figure. Anything less than a full window was not saved
     * and is not invented here. */
    if (restored->window_m >= (float)WF_EST_CONS_WINDOW_M &&
        sane_f(restored->window_m) && sane_f(restored->window_wh)) {
        double per_m  = (double)restored->window_m / WF_EST_CONS_BUCKETS;
        double per_wh = (double)restored->window_wh / WF_EST_CONS_BUCKETS;
        for (int i = 0; i < WF_EST_CONS_BUCKETS; i++) {
            e->cons_m[i]  = per_m;
            e->cons_wh[i] = per_wh;
        }
        e->cons_head   = 0;
        e->cons_filled = WF_EST_CONS_BUCKETS;
    }

    if (!restored->valid) {
        return;
    }
    /* A count saved against a different Rated Capacity is a count of something
     * else. Rather than rescale a number whose provenance we would then be
     * guessing at, start cold - the first BMS answer acquires within a second
     * either way. */
    if ((double)restored->rated_capacity_ah != WF_EST_RATED_CAPACITY_AH) {
        return;
    }
    e->coulomb_ah   = (double)restored->coulomb_ah;
    e->remaining_wh = (double)restored->remaining_wh;
    e->restored     = true;
}

/* ------------------------------------------------------------- Consumption */

/* The bucket in progress has collected its span of road. File it in the ring,
 * dropping whatever it displaces, and start a fresh one.
 *
 * A bucket ends up holding its span plus the overshoot of the motion step that
 * crossed the boundary - fifty metres plus a couple, at road speed - and the
 * overshoot is not carried forward. That is why the ratio is computed from the
 * ring's own summed metres rather than from BUCKETS * BUCKET_M: the window is
 * a length of road that was actually covered, not a length assumed. */
static void cons_close_bucket(wf_est_t *e)
{
    e->cons_m[e->cons_head]  = e->cons_part_m;
    e->cons_wh[e->cons_head] = e->cons_part_wh;
    e->cons_head = (uint8_t)((e->cons_head + 1u) % WF_EST_CONS_BUCKETS);
    if (e->cons_filled < WF_EST_CONS_BUCKETS) {
        e->cons_filled++;
    }
    e->cons_part_m  = 0.0;
    e->cons_part_wh = 0.0;
}

/* The ring's totals, summed in index order and never in ring order, so the
 * sum is a function of the contents alone: two runs that reached the same
 * buckets by different routes add them up in the same sequence and get the
 * same last bit. Twenty additions, which is why nothing here is accumulated
 * incrementally - an incremental total would drift as buckets came and went,
 * and would drift differently depending on when the ride started. */
static void cons_window(const wf_est_t *e, double *wh, double *m)
{
    double sum_wh = 0.0;
    double sum_m  = 0.0;
    for (int i = 0; i < WF_EST_CONS_BUCKETS; i++) {
        sum_wh += e->cons_wh[i];
        sum_m  += e->cons_m[i];
    }
    *wh = sum_wh;
    *m  = sum_m;
}

/* ---------------------------------------------------------------- distance */

/* One Odometer reading into the Anchor. Never moves distance itself, exactly
 * as a BMS answer never moves the Coulomb Count - the pull in distance_step()
 * is the only thing that does. */
static void odo_fold(wf_est_t *e, const wf_ctrl_live_t *live)
{
    if (!live->odo_valid) {
        return;
    }
    if (!e->odo_seen) {
        /* Acquisition, and the only assignment in here. Setting the Anchor to
         * whatever distance already is costs nothing at all - it is the
         * Odometer agreeing to measure from here - which is what lets it
         * acquire mid-ride, or against a distance restored from NVS, without
         * a step. */
        e->odo_counts     = live->odometer_raw;
        e->odo_distance_m = e->distance_m;
        e->odo_seen       = true;
        e->odo_samples++;
        return;
    }

    /* Wrap-safe u16 modular subtraction, in counts, before anything becomes a
     * distance: 65535 to 0 is one count. Differences only, never the absolute
     * reading, so the wrap is not a case to detect. */
    uint16_t d_counts = wf_ctrl_odo_delta_counts(e->odo_counts,
                                                 live->odometer_raw);
    e->odo_counts = live->odometer_raw;
    e->odo_samples++;

    /* More than half the counter's span forward is more plausibly a reading
     * that went backwards - which this subtraction cannot distinguish from a
     * nearly-complete wrap, and says so in wfdecode.h. Re-base on the new
     * reading and credit nothing: crediting it would put a 3000 km phantom
     * trip into the Anchor and drag distance after it for the rest of the
     * ride. Nothing on this bike covers 3276 km between two Odometer frames.
     */
    if (d_counts > 0x8000u) {
        return;
    }
    e->odo_distance_m += (double)wf_ctrl_odo_metres(d_counts);
}

/* One motion frame: integrate road speed, then pull toward the Odometer. */
static void distance_step(wf_est_t *e, uint32_t t_ms,
                          const wf_ctrl_live_t *live)
{
    if (!e->speed_t_valid) {
        /* Nothing to integrate over yet: the first frame only starts the
         * clock, the same as the first power sample. */
        e->speed_t_ms    = t_ms;
        e->speed_t_valid = true;
        return;
    }

    uint32_t dt_ms = t_ms - e->speed_t_ms;
    e->speed_t_ms = t_ms;
    if (dt_ms == 0) {
        return;
    }
    if (dt_ms > WF_EST_DT_MAX_MS) {
        /* A link that dropped for a minute must not book a minute at the last
         * speed it happened to see. The metres actually covered during the
         * drop come back through the Odometer instead, which is the whole
         * point of having one. */
        dt_ms = WF_EST_DT_MAX_MS;
    }
    double dt_s = (double)dt_ms / 1000.0;

    double d_m = 0.0;
    if (live->speed_valid) {
        /* km/h to m/s. The speed itself is wf_ctrl_speed_kmh()'s and is not
         * re-derived here; this is a unit conversion and nothing more. */
        d_m = (double)live->cur_speed_kmh * (1000.0 / 3600.0) * dt_s;
        if (d_m < 0.0) {
            d_m = 0.0;
        }
        e->speed_seen = true;
    }

    double pull = 0.0;
    if (e->odo_seen) {
        double k = dt_s / WF_EST_DIST_TAU_S;
        if (k > 1.0) {
            k = 1.0;
        }
        pull = (e->odo_distance_m - e->distance_m) * k;
    }

    /* Floored at zero, which is the one place distance is not treated the way
     * the Coulomb Count is. The correction may slow the figure to a standstill
     * and may not reverse it: a distance that ticks backwards on the rider's
     * screen is wrong in a way a watt-hour figure is not, and an over-reading
     * speed is absorbed by the figure sitting still while the Odometer catches
     * up. */
    double step = d_m + pull;
    if (step < 0.0) {
        step = 0.0;
    }
    e->distance_m += step;
    e->distance_samples++;

    /* Consumption's denominator, and it is the same metres the rider watches
     * accumulate - the fused figure, not the raw integral of speed - so the
     * two can never disagree about how far the bike has gone. */
    e->alltime_m   += step;
    e->cons_part_m += step;
    if (e->cons_part_m >= WF_EST_CONS_BUCKET_M) {
        cons_close_bucket(e);
    }
}

void wf_est_feed_ctrl(wf_est_t *e, uint32_t t_ms, uint8_t frame_type,
                      const wf_ctrl_live_t *live)
{
    if (e == NULL || live == NULL) {
        return;
    }
    e->last_t_ms    = t_ms;
    e->last_t_valid = true;

    if (frame_type == WF_CTRL_TYPE_ODO) {
        odo_fold(e, live);
        return;
    }
    if (wf_ctrl_type_in(wf_ctrl_type_motion, WF_CTRL_TYPE_MOTION_COUNT,
                        frame_type)) {
        distance_step(e, t_ms, live);
        return;
    }

    if (!wf_ctrl_type_in(wf_ctrl_type_power, WF_CTRL_TYPE_POWER_COUNT,
                         frame_type) ||
        !live->power_valid) {
        return;
    }

    double volts = (double)live->pack_v;
    double amps  = (double)live->line_current_a;
    e->last_power_w = volts * amps;
    e->power_samples++;

    if (!e->power_t_valid) {
        /* Nothing to integrate over yet: the first sample only starts the
         * clock. */
        e->power_t_ms    = t_ms;
        e->power_t_valid = true;
        return;
    }

    /* Modular, so a timestamp that wraps is a short interval and not a
     * backwards leap - the same arithmetic the Odometer's delta uses, for the
     * same reason. */
    uint32_t dt_ms = t_ms - e->power_t_ms;
    e->power_t_ms = t_ms;
    if (dt_ms == 0) {
        return;
    }
    if (dt_ms > WF_EST_DT_MAX_MS) {
        /* A link that dropped for a minute must not book a minute of whatever
         * current it happened to be drawing when it went. */
        dt_ms = WF_EST_DT_MAX_MS;
    }

    double dt_s   = (double)dt_ms / 1000.0;
    double dt_h   = dt_s / 3600.0;
    double d_ah   = amps * dt_h;          /* positive is discharge */
    double d_wh   = volts * amps * dt_h;

    e->used_ah      += d_ah;
    e->used_wh      += d_wh;
    e->coulomb_ah   -= d_ah;
    e->remaining_wh -= d_wh;

    /* Consumption's numerator, taken here and not from remaining_wh: the
     * Anchor's pull below moves Remaining Energy without any energy having
     * left the Pack, and a Consumption built on that would be measuring the
     * BMS rather than the riding. Energy drawn while standing still goes in
     * too - it did leave the Pack - and lands in whichever bucket is open when
     * the bike next moves. */
    e->alltime_wh   += d_wh;
    e->cons_part_wh += d_wh;

    /* The Anchor, applied as a pull and never as an assignment. Nothing
     * happens while it is stale, which is the whole of the gap behaviour: the
     * integration runs alone through the gap and the pull resumes at its own
     * pace afterwards, so the rider sees no step at the moment the BMS comes
     * back. */
    if (!e->acquired || !e->anchor_seen) {
        return;
    }
    uint32_t age_ms = t_ms - e->anchor_t_ms;
    if (age_ms > WF_EST_ANCHOR_STALE_MS) {
        return;
    }
    double k = dt_s / WF_EST_ANCHOR_TAU_S;
    if (k > 1.0) {
        k = 1.0;
    }
    e->coulomb_ah   += (anchor_coulomb_ah(e->anchor_soc_pct) - e->coulomb_ah) * k;
    e->remaining_wh += (wf_est_energy_above_limp_wh(e->anchor_soc_pct) -
                        e->remaining_wh) * k;
}

void wf_est_feed_bms(wf_est_t *e, uint32_t t_ms, const wf_bms_t *bms)
{
    if (e == NULL || bms == NULL) {
        return;
    }
    e->last_t_ms    = t_ms;
    e->last_t_valid = true;

    double soc = (double)bms->soc_pct;
    if (!(soc >= 0.0) || soc > 100.0) {
        /* Not a percentage, so not an Anchor. The replay harness asserts the
         * same bound on every Capture; restated here because the estimator
         * must not be the thing that trusts a decoder bug. */
        return;
    }

    e->anchor_soc_pct = soc;
    e->anchor_t_ms    = t_ms;
    e->anchor_seen    = true;
    e->anchor_samples++;

    if (!e->acquired) {
        /* Acquisition: the one and only time a BMS answer is assigned rather
         * than pulled toward. There is no integrated value to preserve yet. */
        e->coulomb_ah   = anchor_coulomb_ah(soc);
        e->remaining_wh = wf_est_energy_above_limp_wh(soc);
        e->acquired     = true;
    }
}

void wf_est_get(const wf_est_t *e, wf_est_out_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (e == NULL) {
        return;
    }

    out->valid    = e->acquired || e->restored;
    out->anchored = e->acquired;

    /* Clamped at zero for the rider. Past the Limp Point there is no model
     * here worth showing a number from - issue #8 measures what actually
     * happens down there - and "0 Wh" is the honest thing to say meanwhile.
     * The unclamped value stays in the state for the harness to look at. */
    out->remaining_wh = e->remaining_wh > 0.0 ? e->remaining_wh : 0.0;

    out->coulomb_ah     = e->coulomb_ah;
    out->soc_pct        = (e->coulomb_ah / WF_EST_RATED_CAPACITY_AH) * 100.0;
    out->anchor_soc_pct = e->anchor_soc_pct;
    out->used_ah        = e->used_ah;
    out->used_wh        = e->used_wh;
    out->power_w        = e->last_power_w;
    out->power_samples  = e->power_samples;
    out->anchor_samples = e->anchor_samples;

    out->distance_valid   = e->speed_seen || e->odo_seen || e->distance_restored;
    out->odo_anchored     = e->odo_seen;
    out->distance_m       = e->distance_m;
    out->odo_distance_m   = e->odo_distance_m;
    out->distance_samples = e->distance_samples;
    out->odo_samples      = e->odo_samples;

    /* Consumption. Two candidate divisions, and the guard on each is the
     * whole of the stationary-bike criterion.
     *
     * The window first, because it is the figure worth showing: the ring
     * has to hold a full window of closed buckets before it says anything, so
     * a bike that has never covered WF_EST_CONS_WINDOW_M cannot produce a
     * windowed figure at all and no explicit guard is needed - the summed
     * metres are a kilometre by construction.
     *
     * The all-time average second, standing in until then, and that one does
     * need a floor: its denominator starts at nothing on a Monitor that has
     * never ridden, and a hundred watt-hours over four metres is not a large
     * Consumption but a meaningless one. WF_EST_CONS_MIN_DIST_M is one
     * Odometer count, the shortest distance the Monitor can claim to know.
     *
     * Below both, `consumption_valid` is false and the screen shows a dash.
     * That is the honest answer and it is what cap0007's 16.7 m produces. */
    double win_wh, win_m;
    cons_window(e, &win_wh, &win_m);
    out->window_m   = win_m;
    out->alltime_m  = e->alltime_m;
    out->alltime_wh = e->alltime_wh;

    if (e->cons_filled >= WF_EST_CONS_BUCKETS && win_m > 0.0) {
        out->consumption_valid    = true;
        out->consumption_windowed = true;
        out->consumption_wh_per_km = win_wh / win_m * 1000.0;
    } else if (e->alltime_m >= WF_EST_CONS_MIN_DIST_M) {
        out->consumption_valid    = true;
        out->consumption_windowed = false;
        out->consumption_wh_per_km = e->alltime_wh / e->alltime_m * 1000.0;
    }

    if (e->anchor_seen && e->last_t_valid) {
        out->anchor_age_ms = e->last_t_ms - e->anchor_t_ms;
        out->anchor_fresh  = out->anchor_age_ms <= WF_EST_ANCHOR_STALE_MS;
    }
}

void wf_est_save(const wf_est_t *e, wf_est_persist_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->version           = WF_EST_PERSIST_VERSION;
    out->rated_capacity_ah = (float)WF_EST_RATED_CAPACITY_AH;
    if (e == NULL) {
        return;
    }
    out->valid        = e->acquired || e->restored;
    out->coulomb_ah   = (float)e->coulomb_ah;
    out->remaining_wh = (float)e->remaining_wh;
    /* Its own flag: `valid` above is about the charge figures, which need an
     * Anchor this Monitor may never have got, and distance needs no such
     * thing. A ride with a Controller and no BMS still has metres to keep. */
    out->distance_valid = e->speed_seen || e->odo_seen || e->distance_restored;
    out->distance_m     = (float)e->distance_m;

    /* The all-time totals, unconditionally: they are the only thing here that
     * is supposed to improve across rides rather than be replaced by the last
     * one, and they do that by being totals. Zeroes cost nothing to write and
     * a flag would only be a second way of saying alltime_m is zero. */
    out->alltime_wh = (float)e->alltime_wh;
    out->alltime_m  = (float)e->alltime_m;

    /* The window, only when it is a whole one. Half a window restored as if it
     * were whole would report a windowed figure over half the road it claims;
     * the all-time average is the honest fallback until the ring fills again.
     * The bucket in progress is not saved with it - at most one bucket's worth
     * of road is lost across a power cycle. */
    if (e->cons_filled >= WF_EST_CONS_BUCKETS) {
        double win_wh, win_m;
        cons_window(e, &win_wh, &win_m);
        out->window_wh = (float)win_wh;
        out->window_m  = (float)win_m;
    }
}
