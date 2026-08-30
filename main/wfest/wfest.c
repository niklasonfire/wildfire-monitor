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
    put_u16(&buf[22], wf_crc16(buf, WF_EST_PERSIST_BYTES - 2, PERSIST_CRC_INIT));
    return true;
}

bool wf_est_persist_decode(const uint8_t *buf, size_t len, wf_est_persist_t *out)
{
    if (buf == NULL || out == NULL || len != WF_EST_PERSIST_BYTES) {
        return false;
    }
    if (buf[0] != PERSIST_MAGIC_0 || buf[1] != PERSIST_MAGIC_1) {
        return false;
    }
    /* A version this build has never written is rejected outright; an older
     * one it knows the layout of is migrated. Version 1 is version 2 without
     * distance_m, in bytes it left zeroed, so the migration is to read the
     * distance as the zero a build that did not count metres knew. */
    uint16_t version = get_u16(&buf[2]);
    if (version < WF_EST_PERSIST_VERSION_MIN ||
        version > WF_EST_PERSIST_VERSION) {
        return false;
    }
    if (get_u16(&buf[22]) !=
        wf_crc16(buf, WF_EST_PERSIST_BYTES - 2, PERSIST_CRC_INIT)) {
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
    return true;
}

/* ------------------------------------------------------------ the estimator */

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
}
