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
    /* buf[5] and buf[18..21] stay zero: room for the next field without a
     * version bump being the only way to add one. */
    put_f32(&buf[6],  p->coulomb_ah);
    put_f32(&buf[10], p->remaining_wh);
    put_f32(&buf[14], p->rated_capacity_ah);
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
    if (get_u16(&buf[2]) != WF_EST_PERSIST_VERSION) {
        return false;
    }
    if (get_u16(&buf[22]) !=
        wf_crc16(buf, WF_EST_PERSIST_BYTES - 2, PERSIST_CRC_INIT)) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->version           = get_u16(&buf[2]);
    out->valid             = buf[4] != 0;
    out->coulomb_ah        = get_f32(&buf[6]);
    out->remaining_wh      = get_f32(&buf[10]);
    out->rated_capacity_ah = get_f32(&buf[14]);
    return true;
}

/* ------------------------------------------------------------ the estimator */

void wf_est_init(wf_est_t *e, const wf_est_persist_t *restored)
{
    if (e == NULL) {
        return;
    }
    memset(e, 0, sizeof(*e));

    if (restored == NULL || !restored->valid ||
        restored->version != WF_EST_PERSIST_VERSION) {
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

void wf_est_feed_ctrl(wf_est_t *e, uint32_t t_ms, uint8_t frame_type,
                      const wf_ctrl_live_t *live)
{
    if (e == NULL || live == NULL) {
        return;
    }
    e->last_t_ms    = t_ms;
    e->last_t_valid = true;

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
}
