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

double wf_est_sag_reserve_wh(double sag_v)
{
    /* Written the way round that rejects a NaN as well as a negative: no Sag,
     * no reserve. */
    if (!(sag_v > 0.0)) {
        return 0.0;
    }
    /* How much of the Pack the band is worth. The line is straight, so a band
     * of a given width in volts is the same number of amp-hours wherever it
     * sits - which is why this needs no State of Charge. */
    double pct = sag_v / WF_EST_PACK_V_PER_SOC_PCT;
    double ah  = (pct / 100.0) * WF_EST_RATED_CAPACITY_AH;
    /* Times the mean voltage that charge would have come out at: the band runs
     * from the Limp Point up to one Sag above it. */
    return ah * (WF_EST_LIMP_POINT_V + sag_v / 2.0);
}

double wf_est_cell_band_v(double spread_v)
{
    /* Written the way round that rejects a NaN as well as a spread inside the
     * deadband: a healthy Pack's imbalance is already in the line the Pack
     * model was fitted to, and is not charged to the rider a second time. */
    if (!(spread_v > WF_EST_CELL_DEADBAND_V)) {
        return 0.0;
    }
    return WF_EST_PACK_CELLS * (spread_v - WF_EST_CELL_DEADBAND_V);
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
    put_f32(&buf[38], p->ir_ohm);           /* version 4 from here down */
    put_u16(&buf[42], p->ir_weight);
    put_u16(&buf[44], wf_crc16(buf, WF_EST_PERSIST_BYTES - 2, PERSIST_CRC_INIT));
    return true;
}

/* How long a blob of a given version is. The length is what selects the layout
 * and the version has to agree with it, which is what stops a field ever being
 * read out of bytes the writer did not write. */
static size_t persist_bytes_for(uint16_t version)
{
    if (version >= 4) {
        return WF_EST_PERSIST_BYTES;
    }
    if (version >= 3) {
        return WF_EST_PERSIST_BYTES_V3;
    }
    return WF_EST_PERSIST_BYTES_V2;
}

bool wf_est_persist_decode(const uint8_t *buf, size_t len, wf_est_persist_t *out)
{
    if (buf == NULL || out == NULL) {
        return false;
    }
    if (len != WF_EST_PERSIST_BYTES && len != WF_EST_PERSIST_BYTES_V3 &&
        len != WF_EST_PERSIST_BYTES_V2) {
        return false;
    }
    if (buf[0] != PERSIST_MAGIC_0 || buf[1] != PERSIST_MAGIC_1) {
        return false;
    }
    /* The length selects the layout and the version has to agree with it, so
     * no field is ever read out of bytes the writer did not write. A version
     * this build has never written is rejected outright; an older layout it
     * knows is migrated - version 1 is version 2 without distance_m, version 2
     * is version 3 without Consumption, version 3 is version 4 without
     * Internal Resistance, and in every case the migration is to restore what
     * that build actually knew and no more. */
    uint16_t version = get_u16(&buf[2]);
    if (version < WF_EST_PERSIST_VERSION_MIN ||
        version > WF_EST_PERSIST_VERSION) {
        return false;
    }
    if (len != persist_bytes_for(version)) {
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
    if (version >= 3) {
        out->alltime_wh = get_f32(&buf[22]);
        out->alltime_m  = get_f32(&buf[26]);
        out->window_wh  = get_f32(&buf[30]);
        out->window_m   = get_f32(&buf[34]);
    }
    if (version >= 4) {
        out->ir_ohm    = get_f32(&buf[38]);
        out->ir_weight = get_u16(&buf[42]);
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

    /* The one thing a Monitor is seeded with that did not come out of NVS: the
     * offline fit, copied out of wf_fit.h. It is a copy of #defines and not a
     * decision - ADR-0005 - and nothing on the bike ever writes it again. */
    wf_est_curve_default(&e->advice_curve);

    if (restored == NULL ||
        restored->version < WF_EST_PERSIST_VERSION_MIN ||
        restored->version > WF_EST_PERSIST_VERSION) {
        return;
    }

    /* Distance first, and on its own terms: it is metres travelled, it does
     * not depend on the Pack model, and the Rated Capacity guard below has
     * nothing to say about it. The comparison is written the way round that
     * rejects a NaN as well as a negative. */
    if (restored->distance_valid && restored->distance_m >= 0.0f &&
        sane_f(restored->distance_m)) {
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

    /* Internal Resistance, on its own terms again: it is a fact about the Pack
     * and needs neither an Anchor nor the Rated Capacity guard below. Both
     * halves or neither, and only inside the plausibility bound - a blob that
     * passed its CRC and still holds two ohms is not a Pack, and restoring it
     * would put a Sag reserve of the whole battery on the rider's screen. The
     * weight is capped on the way in as well as on the way out, so a corrupted
     * count cannot freeze the estimate.
     *
     * Restored at full weight and therefore at full confidence: the fade-in is
     * for a Monitor learning its Pack for the first time, not for one that
     * already knows it and has just been switched off and on. */
    if (restored->ir_weight > 0 &&
        (double)restored->ir_ohm >= WF_EST_IR_MIN_OHM &&
        (double)restored->ir_ohm <= WF_EST_IR_MAX_OHM) {
        e->ir_ohm    = (double)restored->ir_ohm;
        e->ir_weight = restored->ir_weight > WF_EST_IR_SAMPLES_MAX
                           ? WF_EST_IR_SAMPLES_MAX
                           : (uint32_t)restored->ir_weight;
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
    /* And the same check every other restored float gets, on the two figures
     * that need it most. Both or neither: the count and the energy are one
     * statement about the Pack, and half of it is not one.
     *
     * A NaN here would not be wrong once. The Anchor corrects Remaining Energy
     * by `x += (target - x) * k`, which is NaN for every k, so no BMS answer
     * could ever pull it back; Range is a quotient of it and would follow it;
     * and wf_est_save() would write it out again under a fresh valid CRC. It
     * is the one error in this module that a power cycle makes worse. */
    if (!sane_f(restored->coulomb_ah) || !sane_f(restored->remaining_wh)) {
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

/* ------------------------------------------------- Internal Resistance and Sag
 *
 * One pair of consecutive power-block samples, and the question is whether it
 * is a load step worth measuring a Pack with. Every answer here is "no" except
 * on a real launch or a real throttle release, which is the point: a mean of
 * everything the stream offers is a mean of quantisation noise.
 *
 * The rules, in the order they are cheapest to apply, and every one of them
 * drops the pair rather than folding a weakened version of it in:
 *
 *   not consecutive     the two samples must be at most WF_EST_IR_MAX_DT_MS
 *                       apart. A pair straddling a dropped frame is a ramp
 *                       that looks like a step, and there is no way to tell
 *                       from the numbers which it was.
 *   too small           |dI| below WF_EST_IR_MIN_DI_A cannot beat the two
 *                       quantisations - see the derivation in wfest.h. This is
 *                       the rule that rejects almost every pair, and it is
 *                       supposed to.
 *   a BMS gap           a resistance with no State of Charge attached to it is
 *                       not a State of Health reading, because resistance
 *                       depends strongly on charge and on temperature. If the
 *                       Anchor is stale the step is thrown away rather than
 *                       recorded against a State of Charge nobody vouches for.
 *   not a Pack          the ratio has to land inside the plausibility bound.
 *                       This is also what rejects a step where the voltage
 *                       moved the wrong way - that gives a negative ratio,
 *                       which is below the floor - and any step where the
 *                       voltage did not move at all.
 *
 * What survives is folded into a running mean whose weight is capped, so an
 * estimate that has seen a few launches stops moving with any one of them.
 */
static void ir_fold(wf_est_t *e, uint32_t t_ms, uint32_t dt_ms, double volts,
                    double amps)
{
    if (dt_ms == 0 || dt_ms > WF_EST_IR_MAX_DT_MS) {
        return;
    }
    double di = amps - e->ir_prev_a;
    double adi = di < 0.0 ? -di : di;
    if (!(adi >= WF_EST_IR_MIN_DI_A)) {
        return;
    }
    /* Sharp enough to try. From here on a refusal is worth counting: a Pack
     * that produces launches and nothing but rejections is a decode problem or
     * a wiring problem, and a silent zero would hide it. */
    if (!e->acquired || !e->anchor_seen ||
        (uint32_t)(t_ms - e->anchor_t_ms) > WF_EST_ANCHOR_STALE_MS) {
        e->ir_rejected++;
        return;
    }
    /* Sag per amp. The minus is the physics: voltage falls as current rises,
     * and both directions of step give the same positive resistance. */
    double r = -(volts - e->ir_prev_v) / di;
    if (!(r >= WF_EST_IR_MIN_OHM) || r > WF_EST_IR_MAX_OHM) {
        e->ir_rejected++;
        return;
    }

    if (e->ir_weight < WF_EST_IR_SAMPLES_MAX) {
        e->ir_weight++;
    }
    e->ir_ohm += (r - e->ir_ohm) / (double)e->ir_weight;
    e->ir_soc_pct  = e->anchor_soc_pct;
    e->ir_steps++;
}

/* The load Sag is taken at: the line current, averaged over WF_EST_SAG_TAU_S.
 *
 * This is the whole of why a Sag-corrected Limp Point does not make Range
 * jitter. The Limp Point moves with this number rather than with the last
 * frame, so one noisy sample moves it by dt/TAU - about a hundredth - of what
 * the same current held for a minute does, while a launch the rider is
 * actually holding walks it up frame by frame and easing off walks it back
 * down. The filter is in the input, one level below the quotient, exactly
 * where Consumption's is; Range still has none of its own.
 *
 * The first sample assigns rather than pulls, the same acquisition the Anchor
 * and the Odometer make: there is no previous average to preserve, and
 * starting from zero would spend the first twenty seconds of every ride
 * understating the Sag the rider is already paying.
 */
static void load_step(wf_est_t *e, uint32_t dt_ms, double amps)
{
    if (!e->load_valid) {
        e->load_a     = amps;
        e->load_valid = true;
        return;
    }
    if (dt_ms == 0) {
        return;
    }
    if (dt_ms > WF_EST_DT_MAX_MS) {
        dt_ms = WF_EST_DT_MAX_MS;
    }
    double k = ((double)dt_ms / 1000.0) / WF_EST_SAG_TAU_S;
    if (k > 1.0) {
        k = 1.0;
    }
    e->load_a += (amps - e->load_a) * k;
}

/* One power sample into both, then remembered as the previous one. Called
 * before the energy integration so that the pair it measures across is the
 * pair the stream delivered, whatever the integration then decides to do with
 * the interval. */
static void sag_step(wf_est_t *e, uint32_t t_ms, double volts, double amps)
{
    uint32_t dt_ms = 0;
    if (e->ir_prev_valid) {
        dt_ms = t_ms - e->ir_prev_t_ms;   /* modular, like every other delta */
        ir_fold(e, t_ms, dt_ms, volts, amps);
    }
    load_step(e, dt_ms, amps);

    e->ir_prev_v     = volts;
    e->ir_prev_a     = amps;
    e->ir_prev_t_ms  = t_ms;
    e->ir_prev_valid = true;
}

/* -------------------------------------------------------- the weakest Cell
 *
 * One BMS response's Cell block, and the question asked of it first is whether
 * it describes a 28-Cell lithium Pack at all. It is the same shape of question
 * ir_fold() asks of a load step and it is refused the same way - the block is
 * dropped whole, the previous reading stands, and the refusal is counted:
 *
 *   not this Pack       a cell count that is not WF_EST_PACK_CELLS. A response
 *                       that decoded but carries a zero-filled array fails
 *                       here or on the next rule, and either way produces no
 *                       clamp rather than a Pack of dead Cells.
 *   not a lithium Cell  any Cell outside WF_EST_CELL_MIN_MV..MAX_MV. This is
 *                       the rule that catches the zero fill and the 6 V
 *                       "Cell", and it is the same 2.0-4.5 V bound
 *                       tests/host/replay.c holds every Capture to.
 *   registers disagree  an average that is not between the array's own lowest
 *                       and highest Cell. The average is the BMS's own answer
 *                       and the array is what checks it; two registers that
 *                       cannot both be true are not a Pack.
 *   too far gone        a spread past WF_EST_CELL_MAX_SPREAD_V, where a dying
 *                       Cell and a slipped register map stop being
 *                       distinguishable. Refusing is what keeps Range at the
 *                       unclamped estimate instead of at zero.
 *
 * What survives is folded into three running means whose weight is capped, so
 * an imbalance that has been watched for a few answers stops moving with any
 * one of them - and one millivolt of movement on a 1 mV quantised register
 * cannot walk the rider's Range. Indexed by answers and not by time, so there
 * is no second clock and no second time constant.
 */
static void cell_fold(wf_est_t *e, const wf_bms_t *bms)
{
    /* The Pack model's Cell count has to fit the decoder's array, or the loop
     * below would read past it. Compile-time, because it is a fact about two
     * constants and not about a Pack. */
    typedef char wfest_cells_fit[(WF_EST_PACK_CELLS <= WF_BMS_MAX_CELLS)
                                 ? 1 : -1];
    (void)sizeof(wfest_cells_fit);

    if (bms->cell_count != WF_EST_PACK_CELLS) {
        e->cell_rejected++;
        return;
    }

    /* Imbalance is read at rest and nowhere else. The Cell block is scanned
     * one Cell at a time, so under load it is a ramp in time rather than a
     * profile across Cells - cap0001 puts 48 mV of that into a Pack that sits
     * 5 mV apart when it is quiet. Not a rejection: the answer is true, it
     * just does not answer this question. See the header comment. */
    double amps = (double)bms->current_a;
    if (amps < 0.0) {
        amps = -amps;
    }
    if (amps >= WF_EST_CELL_QUIET_A) {
        return;
    }

    /* The lowest, the second-lowest and the highest, in one pass. The
     * second-lowest is what the Pack's own fan-out is measured against, and it
     * is the reason the highest and lowest registers are not simply read: they
     * do not carry it. */
    uint16_t lo = 0xffffu, lo2 = 0xffffu, hi = 0;
    for (int i = 0; i < WF_EST_PACK_CELLS; i++) {
        uint16_t mv = bms->cell_mv[i];
        if (mv < WF_EST_CELL_MIN_MV || mv > WF_EST_CELL_MAX_MV) {
            e->cell_rejected++;
            return;
        }
        if (mv < lo) {
            lo2 = lo;
            lo  = mv;
        } else if (mv < lo2) {
            lo2 = mv;
        }
        if (mv > hi) {
            hi = mv;
        }
    }

    uint16_t avg = bms->avg_cell_mv;
    if (avg < lo || avg > hi) {
        e->cell_rejected++;
        return;
    }
    double spread = (double)(avg - lo) / 1000.0;
    if (spread > WF_EST_CELL_MAX_SPREAD_V) {
        e->cell_rejected++;
        return;
    }
    double gap  = (double)(lo2 - lo) / 1000.0;
    double body = (double)(hi - lo2) / 1000.0;

    if (e->cell_weight < WF_EST_CELL_SAMPLES_MAX) {
        e->cell_weight++;
    }
    double k = 1.0 / (double)e->cell_weight;
    e->cell_spread_v += (spread - e->cell_spread_v) * k;
    e->cell_gap_v    += (gap    - e->cell_gap_v)    * k;
    e->cell_body_v   += (body   - e->cell_body_v)   * k;
    e->cell_min_mv    = lo;
    e->cell_avg_mv    = avg;
    e->cell_samples++;
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

/* --------------------------------------------------------------- the advice */

/* Whether a suggestion is one the rider could hold, given what they are doing.
 * Three floors, and the proportional one is the load-bearing one: what counts
 * as backing off depends on the speed being backed off from. Every comparison
 * is written the way round that rejects a NaN.
 *
 * The fit's own supported range is deliberately not here. It belongs to
 * wf_est_advice_at(), which is where `extrapolated` comes back from, so that
 * "the rider could hold this" and "the archive can speak about this" stay two
 * separate refusals. */
static bool advice_holdable(double cruise_kmh, double speed_kmh)
{
    return speed_kmh >= WF_EST_ADVICE_MIN_SPEED_KMH &&
           speed_kmh <= cruise_kmh - WF_EST_ADVICE_STEP_KMH &&
           speed_kmh >= cruise_kmh * (1.0 - WF_EST_ADVICE_MAX_DROP_FRAC);
}

/* The stop side of the hysteresis: does the advice already on the screen still
 * earn its place? The latched speed is re-evaluated rather than re-picked, so
 * a suggestion cannot quietly become a different suggestion - if the latched
 * one has stopped being holdable or has stopped being worth taking, the
 * episode ends and a new one may begin later at a new speed.
 *
 * The two judgement thresholds here are the widened _KEEP ones, which is the
 * whole of the hysteresis: the boundary that ends an episode is not the
 * boundary that started it. The kilometre floor is not widened; see below. */
static bool advice_holds(const wf_est_curve_t *curve, double cruise_kmh,
                         double range_km, double usable_frac, double speed_kmh,
                         wf_est_advice_t *out)
{
    if (!(usable_frac <= WF_EST_ADVICE_LOW_FRAC_KEEP)) {
        return false;
    }
    if (!advice_holdable(cruise_kmh, speed_kmh)) {
        return false;
    }
    if (!wf_est_advice_at(curve, cruise_kmh, range_km, speed_kmh, out)) {
        return false;
    }
    /* The fraction widens; the kilometre floor does not, because it is a fact
     * about what the panel can render and not a judgement that can be stickier
     * once it has been made. */
    return out->gain_km >= WF_EST_ADVICE_GAIN_KM &&
           out->gain_frac >= WF_EST_ADVICE_GAIN_FRAC_KEEP;
}

/* One motion frame's worth of "should the advice be up", and the only place
 * the decision is made. It runs here, on the motion block, because that is
 * where road speed arrives and where a dt worth timing with already exists.
 *
 * It reads the estimate through wf_est_get() rather than reaching into the
 * accumulators, which costs a memset and a few dozen flops at ~5.2 Hz and buys
 * the property worth having: the advice is judged against exactly the figures
 * the rider is being shown, Sag, Cell clamp, window handover and all. There is
 * no recursion - wf_est_get() computes and stores nothing.
 *
 * The flip is deliberately not `want != shown`. While the advice is hidden the
 * question is "should this start", while it is up the question is "should this
 * stop", and they are asked of different functions with different thresholds.
 * What the timers see is one bit either way: the condition that would flip the
 * current state either holds this instant or does not.
 */
static void advice_step(wf_est_t *e, uint32_t dt_ms)
{
    wf_est_out_t o;
    wf_est_get(e, &o);

    wf_est_advice_t a;
    memset(&a, 0, sizeof(a));

    bool flipping;
    if (!o.range_valid || !e->cruise_valid) {
        /* No Range to offer a fraction of, or no idea what the rider is doing.
         * Hidden stays hidden; shown starts counting toward hidden. */
        flipping = e->advice_shown;
    } else if (e->advice_shown) {
        flipping = !advice_holds(&e->advice_curve, e->cruise_kmh, o.range_km,
                                 o.usable_frac, e->advice_speed_kmh, &a);
    } else {
        flipping = wf_est_advice_pick(&e->advice_curve, e->cruise_kmh,
                                      o.range_km, o.usable_frac, &a);
    }

    /* Saturating, because these only ever have to outgrow two constants and a
     * ride longer than a month of milliseconds must not wrap one of them back
     * under a threshold.
     *
     * The overflow test only applies to a counter that grew. A frame the
     * condition did not hold on RESETS advice_hold_ms, and a reset is a
     * decrease by construction - testing it for overflow would read the reset
     * as a wrap and saturate the timer instead of clearing it, which is the
     * one failure that would disable the arming outright: a saturated
     * advice_hold_ms is permanently past WF_EST_ADVICE_ARM_MS, so the advice
     * would appear on the first agreeing frame for the rest of the ride.
     * advice_state_ms needs no such guard - it only ever grows. */
    if (flipping) {
        uint32_t hold = e->advice_hold_ms + dt_ms;
        e->advice_hold_ms = hold < e->advice_hold_ms ? 0xffffffffu : hold;
    } else {
        e->advice_hold_ms = 0;
    }
    uint32_t age = e->advice_state_ms + dt_ms;
    e->advice_state_ms = age < e->advice_state_ms ? 0xffffffffu : age;

    if (!flipping) {
        return;
    }
    if (!e->advice_shown) {
        /* Ten seconds of continuous agreement before anything appears. */
        if (e->advice_hold_ms >= WF_EST_ADVICE_ARM_MS) {
            e->advice_shown     = true;
            e->advice_speed_kmh = a.speed_kmh;
            e->advice_hold_ms   = 0;
            e->advice_state_ms  = 0;
        }
        return;
    }
    /* And thirty seconds on the screen before it may leave, whatever happens
     * in between. A rider who glances down has to find the same screen they
     * looked away from. */
    if (e->advice_hold_ms >= WF_EST_ADVICE_ARM_MS &&
        e->advice_state_ms >= WF_EST_ADVICE_DWELL_MS) {
        e->advice_shown     = false;
        e->advice_speed_kmh = 0.0;
        e->advice_hold_ms   = 0;
        e->advice_state_ms  = 0;
    }
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
     * two can never disagree about how far the bike has gone.
     *
     * The bucket is cut here, on the metres, and the energy that goes with it
     * is whatever the power block had booked by this instant. Those two
     * integrals are sampled on different frame types, so the cut lands up to
     * one frame interval apart in them - a fifth of a second, one part in five
     * hundred of a window. Once the ring is turning every bucket carries the
     * same offset and it cancels; on the very first window it does not, and
     * that is where the Range figure's one legitimate discontinuity comes
     * from. See the Range section of wfest.h: the handover from the all-time
     * average to the window is a change of source, not a step in one figure,
     * and moving this cut only changes which frame order it favours. */
    e->alltime_m   += step;
    e->cons_part_m += step;
    if (e->cons_part_m >= WF_EST_CONS_BUCKET_M) {
        cons_close_bucket(e);
    }

    /* Cruise: the speed the rider is holding, as opposed to the speed they are
     * doing this frame. Same first-order shape and the same tau as the load
     * average behind Sag, and here for the same reason - a suggestion that
     * changed every time a throttle moved would be a suggestion nobody could
     * act on. Acquired outright on the first road speed, because there is
     * nothing to preserve and a rider should not have to wait a minute for the
     * average to climb off zero.
     *
     * A frame with no road speed in it - before the wheel geometry has arrived
     * - moves neither this nor distance: an unknown speed is not a slow one. */
    if (live->speed_valid) {
        double kmh = (double)live->cur_speed_kmh;
        if (!e->cruise_valid) {
            e->cruise_kmh   = kmh;
            e->cruise_valid = true;
        } else {
            double k = dt_s / WF_EST_ADVICE_SPEED_TAU_S;
            if (k > 1.0) {
                k = 1.0;
            }
            e->cruise_kmh += (kmh - e->cruise_kmh) * k;
        }
    }

    /* And the one decision this module makes about what to put on the screen
     * rather than what to compute. Last, so it sees this frame's metres, this
     * frame's cruise and this frame's closed bucket. */
    advice_step(e, dt_ms);
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

    /* The Pack under load: a step measured against the previous sample, and
     * the load average the Limp Point is moved by. Both before the integration
     * and independent of it - a sample that starts the clock rather than
     * closing an interval is still a voltage and a current the Pack showed. */
    sag_step(e, t_ms, volts, amps);

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

    /* The Cell block first, and independently of the State of Charge below:
     * they are different registers with different failure modes, and an
     * imbalance is still worth knowing about from a response whose State of
     * Charge did not survive its own plausibility check. */
    cell_fold(e, bms);

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

    /* Sag, and the Limp Point it moves. The estimate is faded in over its
     * first WF_EST_IR_CONFIDENT_SAMPLES accepted steps so that the first one
     * of a Monitor's life - which arrives mid-launch, because that is what
     * makes it acceptable - does not take several kilometres off the Range in
     * a single frame. A restored estimate comes back at full weight and so at
     * full confidence.
     *
     * The load is the averaged one and never the last frame's, which is the
     * whole of why this cannot make Range twitch; see load_step(). */
    if (e->ir_weight > 0) {
        uint32_t w = e->ir_weight < WF_EST_IR_CONFIDENT_SAMPLES
                         ? e->ir_weight
                         : WF_EST_IR_CONFIDENT_SAMPLES;
        double conf = (double)w / (double)WF_EST_IR_CONFIDENT_SAMPLES;
        out->ir_valid = true;
        out->ir_ohm   = e->ir_ohm;
        out->sag_v    = e->ir_ohm * conf * e->load_a;
        if (out->sag_v < 0.0) {
            out->sag_v = 0.0;
        }
    }
    out->ir_weight    = e->ir_weight;
    out->ir_steps     = e->ir_steps;
    out->ir_rejected  = e->ir_rejected;
    out->ir_soc_pct   = e->ir_soc_pct;
    out->load_a       = e->load_a;
    /* The Controller's own line, and the Cells do not move it: it cuts on Pack
     * voltage and has never heard of a Cell. What the weakest Cell moves is
     * when the *Pack* gets there, which is `cell_band_v` below and a different
     * mechanism - the BMS's own per-Cell protection - reached first. */
    out->limp_point_v = WF_EST_LIMP_POINT_V + out->sag_v;

    /* The weakest Cell. Read time, exactly as Sag is, and for the same reason:
     * the integration and the Anchor count down to the fixed 84.0 V line, so
     * no accumulator is ever re-based because a Cell moved and a change to
     * this model cannot corrupt a Coulomb Count.
     *
     * With no plausible Cell block ever seen, every figure here stays exactly
     * zero and Range is the Pack-average estimate untouched - the same "no
     * invented number" rule the Internal Resistance follows, and what makes a
     * Monitor that has never heard from the BMS's Cell registers behave
     * exactly as it did before this existed. */
    if (e->cell_weight > 0) {
        out->cell_valid    = true;
        out->cell_min_mv   = e->cell_min_mv;
        out->cell_avg_mv   = e->cell_avg_mv;
        out->cell_spread_v = e->cell_spread_v;
        out->cell_band_v   = wf_est_cell_band_v(e->cell_spread_v);
        /* Binding and costing a watt-hour are the same condition, which is
         * what lets main/ui.c mark the row exactly when the clamp is what is
         * holding the number down. */
        out->cell_clamped  = out->cell_band_v > 0.0;
        /* The warning, and it is a ratio rather than a voltage: the gap below
         * the lowest Cell against the fan-out of the 27 above it. Both are
         * multiplied by the local steepness of the discharge curve, so the
         * steepness - the whole of why divergence widens as the Pack empties -
         * cancels, and a healthy Pack at low charge reads what it read at high
         * charge. The clamp has to be binding as well, so a Pack whose Cells
         * sit within a few millivolts of each other cannot raise an alarm
         * because one of them is a millivolt lower than the rest. */
        out->cell_diverged = out->cell_clamped &&
                             e->cell_gap_v >=
                                 WF_EST_CELL_OUTLIER_K * e->cell_body_v;
    }
    out->cell_samples  = e->cell_samples;
    out->cell_rejected = e->cell_rejected;

    /* Clamped at zero for the rider. Past the Limp Point there is no model
     * here worth showing a number from - issue #8 measures what actually
     * happens down there - and "0 Wh" is the honest thing to say meanwhile.
     * The unclamped value stays in the state for the harness to look at.
     *
     * The two reserves come off here and nowhere else, and they come off as
     * one band rather than as two subtractions: the band carries the mean
     * voltage its charge would have been delivered at, so it is nonlinear in
     * its own width and reserve(a) + reserve(b) is not reserve(a + b). What
     * the rider has left is the integrated figure minus the band between the
     * resting Limp Point and the point their throttle and their worst Cell
     * between them have actually put it at.
     *
     * `remaining_pack_wh` is the same figure with the Cell band left out - the
     * Pack-average estimate - so that "Range takes the lower of the two" is
     * readable off the pair rather than asserted in prose. The Cell band is
     * never negative, so the subtraction *is* the minimum. */
    double sag_reserve   = wf_est_sag_reserve_wh(out->sag_v);
    double total_reserve = wf_est_sag_reserve_wh(out->sag_v + out->cell_band_v);
    out->cell_reserve_wh = total_reserve - sag_reserve;

    double pack_rem = e->remaining_wh - sag_reserve;
    out->remaining_pack_wh = pack_rem > 0.0 ? pack_rem : 0.0;

    double rem = e->remaining_wh - total_reserve;
    out->remaining_wh = rem > 0.0 ? rem : 0.0;

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

    /* Range, and it is one division of two numbers this function has already
     * produced. Nothing is recomputed for it and nothing is filtered: the
     * steadiness is Consumption's ring, upstream, and the crawl below the Limp
     * Point is already outside `remaining_wh` and is not subtracted twice. See
     * the Range section of wfest.h for why that is the whole of it.
     *
     * Three things have to hold before the division is allowed, and the third
     * is the one that matters:
     *
     *   there is a Remaining Energy at all - `valid`, so a Monitor that has
     *   never seen the BMS shows a dash rather than a confident zero;
     *   there is a Consumption at all, from either source;
     *   and that Consumption is at least WF_EST_RANGE_MIN_CONS_WH_PER_KM.
     *
     * The last comparison is written as `>=` against the floor rather than as
     * `!= 0`, which rejects a negative Consumption, a Consumption of nearly
     * nothing and a NaN in one, and is why no zero can reach the denominator.
     * `remaining_wh` is the clamped figure, so Range is zero at the Limp Point
     * and never negative below it. */
    if (out->valid && out->consumption_valid &&
        out->consumption_wh_per_km >= WF_EST_RANGE_MIN_CONS_WH_PER_KM) {
        out->range_valid = true;
        out->range_km    = out->remaining_wh / out->consumption_wh_per_km;
    }

    /* How full the Pack is, in the units the advice's first condition is
     * written in: what is left over what a full one holds above the Limp
     * Point. The denominator is a constant of the Pack model; the numerator is
     * the clamped figure, so Sag and the weakest Cell are already in it. */
    double full_wh = wf_est_energy_above_limp_wh(100.0);
    if (full_wh > 0.0) {
        out->usable_frac = out->remaining_wh / full_wh;
    }

    /* The advice. Whether it is on the screen at all was decided on the motion
     * block, by advice_step(), because that decision needs a clock and this
     * function has none; what is done here is the arithmetic that goes with a
     * speed already chosen.
     *
     * Recomputed rather than stored, so the kilometres fall with the Pack
     * while the speed stays put - and so that a counterfactual which has
     * stopped being computable takes the row off the screen this frame rather
     * than at the end of the dwell. The latch can only keep a suggestion up;
     * it can never conjure one. */
    out->cruise_valid = e->cruise_valid;
    out->cruise_kmh   = e->cruise_kmh;
    if (e->advice_shown && out->range_valid) {
        wf_est_advice_t a;
        if (wf_est_advice_at(&e->advice_curve, e->cruise_kmh, out->range_km,
                             e->advice_speed_kmh, &a)) {
            out->advice_valid     = true;
            out->advice_speed_kmh = a.speed_kmh;
            out->advice_range_km  = a.range_km;
            out->advice_gain_km   = a.gain_km;
        }
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

    /* Internal Resistance, with the weight it was averaged over, which is what
     * makes it improve across rides rather than start again: the next ride
     * folds its steps into this mean at this weight. The load average behind
     * Sag is deliberately not here - it is riding, not a Pack. */
    out->ir_ohm    = (float)e->ir_ohm;
    out->ir_weight = (uint16_t)e->ir_weight;
}

/* ------------------------------------------- Consumption at a chosen speed */

double wf_est_fit_eval(double a, double b, double c, double speed_kmh)
{
    /* Horner, and not because it is faster on three terms - because it is one
     * multiply-add chain rather than two independent products summed, which
     * is one fewer place for -ffp-contract to have been the difference
     * between the Monitor's answer and the harness's. The fit is a straight
     * a + b*v + c*v^2 either way. */
    return a + speed_kmh * (b + speed_kmh * c);
}

void wf_est_curve_default(wf_est_curve_t *out)
{
    if (out == NULL) {
        return;
    }
    out->fitted              = WF_FIT_FITTED != 0;
    out->a_wh_per_km         = WF_FIT_A_WH_PER_KM;
    out->b_wh_per_km_per_kmh = WF_FIT_B_WH_PER_KM_PER_KMH;
    out->c_wh_per_km_per_kmh2 = WF_FIT_C_WH_PER_KM_PER_KMH2;
    out->speed_min_kmh       = WF_FIT_SPEED_MIN_KMH;
    out->speed_max_kmh       = WF_FIT_SPEED_MAX_KMH;
}

void wf_est_curve_at(const wf_est_curve_t *curve, double speed_kmh,
                     wf_est_fit_t *out)
{
    if (out == NULL) {
        return;
    }
    out->fitted        = false;
    out->extrapolated  = false;
    out->wh_per_km     = 0.0;
    out->speed_min_kmh = curve != NULL ? curve->speed_min_kmh : 0.0;
    out->speed_max_kmh = curve != NULL ? curve->speed_max_kmh : 0.0;

    if (curve == NULL || !curve->fitted) {
        /* No fit at all. Nothing is produced at any speed, and the range
         * stays the pair of zeroes wf_fit.h holds - so a caller that checks
         * the range instead of the flag also finds no speed inside it. */
        return;
    }
    out->fitted = true;

    /* Written the way round that rejects a NaN as outside the range, since a
     * NaN compares false against both bounds and the honest answer to "is
     * this speed supported" for a speed that is not a number is no. */
    if (!(speed_kmh >= curve->speed_min_kmh &&
          speed_kmh <= curve->speed_max_kmh)) {
        out->extrapolated = true;
    }
    out->wh_per_km = wf_est_fit_eval(curve->a_wh_per_km,
                                     curve->b_wh_per_km_per_kmh,
                                     curve->c_wh_per_km_per_kmh2, speed_kmh);
}

void wf_est_consumption_at_speed(double speed_kmh, wf_est_fit_t *out)
{
    /* The committed constants, and nothing else: the Monitor's own answer is
     * these two composed, so a host test driving wf_est_curve_at() with
     * coefficients of its own is exercising the very same evaluator. */
    wf_est_curve_t curve;
    wf_est_curve_default(&curve);
    wf_est_curve_at(&curve, speed_kmh, out);
}

/* ---------------------------------------------------------------- the advice */

bool wf_est_advice_at(const wf_est_curve_t *curve, double cruise_kmh,
                      double range_km, double speed_kmh, wf_est_advice_t *out)
{
    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (curve == NULL || !curve->fitted) {
        return false;
    }
    /* A fraction of no Range is no Range. Written the way round that rejects a
     * NaN, and it also keeps the division below off a zero denominator. */
    if (!(range_km > 0.0)) {
        return false;
    }

    wf_est_fit_t now, slower;
    wf_est_curve_at(curve, cruise_kmh, &now);
    wf_est_curve_at(curve, speed_kmh, &slower);

    /* Both ends, and this is the acceptance criterion about extrapolation as
     * one comparison. The curve is a quadratic fitted over a span of speeds; a
     * kilometre either side of that span it is a guess, and a guess is not
     * something to tell a rider who is deciding whether they can get home. */
    if (now.extrapolated || slower.extrapolated) {
        return false;
    }
    if (!(now.wh_per_km > 0.0) || !(slower.wh_per_km > 0.0)) {
        return false;
    }
    /* Nothing to offer if the slower speed is not actually cheaper - a flat
     * curve, or a rider already below the bottom of the bathtub. */
    if (!(slower.wh_per_km < now.wh_per_km)) {
        return false;
    }

    out->speed_kmh = speed_kmh;
    out->range_km  = range_km * (now.wh_per_km / slower.wh_per_km);
    out->gain_km   = out->range_km - range_km;
    out->gain_frac = out->gain_km / range_km;
    return true;
}

bool wf_est_advice_pick(const wf_est_curve_t *curve, double cruise_kmh,
                        double range_km, double usable_frac,
                        wf_est_advice_t *out)
{
    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (curve == NULL || !curve->fitted) {
        return false;
    }

    /* The first condition: the Pack has to be low enough for any of this to
     * matter. Written the way round that rejects a NaN, so an estimate that
     * has gone wrong is silent rather than loud. */
    if (!(usable_frac <= WF_EST_ADVICE_LOW_FRAC)) {
        return false;
    }
    /* A cruise that is not a plausible road speed is not something to reason
     * from, and the bound is also what keeps the cast below defined. */
    if (!(cruise_kmh > 0.0 && cruise_kmh < 1000.0)) {
        return false;
    }

    /* The ladder: round numbers, walked down from the cruise. The first rung
     * that clears both gain thresholds wins, because the least sacrifice that
     * changes the outcome is the one a rider will take. */
    double top = (double)(long)(cruise_kmh / WF_EST_ADVICE_STEP_KMH) *
                 WF_EST_ADVICE_STEP_KMH;
    for (int i = 0; i < WF_EST_ADVICE_LADDER_MAX; i++) {
        double v = top - (double)i * WF_EST_ADVICE_STEP_KMH;

        if (!advice_holdable(cruise_kmh, v)) {
            /* Too close to the cruise to be worth asking for: keep walking
             * down. Below one of the floors: nothing further down clears them
             * either, so stop. */
            if (v > cruise_kmh - WF_EST_ADVICE_STEP_KMH) {
                continue;
            }
            break;
        }
        wf_est_advice_t a;
        if (!wf_est_advice_at(curve, cruise_kmh, range_km, v, &a)) {
            /* Off the bottom of the fit, or no gain to be had at all. Both are
             * monotone in the same direction, so there is nothing below this
             * worth trying. */
            break;
        }
        /* The second condition, and it is two tests because "meaningful" is
         * two things: enough of the Range they have to change the decision,
         * and enough kilometres for the screen to be able to show it. */
        if (a.gain_frac >= WF_EST_ADVICE_GAIN_FRAC &&
            a.gain_km >= WF_EST_ADVICE_GAIN_KM) {
            *out = a;
            return true;
        }
    }
    return false;
}

void wf_est_set_curve(wf_est_t *e, const wf_est_curve_t *curve)
{
    if (e == NULL || curve == NULL) {
        return;
    }
    e->advice_curve = *curve;
}
