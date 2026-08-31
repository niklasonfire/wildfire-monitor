/*
 * replay - runs a recorded Capture through the Monitor's decoders, off the
 * bike.
 *
 * Everything the Monitor decodes it decodes with main/wfdecode, and nothing in
 * there needs a radio, a board or a rider: hand it the bytes and it produces
 * the same numbers it would have produced on the handlebars. So a checked-in
 * Capture plus this file is a regression test for every claim in
 * docs/field-table.md, run in a second instead of by eye once, on a ride.
 *
 * For every .wfl in the fixture directory it replays the whole file, collects
 * a table of measurements and compares them against the .expect file sitting
 * next to it. Invariants that hold for any Capture (every Controller frame's
 * checksum verifies, every BMS response decodes) are checked here; everything
 * that is a fact about one particular ride lives in that ride's .expect file.
 * Adding a Capture is therefore two files and no change to this program.
 *
 *   ./replay [fixture-dir]           default: tests/fixtures
 *   ./replay --fields <capture.wfl>  every generated field of every record
 *
 * The second mode is the other half of ADR-0002. It prints what the generated
 * C decoder makes of every record, in a canonical form the generated Python
 * decoder prints identically, so tests/test_field_table.py can assert the
 * two languages agree byte for byte rather than by inspection.
 *
 * Every Capture also goes through main/wfest, the estimation seam, and the
 * whole Remaining Energy curve is hashed. Each fixture is replayed twice and
 * the two hashes have to match bit for bit: that is the acceptance criterion
 * "replaying a recorded Capture produces the identical Remaining Energy curve
 * every time", asserted rather than intended. It is a hash of every point and
 * not of the final value, because a curve can arrive at the right answer by
 * two different routes and only one of them is deterministic.
 */
#include <dirent.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wfdecode.h"
#include "wfest.h"
#include "wfl_read.h"

#define MAX_FIXTURE_BYTES (16 * 1024 * 1024)
#define MAX_METRICS       96

/* ------------------------------------------------------- measured values */

/* One number the replay produced. Everything is carried as a double: the
 * .expect file compares against text either way, and an integer count up to
 * 2^53 survives the trip exactly. */
typedef struct {
    const char *name;
    double      value;
} metric_t;

typedef struct {
    metric_t m[MAX_METRICS];
    int      n;
} metrics_t;

static void put(metrics_t *ms, const char *name, double value)
{
    if (ms->n >= MAX_METRICS) {
        fprintf(stderr, "too many metrics, raise MAX_METRICS\n");
        exit(2);
    }
    ms->m[ms->n].name = name;
    ms->m[ms->n].value = value;
    ms->n++;
}

static const metric_t *find(const metrics_t *ms, const char *name)
{
    for (int i = 0; i < ms->n; i++) {
        if (strcmp(ms->m[i].name, name) == 0) {
            return &ms->m[i];
        }
    }
    return NULL;
}

/* --------------------------------------------------------------- the run */

typedef struct {
    /* record counts */
    long ctrl_records, bms_records, event_records, telem_records, imu_records;
    long marker_records, other_records;
    /* decoding */
    long ctrl_frames_ok, ctrl_frames_bad;
    long bms_responses_ok, bms_responses_bad;
    /* Controller live state at the end of the ride, plus the extremes */
    wf_ctrl_live_t live;
    unsigned rpm_max;
    double   speed_kmh_max;
    long     speed_samples;
    /* The power block: pack voltage and line current, eight of the 55 frame
     * types, and the only Controller telemetry fast enough for ADR-0003's
     * split of authority to mean anything. */
    long     power_frames;
    double   ctrl_pack_v_min, ctrl_pack_v_max;
    double   ctrl_current_a_min, ctrl_current_a_max;
    /* BMS */
    double   pack_v_min, pack_v_max;
    double   soc_pct_min, soc_pct_max;
    double   current_a_min, current_a_max;
    unsigned cell_mv_min, cell_mv_max;
    unsigned cell_count;
    bool     cell_count_stable;
    /* Cross-checks between registers that must describe the same Pack. Each
     * counts the responses where they disagree by more than the BMS's own
     * sampling jitter, so the assertion is "0", not "close enough". */
    long     cell_sum_mismatch;      /* reg 40 vs the sum of reg 0-27 */
    long     cell_extreme_mismatch;  /* reg 43/44 vs the extremes of reg 0-27 */
    long     cell_count_disagree;    /* reg 49 vs reg 51 */
    /* Rated Capacity, which is not a register: remaining_ah divided by the
     * State of Charge. Collected per response so that a ride watching the Pack
     * empty says whether the derivation holds as the divisor shrinks - which
     * is where it is weakest, and why reading a nameplate register instead is
     * still the open half of issue #3. */
    double   remaining_ah_min, remaining_ah_max;
    double   rated_ah_min, rated_ah_max;
    long     rated_ah_samples;       /* responses the derivation was safe on */
    /* The two redundant registers, checked as identities rather than values.
     * Both must be exactly derivable from registers decoded elsewhere, and a
     * single failure means the map has slipped, not that the BMS rounded. */
    unsigned cell_delta_mv_max;
    unsigned power_w_max;
    long     cell_delta_redundant_mismatch;  /* reg 56 vs reg 43 - reg 44 */
    long     power_redundant_mismatch;       /* reg 57 vs reg 40 x |reg 41| */
    /* The one cross-device check there is: the Controller and the BMS measure
     * the same Pack with separate instruments, so the largest gap between
     * their two pack voltages is the strongest evidence either decode is
     * right. Sampled at every BMS response, against whatever the Controller
     * last said. */
    double   pack_v_device_gap_max;
    long     pack_v_device_samples;
    long     dropped_last;
    uint32_t duration_ms;
    /* The estimation seam, main/wfest, fed the same records in the same order
     * the Monitor feeds it live. Nothing here reads a clock: every t_ms comes
     * out of the record it arrived on. */
    wf_est_t est;
    uint64_t est_hash;          /* over the whole Remaining Energy curve */
    long     est_points;
    double   est_wh_first, est_wh_last;
    double   est_wh_min, est_wh_max;
    /* How far the Coulomb Count ever got from its Anchor, in percent of the
     * Pack. The acceptance criterion is that this does not diverge. */
    double   est_soc_gap_max;
    long     est_soc_gap_samples;
    /* Distance, fused from integrated speed and the Odometer. Hashed the same
     * way and for the same reason as the energy curve, and separately from it,
     * because distance becomes valid the moment the Controller speaks and the
     * energy curve waits for the BMS. */
    uint64_t dist_hash;
    long     dist_points;
    double   dist_m_last, dist_odo_last;
    double   dist_step_back_max;   /* largest backwards move, must stay 0 */
    /* Range. Sampled wherever there is one, which on cap0007 is nowhere: that
     * ride covers 16.7 m, produces no Consumption and therefore no Range, and
     * cap0007.expect pins exactly that. The behavioural assertions are in
     * tests/host/unit.c on synthesised riding, for the same reason
     * Consumption's are. What is tracked here is what a Capture can honestly
     * say: how far the figure moved and which way.
     *
     * `est_range_step_up_max` is the one to watch when a longer ride lands.
     * On a ride with no regeneration and no change of Consumption source it
     * is the monotonicity criterion as a number, and it belongs in that ride's
     * .expect file - which is where a per-ride fact goes. It is deliberately
     * not an invariant here: a Range that rises when the rider slows down is
     * the figure working, and an invariant would forbid it on every Capture. */
    /* Internal Resistance and the Sag it puts on the Limp Point. cap0007
     * cannot produce either - its largest load step is under half of
     * WF_EST_IR_MIN_DI_A - and "this ride measured no Pack" is exactly the
     * fact worth pinning on it, so these are collected unconditionally. On a
     * ride with hard launches in it they are the per-ride record of what the
     * Pack looked like that day, which is what issue #21 will compare across
     * months. */
    double   sag_v_max;
    long     range_points;
    double   range_km_first, range_km_last;
    double   range_km_prev;
    double   range_step_up_max;    /* largest upward move over the ride */
    /* Records walked, and whether a power cycle was simulated part way through
     * this run. See power_cycle() and the criterion it exists for. */
    long     records;
    bool     cycled;
} run_t;

/* FNV-1a over the raw bytes of every point on the curve. The comparison is
 * between two replays of the same file in the same binary, so hashing the bit
 * pattern is exactly the right strictness: no tolerance, because a tolerance
 * is the thing this criterion is about not needing. */
static void curve_point(run_t *r, double wh)
{
    uint8_t bytes[sizeof(double)];
    memcpy(bytes, &wh, sizeof(bytes));
    for (size_t i = 0; i < sizeof(bytes); i++) {
        r->est_hash ^= bytes[i];
        r->est_hash *= 0x100000001b3ull;
    }
    if (r->est_points == 0) {
        r->est_wh_first = wh;
        r->est_wh_min = r->est_wh_max = wh;
    }
    if (wh < r->est_wh_min) r->est_wh_min = wh;
    if (wh > r->est_wh_max) r->est_wh_max = wh;
    r->est_wh_last = wh;
    r->est_points++;
}

/* The same, over the distance curve. Kept apart from the energy one so that a
 * change to either shows up as itself. */
static void dist_point(run_t *r, double m)
{
    uint8_t bytes[sizeof(double)];
    memcpy(bytes, &m, sizeof(bytes));
    for (size_t i = 0; i < sizeof(bytes); i++) {
        r->dist_hash ^= bytes[i];
        r->dist_hash *= 0x100000001b3ull;
    }
    /* Distance is monotone by construction - the Odometer may slow it to a
     * standstill and may not reverse it - so this is the assertion, sampled at
     * every point the rider could have looked at the screen. */
    double back = r->dist_m_last - m;
    if (r->dist_points > 0 && back > r->dist_step_back_max) {
        r->dist_step_back_max = back;
    }
    r->dist_m_last = m;
    r->dist_points++;
}

/* Sampled after every record the estimator was fed, so the curve is what the
 * live screen would have shown at each of those instants. */
static void est_sample(run_t *r)
{
    wf_est_out_t o;
    wf_est_get(&r->est, &o);
    if (o.sag_v > r->sag_v_max) {
        r->sag_v_max = o.sag_v;
    }
    if (o.distance_valid) {
        dist_point(r, o.distance_m);
        r->dist_odo_last = o.odo_distance_m;
    }
    if (o.range_valid) {
        if (r->range_points == 0) {
            r->range_km_first = o.range_km;
            r->range_km_prev  = o.range_km;
        }
        double up = o.range_km - r->range_km_prev;
        if (up > r->range_step_up_max) {
            r->range_step_up_max = up;
        }
        r->range_km_prev = o.range_km;
        r->range_km_last = o.range_km;
        r->range_points++;
    }
    if (!o.valid) {
        return;
    }
    curve_point(r, o.remaining_wh);
    if (o.anchored && o.anchor_samples > 0) {
        double gap = o.soc_pct - o.anchor_soc_pct;
        if (gap < 0.0) {
            gap = -gap;
        }
        if (gap > r->est_soc_gap_max) {
            r->est_soc_gap_max = gap;
        }
        r->est_soc_gap_samples++;
    }
}

/* ------------------------------------------------- the synthesised wrap */

/* `odometer_raw` reads a constant 14 through the whole of cap0007, so no
 * Capture we hold crosses the Odometer's u16 wrap and no Capture we are likely
 * to take soon will either - it happens once every 6553 km. The acceptance
 * criterion is still that a wrap produces no discontinuity, so the wrap is
 * synthesised into a real ride: every Odometer frame's count is rewritten to a
 * ramp and the frame re-checksummed, so it goes through the same parser, the
 * same decoder and the same estimator that a real frame does, over the real
 * ride's timing and the real ride's speeds.
 *
 * The assertion is then the strongest one available. The same ramp is run
 * twice, once from a count well clear of the wrap and once from six counts
 * short of it, and the two distance curves have to be identical bit for bit -
 * not close. The Anchor is built from wrap-safe count differences and never
 * from an absolute reading, so a wrap is not an event it can notice. */
typedef struct {
    bool     on;
    uint16_t start;      /* the count the ramp begins at */
} odo_synth_t;

/* One Field Table entry by name. Read rather than repeated: the Odometer's
 * frame type and its payload offset are declared in field-table.json, and a
 * test that hard-codes them is a second declaration to drift from the first. */
static const wf_field_t *ctrl_field(const char *name)
{
    for (int i = 0; i < WF_CTRL_FIELD_COUNT; i++) {
        if (strcmp(wf_ctrl_field_table[i].name, name) == 0) {
            return &wf_ctrl_field_table[i];
        }
    }
    fprintf(stderr, "the Field Table has no Controller field `%s`\n", name);
    exit(2);
}

/* Rewrites one Odometer frame's count and fixes its checksum, in scratch.
 * Returns the bytes to parse: the rewritten copy, or the original untouched. */
static const uint8_t *synth_odo(const odo_synth_t *s, unsigned tick,
                                const uint8_t *data, uint32_t len,
                                uint8_t scratch[WF_CTRL_FRAME_LEN])
{
    static const wf_field_t *odo;
    if (odo == NULL) {
        odo = ctrl_field("odometer_raw");
    }
    if (!s->on || len != WF_CTRL_FRAME_LEN || odo->frame_type_count != 1 ||
        data[1] != odo->frame_types[0]) {
        return data;
    }
    memcpy(scratch, data, WF_CTRL_FRAME_LEN);
    uint16_t counts = (uint16_t)(s->start + tick);
    scratch[2 + odo->key]     = (uint8_t)(counts & 0xff);
    scratch[2 + odo->key + 1] = (uint8_t)(counts >> 8);
    uint16_t crc = wf_crc16(scratch, WF_CTRL_FRAME_LEN - 2, WF_CTRL_CRC_INIT);
    scratch[14] = (uint8_t)(crc & 0xff);
    scratch[15] = (uint8_t)(crc >> 8);
    return scratch;
}

static void run_init(run_t *r)
{
    memset(r, 0, sizeof(*r));
    r->pack_v_min = r->soc_pct_min = r->current_a_min = 1e9;
    r->pack_v_max = r->soc_pct_max = r->current_a_max = -1e9;
    r->ctrl_pack_v_min = r->ctrl_current_a_min = 1e9;
    r->ctrl_pack_v_max = r->ctrl_current_a_max = -1e9;
    r->cell_mv_min = 0xffffu;
    r->remaining_ah_min = r->rated_ah_min = 1e9;
    r->remaining_ah_max = r->rated_ah_max = -1e9;
    r->cell_count_stable = true;
    r->est_hash = r->dist_hash = 0xcbf29ce484222325ull;   /* FNV-1a basis */
    /* Cold: a fixture carries no persisted state, so every replay of it starts
     * from the same nothing. That is half of why replays agree. */
    wf_est_init(&r->est, NULL);
}

/* ------------------------------------------------- the simulated power cycle
 *
 * The Monitor loses power mid-ride - a flat cell, a knocked switch, a coffee
 * stop long enough to turn it off - and comes back. All that is, from the
 * estimator's point of view, is a save through NVS's bytes and a fresh
 * wf_est_init() from what comes back, which is exactly what this does. It is
 * driven into the middle of a real replay so that the state crossing the break
 * is state a real ride produced, rather than something a test composed.
 *
 * Only the estimator is cycled. The decoded Controller state stays, standing
 * in for a link that came back up carrying the same readings; cycling that too
 * would be testing the decoder's re-acquisition, which is a different
 * criterion and has its own test. */
static void power_cycle(run_t *r)
{
    wf_est_persist_t saved, back;
    uint8_t blob[WF_EST_PERSIST_BYTES];

    wf_est_save(&r->est, &saved);
    if (!wf_est_persist_encode(&saved, blob, sizeof(blob)) ||
        !wf_est_persist_decode(blob, sizeof(blob), &back)) {
        fprintf(stderr, "the estimator's own state did not survive its own "
                        "encoding\n");
        exit(2);
    }
    wf_est_init(&r->est, &back);
    r->cycled = true;
}

/* cycle_at is the record index to simulate a power cycle before, or 0 for a
 * run with no break in it. */
static void replay(const uint8_t *buf, size_t len, wflog_hdr_t *hdr, run_t *r,
                   const odo_synth_t *synth, long cycle_at)
{
    wfl_reader_t rd;
    if (!wfl_open(&rd, buf, len, hdr)) {
        fprintf(stderr, "not a Capture, or too short to hold a header\n");
        exit(2);
    }

    run_init(r);
    unsigned odo_tick = 0;
    uint8_t scratch[WF_CTRL_FRAME_LEN];

    wfl_rec_t rec;
    while (wfl_next(&rd, &rec)) {
        if (cycle_at > 0 && r->records == cycle_at) {
            power_cycle(r);
        }
        r->records++;
        r->duration_ms = rec.t_ms;

        switch (rec.type) {
        case WFREC_MCU: {
            r->ctrl_records++;
            const uint8_t *data = rec.data;
            if (synth->on) {
                const uint8_t *rewritten = synth_odo(synth, odo_tick, rec.data,
                                                     rec.len, scratch);
                if (rewritten != rec.data) {
                    odo_tick++;
                    data = rewritten;
                }
            }
            wf_ctrl_frame_t f;
            if (!wf_ctrl_frame_parse(data, rec.len, &f)) {
                r->ctrl_frames_bad++;
                break;
            }
            r->ctrl_frames_ok++;
            wf_ctrl_apply(&r->live, &f);
            bool motion_frame = wf_ctrl_type_in(wf_ctrl_type_motion,
                                                WF_CTRL_TYPE_MOTION_COUNT,
                                                f.type);
            if (r->live.motion_valid && r->live.cur_rpm > r->rpm_max) {
                r->rpm_max = r->live.cur_rpm;
            }
            if (r->live.speed_valid && motion_frame) {
                r->speed_samples++;
                if (r->live.cur_speed_kmh > r->speed_kmh_max) {
                    r->speed_kmh_max = r->live.cur_speed_kmh;
                }
            }
            /* Counted per frame rather than per type: all eight carry the
             * same two fields, and what matters downstream is how often the
             * pair arrives, not which of the eight brought it. */
            if (r->live.power_valid &&
                wf_ctrl_type_in(wf_ctrl_type_power, WF_CTRL_TYPE_POWER_COUNT,
                                f.type)) {
                r->power_frames++;
                if (r->live.pack_v < r->ctrl_pack_v_min) {
                    r->ctrl_pack_v_min = r->live.pack_v;
                }
                if (r->live.pack_v > r->ctrl_pack_v_max) {
                    r->ctrl_pack_v_max = r->live.pack_v;
                }
                if (r->live.line_current_a < r->ctrl_current_a_min) {
                    r->ctrl_current_a_min = r->live.line_current_a;
                }
                if (r->live.line_current_a > r->ctrl_current_a_max) {
                    r->ctrl_current_a_max = r->live.line_current_a;
                }
            }
            /* Exactly what main/capture.c does on the bike, in the same place:
             * decode, then hand the decoded state and the record's own
             * timestamp to the estimator. */
            wf_est_feed_ctrl(&r->est, rec.t_ms, f.type, &r->live);
            est_sample(r);
            break;
        }

        case WFREC_BMS: {
            r->bms_records++;
            wf_bms_t b;
            if (!wf_bms_decode(rec.data, rec.len, &b)) {
                r->bms_responses_bad++;
                break;
            }
            r->bms_responses_ok++;
            if (b.pack_v < r->pack_v_min) r->pack_v_min = b.pack_v;
            if (b.pack_v > r->pack_v_max) r->pack_v_max = b.pack_v;
            if (b.soc_pct < r->soc_pct_min) r->soc_pct_min = b.soc_pct;
            if (b.soc_pct > r->soc_pct_max) r->soc_pct_max = b.soc_pct;
            if (b.current_a < r->current_a_min) r->current_a_min = b.current_a;
            if (b.current_a > r->current_a_max) r->current_a_max = b.current_a;
            unsigned lo = 0xffffu, hi = 0;
            unsigned sum_mv = 0;
            for (unsigned i = 0; i < b.cell_count && i < WF_BMS_MAX_CELLS; i++) {
                if (b.cell_mv[i] < lo) lo = b.cell_mv[i];
                if (b.cell_mv[i] > hi) hi = b.cell_mv[i];
                sum_mv += b.cell_mv[i];
            }
            if (lo < r->cell_mv_min) r->cell_mv_min = lo;
            if (hi > r->cell_mv_max) r->cell_mv_max = hi;

            /* The Cells, the Pack voltage and the two extreme registers are
             * three separate readings of one Pack. They are sampled a moment
             * apart, so they are allowed to differ by a little and by nothing
             * more: a slipped register map shows up here as a wild number. */
            double from_cells = sum_mv / 1000.0;
            if (from_cells < b.pack_v - 1.0 || from_cells > b.pack_v + 1.0) {
                r->cell_sum_mismatch++;
            }
            if (b.cell_max_mv + 10u < hi || hi + 10u < b.cell_max_mv ||
                b.cell_min_mv + 10u < lo || lo + 10u < b.cell_min_mv) {
                r->cell_extreme_mismatch++;
            }
            if (b.reg[49] != b.reg[51]) {
                r->cell_count_disagree++;
            }
            if (b.remaining_ah < r->remaining_ah_min) {
                r->remaining_ah_min = b.remaining_ah;
            }
            if (b.remaining_ah > r->remaining_ah_max) {
                r->remaining_ah_max = b.remaining_ah;
            }
            /* Rated Capacity, derived rather than read. Guarded on the State
             * of Charge because the derivation divides by it: near empty the
             * divisor is small enough that the BMS's own 0.1 % resolution
             * dominates the answer, which is exactly the weakness a nameplate
             * register would not have. */
            if (b.soc_pct > 5.0f) {
                double rated = b.remaining_ah / (b.soc_pct / 100.0);
                if (rated < r->rated_ah_min) r->rated_ah_min = rated;
                if (rated > r->rated_ah_max) r->rated_ah_max = rated;
                r->rated_ah_samples++;
            }
            /* The two redundant registers. Not "close enough": both identities
             * hold exactly in every response of every Capture we hold, so any
             * mismatch at all means the register map moved under us. */
            if (b.cell_delta_mv > r->cell_delta_mv_max) {
                r->cell_delta_mv_max = b.cell_delta_mv;
            }
            if (b.power_w > r->power_w_max) {
                r->power_w_max = b.power_w;
            }
            if ((int)b.cell_delta_mv != (int)b.cell_max_mv - (int)b.cell_min_mv) {
                r->cell_delta_redundant_mismatch++;
            }
            /* Truncated, not rounded - the same convention avg_cell_mv uses.
             * The half-watt of slack absorbs the last bit of the two floats
             * this multiplies, not a disagreement about the value. */
            double watts = b.pack_v * (b.current_a < 0.0f ? -b.current_a
                                                          : b.current_a);
            double lost = watts - (double)b.power_w;   /* 0 <= lost < 1 */
            if (lost < -0.5 || lost > 1.5) {
                r->power_redundant_mismatch++;
            }
            /* Two instruments, one Pack. The Controller's reading is up to a
             * fifth of a second old here and the BMS averages internally, so
             * they are allowed to differ a little - but only a little, and
             * check_invariants() says how much. */
            if (r->live.power_valid) {
                double gap = r->live.pack_v - b.pack_v;
                if (gap < 0.0) {
                    gap = -gap;
                }
                if (gap > r->pack_v_device_gap_max) {
                    r->pack_v_device_gap_max = gap;
                }
                r->pack_v_device_samples++;
            }
            if (r->bms_responses_ok == 1) {
                r->cell_count = b.cell_count;
            } else if (b.cell_count != r->cell_count) {
                r->cell_count_stable = false;
            }
            /* The Anchor. Per ADR-0003 the estimator takes the State of Charge
             * from here and nothing else - not this device's pack voltage, not
             * its current, and certainly not any energy figure it offers. */
            wf_est_feed_bms(&r->est, rec.t_ms, &b);
            est_sample(r);
            break;
        }

        case WFREC_EVENT:
            r->event_records++;
            if (wfl_is_marker(&rec)) {
                r->marker_records++;
            }
            break;

        case WFREC_TELEM: {
            r->telem_records++;
            wflog_telem_t t;
            if (wfl_telem(&rec, &t)) {
                r->dropped_last = (long)t.dropped;
            }
            break;
        }

        case WFREC_IMU: {
            r->imu_records++;
            wflog_imu_t s;
            if (!wfl_imu(&rec, &s)) {
                fprintf(stderr, "IMU record of %u bytes cannot be read\n", rec.len);
                exit(2);
            }
            break;
        }

        default:
            r->other_records++;
            break;
        }
    }
}

static void collect(const run_t *r, metrics_t *ms)
{
    put(ms, "ctrl_records", (double)r->ctrl_records);
    put(ms, "bms_records", (double)r->bms_records);
    put(ms, "event_records", (double)r->event_records);
    put(ms, "marker_records", (double)r->marker_records);
    put(ms, "telem_records", (double)r->telem_records);
    put(ms, "imu_records", (double)r->imu_records);
    put(ms, "ctrl_frames_ok", (double)r->ctrl_frames_ok);
    put(ms, "ctrl_frames_bad", (double)r->ctrl_frames_bad);
    put(ms, "bms_responses_ok", (double)r->bms_responses_ok);
    put(ms, "bms_responses_bad", (double)r->bms_responses_bad);
    put(ms, "dropped", (double)r->dropped_last);
    put(ms, "duration_ms", (double)r->duration_ms);

    if (r->ctrl_frames_ok > 0) {
        put(ms, "rpm_max", (double)r->rpm_max);
        put(ms, "wheel_radius", (double)r->live.wheel_radius);
        put(ms, "wheel_width", (double)r->live.wheel_width);
        put(ms, "wheel_ratio", (double)r->live.wheel_ratio);
        put(ms, "rate_ratio", (double)r->live.rate_ratio);
        put(ms, "odometer_raw", (double)r->live.odometer_raw);
        put(ms, "engine_temp_last", (double)r->live.engine_temp);
        put(ms, "gear_last", (double)r->live.gear);
    }
    if (r->speed_samples > 0) {
        put(ms, "speed_kmh_max", r->speed_kmh_max);
        /* How many times the ride produced a fresh road speed. One per frame
         * of the motion block, so this is the rate distance is integrated at,
         * pinned as a number: eight of the 55 types, not one. */
        put(ms, "speed_samples", (double)r->speed_samples);
    }
    if (r->power_frames > 0) {
        put(ms, "power_frames", (double)r->power_frames);
        put(ms, "ctrl_pack_v_min", r->ctrl_pack_v_min);
        put(ms, "ctrl_pack_v_max", r->ctrl_pack_v_max);
        put(ms, "ctrl_current_a_min", r->ctrl_current_a_min);
        put(ms, "ctrl_current_a_max", r->ctrl_current_a_max);
    }
    if (r->pack_v_device_samples > 0) {
        put(ms, "pack_v_device_gap_max", r->pack_v_device_gap_max);
    }
    if (r->bms_responses_ok > 0) {
        put(ms, "pack_v_min", r->pack_v_min);
        put(ms, "pack_v_max", r->pack_v_max);
        put(ms, "soc_pct_min", r->soc_pct_min);
        put(ms, "soc_pct_max", r->soc_pct_max);
        put(ms, "current_a_min", r->current_a_min);
        put(ms, "current_a_max", r->current_a_max);
        put(ms, "cell_mv_min", (double)r->cell_mv_min);
        put(ms, "cell_mv_max", (double)r->cell_mv_max);
        put(ms, "cell_count", (double)r->cell_count);
        put(ms, "cell_sum_mismatch", (double)r->cell_sum_mismatch);
        put(ms, "cell_extreme_mismatch", (double)r->cell_extreme_mismatch);
        put(ms, "cell_count_disagree", (double)r->cell_count_disagree);
        put(ms, "remaining_ah_min", r->remaining_ah_min);
        put(ms, "remaining_ah_max", r->remaining_ah_max);
        put(ms, "cell_delta_mv_max", (double)r->cell_delta_mv_max);
        put(ms, "power_w_max", (double)r->power_w_max);
        put(ms, "cell_delta_redundant_mismatch",
            (double)r->cell_delta_redundant_mismatch);
        put(ms, "power_redundant_mismatch", (double)r->power_redundant_mismatch);
    }
    if (r->rated_ah_samples > 0) {
        put(ms, "rated_ah_min", r->rated_ah_min);
        put(ms, "rated_ah_max", r->rated_ah_max);
    }

    if (r->est_points > 0) {
        wf_est_out_t o;
        wf_est_get(&r->est, &o);
        put(ms, "est_points", (double)r->est_points);
        put(ms, "est_wh_first", r->est_wh_first);
        put(ms, "est_wh_last", r->est_wh_last);
        put(ms, "est_wh_min", r->est_wh_min);
        put(ms, "est_wh_max", r->est_wh_max);
        put(ms, "est_used_wh", o.used_wh);
        put(ms, "est_used_ah", o.used_ah);
        put(ms, "est_soc_last", o.soc_pct);
        put(ms, "est_anchor_soc_last", o.anchor_soc_pct);
        put(ms, "est_soc_gap_max", r->est_soc_gap_max);
        put(ms, "est_power_samples", (double)o.power_samples);
        put(ms, "est_anchor_samples", (double)o.anchor_samples);
    }

    /* Internal Resistance. Unconditional for the same reason Consumption's
     * source is: on a ride that never launched, "no steps, no estimate, no
     * Sag" is the fact worth pinning, and if a future change ever measures a
     * Pack off 47 s of crawling these three move off zero and say so. The ohms
     * appear only when there is an estimate, so a fixture cannot pin a
     * resistance that was never measured. */
    {
        wf_est_out_t o;
        wf_est_get(&r->est, &o);
        put(ms, "est_ir_steps", (double)o.ir_steps);
        put(ms, "est_ir_rejected", (double)o.ir_rejected);
        put(ms, "est_sag_v_max", r->sag_v_max);
        if (o.ir_valid) {
            put(ms, "est_ir_ohm", o.ir_ohm);
            put(ms, "est_ir_weight", (double)o.ir_weight);
            put(ms, "est_ir_soc_pct", o.ir_soc_pct);
        }
    }

    if (r->dist_points > 0) {
        wf_est_out_t o;
        wf_est_get(&r->est, &o);
        put(ms, "est_dist_points", (double)r->dist_points);
        put(ms, "est_distance_m", o.distance_m);
        put(ms, "est_odo_distance_m", o.odo_distance_m);
        put(ms, "est_distance_samples", (double)o.distance_samples);
        put(ms, "est_odo_samples", (double)o.odo_samples);
    }

    /* Consumption. Unconditional, because "this ride produced no Consumption
     * figure" is itself the fact worth pinning on a fixture that covers 16.7 m
     * - if a future change ever makes cap0007 divide by that, the source below
     * moves off 0 and the diff says so.
     *
     *   0  no figure: neither denominator cleared its guard
     *   1  the persisted all-time average, standing in
     *   2  the rolling window over the last WF_EST_CONS_WINDOW_M of road
     */
    {
        wf_est_out_t o;
        wf_est_get(&r->est, &o);
        put(ms, "est_cons_source",
            o.consumption_valid ? (o.consumption_windowed ? 2.0 : 1.0) : 0.0);
        put(ms, "est_alltime_m", o.alltime_m);
        put(ms, "est_alltime_wh", o.alltime_wh);
        if (o.consumption_valid) {
            put(ms, "est_cons_wh_per_km", o.consumption_wh_per_km);
        }

        /* Range, on the same terms and for the same reason: "this ride
         * produced no Range" is the fact worth pinning on a fixture that never
         * produces one, and the source is Consumption's own - Range is only
         * ever as windowed as the figure it was divided by.
         *
         *   0  no figure   1  from the all-time average   2  from the window
         */
        put(ms, "est_range_source",
            o.range_valid ? (o.consumption_windowed ? 2.0 : 1.0) : 0.0);
        if (r->range_points > 0) {
            put(ms, "est_range_points", (double)r->range_points);
            put(ms, "est_range_km_first", r->range_km_first);
            put(ms, "est_range_km_last", r->range_km_last);
            put(ms, "est_range_step_up_max", r->range_step_up_max);
        }
    }
}

/* ------------------------------------------------------------- assertions */

static int failures;

static void fail(const char *fixture, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

static void fail(const char *fixture, const char *fmt, ...)
{
    va_list ap;
    fprintf(stderr, "FAIL %s: ", fixture);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    failures++;
}

/* Invariants every Capture has to satisfy, whatever ride it came from. */
static void check_invariants(const char *fixture, const run_t *r)
{
    if (r->ctrl_records == 0) {
        fail(fixture, "no Controller records at all");
    }
    if (r->ctrl_frames_bad != 0) {
        fail(fixture, "%ld of %ld Controller frames failed length, lead byte "
                      "or CRC", r->ctrl_frames_bad, r->ctrl_records);
    }
    if (r->bms_records != 0 && r->bms_responses_ok == 0) {
        fail(fixture, "%ld BMS records and not one of them decoded",
             r->bms_records);
    }
    if (r->bms_responses_ok > 0) {
        if (!r->cell_count_stable) {
            fail(fixture, "cell count changed mid-Capture");
        }
        if (r->cell_mv_min > r->cell_mv_max) {
            fail(fixture, "no cell voltages decoded");
        }
        /* A Cell out of these bounds means the register map slipped, not that
         * the Pack is in trouble: this is lithium, 2.0-4.5 V per Cell. */
        if (r->cell_mv_min < 2000 || r->cell_mv_max > 4500) {
            fail(fixture, "cell voltages %u-%u mV are outside anything a "
                          "lithium cell does", r->cell_mv_min, r->cell_mv_max);
        }
        if (r->cell_sum_mismatch != 0) {
            fail(fixture, "%ld of %ld responses: pack voltage does not match "
                          "the sum of the cells", r->cell_sum_mismatch,
                 r->bms_responses_ok);
        }
        if (r->cell_extreme_mismatch != 0) {
            fail(fixture, "%ld of %ld responses: the highest/lowest cell "
                          "registers do not match the cell array",
                 r->cell_extreme_mismatch, r->bms_responses_ok);
        }
        if (r->cell_count_disagree != 0) {
            fail(fixture, "%ld of %ld responses: the two cell-count registers "
                          "disagree", r->cell_count_disagree,
                 r->bms_responses_ok);
        }
        if (r->soc_pct_min < 0.0 || r->soc_pct_max > 100.0) {
            fail(fixture, "state of charge %.1f-%.1f %% is not a percentage",
                 r->soc_pct_min, r->soc_pct_max);
        }
        /* The two redundant registers, as identities. Exact in all 68
         * responses of cap0006 and cap0007, so any mismatch is a slipped map
         * rather than a rounding difference - and these are the only entries
         * in the BMS table whose whole justification is the identity, so the
         * identity is what has to be re-checked on every Capture. */
        if (r->cell_delta_redundant_mismatch != 0) {
            fail(fixture, "%ld of %ld responses: cell_delta_mv is not "
                          "cell_max_mv - cell_min_mv",
                 r->cell_delta_redundant_mismatch, r->bms_responses_ok);
        }
        if (r->power_redundant_mismatch != 0) {
            fail(fixture, "%ld of %ld responses: power_w is not pack_v x "
                          "|current_a| truncated", r->power_redundant_mismatch,
                 r->bms_responses_ok);
        }
    }
    /* Rated Capacity, the number this project cannot yet read. Derived it is
     * 50 Ah on both Captures; a derivation that lands outside 45-55 Ah means a
     * register moved, because the Pack did not. This is deliberately an
     * invariant and not a per-ride expectation: any Capture of this bike that
     * derives a different Pack is wrong about something. */
    if (r->rated_ah_samples > 0 &&
        (r->rated_ah_min < 45.0 || r->rated_ah_max > 55.0)) {
        fail(fixture, "Rated Capacity derives to %.1f-%.1f Ah, and this Pack "
                      "is 50 Ah", r->rated_ah_min, r->rated_ah_max);
    }
    if (r->power_frames > 0) {
        /* This Pack is 28 lithium cells in series: 56 V flat, 118 V full.
         * Outside that the payload offset has slipped, not the Pack. */
        if (r->ctrl_pack_v_min < 40.0 || r->ctrl_pack_v_max > 130.0) {
            fail(fixture, "the Controller's pack voltage %.1f-%.1f V is outside "
                          "anything 28 lithium cells in series do",
                 r->ctrl_pack_v_min, r->ctrl_pack_v_max);
        }
    }
    /* The check that is worth more than all of the above: two devices with
     * separate instruments on one Pack, and they have to agree. cap0007 keeps
     * them within 0.30 V; 2 V is loose enough for the Controller's reading
     * being up to a poll old under load, and tight enough that a slipped
     * offset on either side cannot hide inside it. */
    if (r->pack_v_device_samples > 0 && r->pack_v_device_gap_max > 2.0) {
        fail(fixture, "the Controller and the BMS disagree about pack voltage "
                      "by up to %.2f V across %ld responses",
             r->pack_v_device_gap_max, r->pack_v_device_samples);
    }

    /* ---- the estimation seam, main/wfest ---- */
    wf_est_out_t o;
    wf_est_get(&r->est, &o);

    if (r->bms_responses_ok > 0 && !o.anchored) {
        fail(fixture, "%ld BMS answers and the estimator never acquired an "
                      "Anchor", r->bms_responses_ok);
    }
    if (o.anchored && r->est_points == 0) {
        fail(fixture, "the estimator anchored but produced no curve");
    }
    /* Remaining Energy counts down to the Limp Point, so it cannot exceed what
     * a full Pack holds above it, and wf_est_get() clamps the bottom at zero.
     * Outside this the model or the current scale has gone wrong, not the
     * ride. */
    if (r->est_points > 0) {
        double full = wf_est_energy_above_limp_wh(100.0);
        if (r->est_wh_min < 0.0 || r->est_wh_max > full) {
            fail(fixture, "Remaining Energy ran %.1f-%.1f Wh, outside the "
                          "0-%.1f Wh a full Pack holds above the Limp Point",
                 r->est_wh_min, r->est_wh_max, full);
        }
    }
    /* The acceptance criterion, as an invariant rather than a per-ride fact:
     * the Coulomb Count is Anchored to the BMS's State of Charge and does not
     * diverge from it. Five percent of the Pack is loose enough for a long
     * ride's honest integration drift between answers and far tighter than
     * anything a broken Anchor would stay inside. */
    if (r->est_soc_gap_samples > 0 && r->est_soc_gap_max > 5.0) {
        fail(fixture, "the Coulomb Count drifted %.2f %% from its Anchor over "
                      "%ld samples", r->est_soc_gap_max, r->est_soc_gap_samples);
    }

    /* ---- Internal Resistance and the Limp Point it moves ----
     *
     * Two invariants, and they hold on any Capture whatever the ride did.
     *
     * A Pack that was measured has to look like a Pack: the estimate lands
     * inside the plausibility bound the estimator itself refuses to average
     * outside of, so a number that got in by another route would show here.
     *
     * A Pack that was not measured has to leave the Limp Point exactly where
     * it was. This is the "no invented resistance" rule as arithmetic: with no
     * accepted step and nothing restored there is no Sag, no reserve, and the
     * fixed 84.0 V is what Remaining Energy counts down to - which is what
     * makes every figure on a ride like cap0007 identical to what it was
     * before any of this existed. */
    if (o.ir_valid) {
        if (o.ir_ohm < WF_EST_IR_MIN_OHM || o.ir_ohm > WF_EST_IR_MAX_OHM) {
            fail(fixture, "an Internal Resistance of %.4f Ohm, outside the "
                          "%.3f-%.3f Ohm a 28-Cell Pack can be",
                 o.ir_ohm, WF_EST_IR_MIN_OHM, WF_EST_IR_MAX_OHM);
        }
        if (o.ir_weight == 0 || o.ir_weight > WF_EST_IR_SAMPLES_MAX) {
            fail(fixture, "an Internal Resistance averaged over a weight of "
                          "%u", o.ir_weight);
        }
    } else if (o.ir_ohm != 0.0 || o.sag_v != 0.0 ||
               o.limp_point_v != WF_EST_LIMP_POINT_V) {
        fail(fixture, "no Internal Resistance was measured, and yet the Limp "
                      "Point moved to %.3f V on %.3f V of Sag from %.4f Ohm",
             o.limp_point_v, o.sag_v, o.ir_ohm);
    }
    if (o.sag_v < 0.0) {
        fail(fixture, "Sag of %.3f V: the Limp Point moved the wrong way",
             o.sag_v);
    }
    if (o.limp_point_v != WF_EST_LIMP_POINT_V + o.sag_v) {
        fail(fixture, "the Limp Point is %.6f V against the %.6f V its own Sag "
                      "implies", o.limp_point_v,
             WF_EST_LIMP_POINT_V + o.sag_v);
    }

    /* ---- distance ---- */
    if (r->dist_points > 0) {
        /* Monotone by construction. The Odometer corrects an over-reading
         * speed by slowing distance to a standstill, never by running it
         * backwards, so any backwards move at all is a fault and not a
         * tolerance to widen. */
        if (r->dist_step_back_max > 0.0) {
            fail(fixture, "distance moved backwards by up to %.6f m",
                 r->dist_step_back_max);
        }
        if (o.distance_m < 0.0) {
            fail(fixture, "distance ended at %.1f m", o.distance_m);
        }
        /* The acceptance criterion: distance agrees with the Odometer's own
         * account of the same journey to within the Odometer's quantisation.
         * The Anchor's account is the truth floored to a whole count, so the
         * fused figure sitting a little above it is right and not a drift;
         * a count is the whole budget either way. */
        double gap = o.distance_m - o.odo_distance_m;
        if (gap < 0.0) {
            gap = -gap;
        }
        if (gap > (double)WF_CTRL_ODO_METRES_PER_COUNT) {
            fail(fixture, "distance %.1f m against the Odometer's %.1f m - "
                          "%.1f m apart, more than its own %d m quantisation",
                 o.distance_m, o.odo_distance_m, gap,
                 WF_CTRL_ODO_METRES_PER_COUNT);
        }
    }

    /* ---- Consumption ---- */

    /* The stationary-bike criterion as an invariant rather than a per-ride
     * fact: a figure is reported only when something cleared a guard, and when
     * nothing did the field is left at zero rather than at whatever a division
     * by nearly no metres produced. Any Capture, including one of a bike
     * standing at a junction with the ignition on for an hour, has to satisfy
     * this. */
    if (o.consumption_valid) {
        if (o.consumption_windowed) {
            if (o.window_m < WF_EST_CONS_WINDOW_M) {
                fail(fixture, "a windowed Consumption of %.1f Wh/km over %.1f "
                              "m, less than the %.0f m window it claims",
                     o.consumption_wh_per_km, o.window_m,
                     WF_EST_CONS_WINDOW_M);
            }
        } else if (o.alltime_m < WF_EST_CONS_MIN_DIST_M) {
            fail(fixture, "an all-time Consumption of %.1f Wh/km over %.1f m, "
                          "below the %.0f m floor",
                 o.consumption_wh_per_km, o.alltime_m, WF_EST_CONS_MIN_DIST_M);
        }
    } else if (o.consumption_wh_per_km != 0.0) {
        fail(fixture, "no Consumption figure, and yet %.3f Wh/km came out of "
                      "the division that was not supposed to happen",
             o.consumption_wh_per_km);
    }

    /* ---- Range ---- */

    /* The same shape of invariant as Consumption's, and it has to hold on any
     * Capture: the division either happened under its guards or did not happen
     * at all, and there is no third state in which a figure exists that no
     * guard let through.
     *
     * The identity is the strongest part. Range times Consumption has to be
     * Remaining Energy, which is what says the estimator did this division and
     * left main/ui.c nothing to do but a "%.0f" - the criterion "Range is
     * computed in the pure estimator; the display only formats it", as
     * arithmetic rather than as a promise. */
    if (o.range_valid) {
        if (!o.valid) {
            fail(fixture, "a Range with no Remaining Energy behind it");
        }
        if (!o.consumption_valid) {
            fail(fixture, "a Range with no Consumption behind it");
        }
        if (o.consumption_wh_per_km < WF_EST_RANGE_MIN_CONS_WH_PER_KM) {
            fail(fixture, "a Range of %.1f km from a Consumption of %.3f "
                          "Wh/km, below the %.1f Wh/km floor", o.range_km,
                 o.consumption_wh_per_km, WF_EST_RANGE_MIN_CONS_WH_PER_KM);
        }
        double implied = o.range_km * o.consumption_wh_per_km;
        double slip = implied - o.remaining_wh;
        if (slip < 0.0) {
            slip = -slip;
        }
        if (slip > 1e-6 * (1.0 + o.remaining_wh)) {
            fail(fixture, "%.3f km at %.3f Wh/km is %.3f Wh, not the %.3f Wh "
                          "of Remaining Energy it was divided from - something "
                          "other than the division moved the figure",
                 o.range_km, o.consumption_wh_per_km, implied, o.remaining_wh);
        }
        /* The Pack cannot hold more than a full charge above the Limp Point,
         * and the Consumption floor is what bounds the quotient from there -
         * which is also what keeps the figure inside the three columns
         * main/ui.c's hero row has for it. */
        double most_km = wf_est_energy_above_limp_wh(100.0) /
                         WF_EST_RANGE_MIN_CONS_WH_PER_KM;
        if (o.range_km < 0.0 || o.range_km > most_km) {
            fail(fixture, "a Range of %.1f km, outside the 0-%.0f km this Pack "
                          "and this floor can produce", o.range_km, most_km);
        }
    } else if (o.range_km != 0.0) {
        fail(fixture, "no Range, and yet %.3f km came out of the division that "
                      "was not supposed to happen", o.range_km);
    }

    /* The all-time totals are the same two integrals the ride already
     * publishes, so on an unbroken replay they have to agree with them exactly
     * - anything else means Consumption is dividing something other than the
     * energy and the metres the rest of the screen shows. On a replay with a
     * simulated power cycle in it they deliberately do not: that is the whole
     * point of persisting them, and the run with the break asserts its own
     * continuity in main(). */
    if (!r->cycled) {
        double d_wh = o.alltime_wh - o.used_wh;
        double d_m  = o.alltime_m - o.distance_m;
        if (d_wh < -1e-9 || d_wh > 1e-9) {
            fail(fixture, "Consumption's all-time energy %.6f Wh is not the "
                          "%.6f Wh the ride drew", o.alltime_wh, o.used_wh);
        }
        if (d_m < -1e-9 || d_m > 1e-9) {
            fail(fixture, "Consumption's all-time distance %.6f m is not the "
                          "%.6f m the ride covered", o.alltime_m, o.distance_m);
        }
    }
}

/* Per ADR-0001 nothing the estimator concludes may reach a Capture, so a
 * Capture cannot carry a record type that would hold one. Every type the
 * format defines is raw device output (WFREC_MCU, WFREC_BMS), board telemetry
 * (WFREC_TELEM, WFREC_IMU) or text the rider or the firmware wrote
 * (WFREC_EVENT). A fixture holding anything else means a derived value found
 * its way into the archive after all. */
static void check_no_derived_records(const char *fixture, const run_t *r)
{
    if (r->other_records != 0) {
        fail(fixture, "%ld records of a type this build does not know: a "
                      "Capture holds raw device output only (ADR-0001)",
             r->other_records);
    }
}

/* ------------------------------------------------ the per-ride .expect file */

/* Compares the measurements against `name value` lines. Blank lines and
 * everything after a # are ignored. An unknown name is a failure, so a typo in
 * an .expect file cannot quietly assert nothing. */
static void check_expect(const char *fixture, const char *path,
                         const metrics_t *ms, bool *out_found)
{
    FILE *f = fopen(path, "r");
    *out_found = f != NULL;
    if (f == NULL) {
        return;
    }

    char line[256];
    int lineno = 0;
    int checked = 0;
    while (fgets(line, sizeof(line), f) != NULL) {
        lineno++;
        char *hash = strchr(line, '#');
        if (hash != NULL) {
            *hash = '\0';
        }
        char name[64];
        double want;
        int n = sscanf(line, "%63s %lf", name, &want);
        if (n == 0 || n == EOF) {
            continue;       /* blank or comment-only */
        }
        if (n != 2) {
            fail(fixture, "%s:%d: expected `name value`", path, lineno);
            continue;
        }
        const metric_t *got = find(ms, name);
        if (got == NULL) {
            fail(fixture, "%s:%d: nothing measured is called `%s`",
                 path, lineno, name);
            continue;
        }
        /* Half a display digit: the values in docs/field-table.md are
         * quoted to one decimal, and that is the precision being asserted. */
        if (got->value < want - 0.05 || got->value > want + 0.05) {
            fail(fixture, "%s = %.3f, expected %.3f", name, got->value, want);
            continue;
        }
        checked++;
    }
    fclose(f);
    printf("  %d expected value%s from %s\n", checked, checked == 1 ? "" : "s",
           path);
}

/* ----------------------------------------------- the cross-language dump */

/* One field, as text. The decimal count comes from the field's own scale, so
 * a value the table says has one decimal is printed with exactly one - which
 * is what keeps C's float and Python's double from disagreeing in a digit
 * neither of them means. */
static void print_field(void *ctx, const char *name, double value, int decimals)
{
    (void)ctx;
    printf(" %s=%.*f", name, decimals, value);
}

/* Every record that decodes, in file order, one line each: the record's index
 * in the file, then every field the generated decoder assigns from it. A
 * frame type the Field Table does not cover still prints its line, empty, so
 * that the two languages have to agree on what they do not know either. */
static void dump_fields(const uint8_t *buf, size_t len)
{
    wfl_reader_t rd;
    if (!wfl_open(&rd, buf, len, NULL)) {
        fprintf(stderr, "not a Capture, or too short to hold a header\n");
        exit(2);
    }

    wf_ctrl_live_t live;
    memset(&live, 0, sizeof(live));

    wfl_rec_t rec;
    long index = 0;
    while (wfl_next(&rd, &rec)) {
        long here = index++;
        if (rec.type == WFREC_MCU) {
            wf_ctrl_frame_t f;
            if (!wf_ctrl_frame_parse(rec.data, rec.len, &f)) {
                continue;
            }
            wf_ctrl_apply(&live, &f);
            printf("c %ld %02x", here, f.type);
            wf_ctrl_fields_dump(&live, f.type, print_field, NULL);
            putchar('\n');
        } else if (rec.type == WFREC_BMS) {
            wf_bms_t b;
            if (!wf_bms_decode(rec.data, rec.len, &b)) {
                continue;
            }
            printf("b %ld", here);
            wf_bms_fields_dump(&b, print_field, NULL);
            putchar('\n');
        }
    }
}

/* ------------------------------------------------------------------- main */

static uint8_t *slurp(const char *path, size_t *out_len)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return NULL;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return NULL;
    }
    long size = ftell(f);
    if (size < 0 || size > MAX_FIXTURE_BYTES) {
        fclose(f);
        return NULL;
    }
    rewind(f);
    uint8_t *buf = malloc((size_t)size + 1);
    if (buf == NULL) {
        fclose(f);
        return NULL;
    }
    size_t got = fread(buf, 1, (size_t)size, f);
    fclose(f);
    if (got != (size_t)size) {
        free(buf);
        return NULL;
    }
    *out_len = got;
    return buf;
}

static int cmp_name(const void *a, const void *b)
{
    return strcmp(*(const char *const *)a, *(const char *const *)b);
}

/* Every .wfl in dir, sorted, so a new fixture is picked up by dropping the
 * file in and the report reads the same on every machine. */
static int list_fixtures(const char *dir, char ***out)
{
    DIR *d = opendir(dir);
    if (d == NULL) {
        fprintf(stderr, "cannot open fixture directory %s\n", dir);
        return -1;
    }
    int n = 0, cap = 16;
    char **names = malloc((size_t)cap * sizeof(*names));
    if (names == NULL) {
        closedir(d);
        return -1;
    }
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        size_t len = strlen(e->d_name);
        if (len < 5 || strcmp(e->d_name + len - 4, ".wfl") != 0) {
            continue;
        }
        if (n == cap) {
            cap *= 2;
            char **bigger = realloc(names, (size_t)cap * sizeof(*names));
            if (bigger == NULL) {
                break;
            }
            names = bigger;
        }
        size_t need = strlen(dir) + 1 + len + 1;
        char *path = malloc(need);
        if (path == NULL) {
            break;
        }
        snprintf(path, need, "%s/%s", dir, e->d_name);
        names[n++] = path;
    }
    closedir(d);
    qsort(names, (size_t)n, sizeof(*names), cmp_name);
    *out = names;
    return n;
}

static void report(const wflog_hdr_t *h, const run_t *r)
{
    printf("  seq=%u note=\"%s\" mcu=%s bms=%s\n",
           (unsigned)h->seq, h->note, h->mcu_addr, h->bms_addr);
    printf("  %ld controller frames (%ld rejected), %ld BMS responses "
           "(%ld rejected), %ld events (%ld markers), %ld telemetry, %ld imu, "
           "%.1f s\n",
           r->ctrl_records, r->ctrl_frames_bad, r->bms_responses_ok,
           r->bms_responses_bad, r->event_records, r->marker_records,
           r->telem_records, r->imu_records, r->duration_ms / 1000.0);
    if (r->ctrl_frames_ok > 0) {
        printf("  controller: rpm<=%u speed<=%.2f km/h odo=%u (%u m) temp=%d C\n",
               r->rpm_max, r->speed_kmh_max, r->live.odometer_raw,
               (unsigned)wf_ctrl_odo_metres(r->live.odometer_raw),
               r->live.engine_temp);
    }
    if (r->power_frames > 0) {
        printf("  power: pack %.1f-%.1f V, line %.2f-%.2f A, %ld frames "
               "(%.1f Hz), device gap <=%.2f V\n",
               r->ctrl_pack_v_min, r->ctrl_pack_v_max, r->ctrl_current_a_min,
               r->ctrl_current_a_max, r->power_frames,
               r->duration_ms ? r->power_frames * 1000.0 / r->duration_ms : 0.0,
               r->pack_v_device_gap_max);
    }
    if (r->bms_responses_ok > 0) {
        printf("  bms: pack %.1f-%.1f V, soc %.1f-%.1f %%, current %.1f-%.1f A, "
               "%u cells %u-%u mV\n",
               r->pack_v_min, r->pack_v_max, r->soc_pct_min, r->soc_pct_max,
               r->current_a_min, r->current_a_max, r->cell_count,
               r->cell_mv_min, r->cell_mv_max);
    }
    if (r->est_points > 0) {
        wf_est_out_t o;
        wf_est_get(&r->est, &o);
        printf("  estimate: %.0f -> %.0f Wh above a provisional %.1f V Limp "
               "Point (%.0f %% of Pack), %.2f Wh drawn, coulomb %.2f %% vs "
               "anchor %.2f %%, curve %ld points hash %016llx\n",
               r->est_wh_first, r->est_wh_last, WF_EST_LIMP_POINT_V,
               wf_est_limp_soc_pct(), o.used_wh, o.soc_pct, o.anchor_soc_pct,
               r->est_points, (unsigned long long)r->est_hash);
    }
    {
        wf_est_out_t o;
        wf_est_get(&r->est, &o);
        if (o.ir_valid) {
            printf("  pack: %.1f mOhm over %u load steps (%lu refused), Sag "
                   "<=%.2f V, Limp Point %.1f V at %.1f A\n",
                   o.ir_ohm * 1000.0, o.ir_weight,
                   (unsigned long)o.ir_rejected, r->sag_v_max, o.limp_point_v,
                   o.load_a);
        } else {
            printf("  pack: no load step over %.1f A in this ride, so no "
                   "Internal Resistance and no Sag - the Limp Point stays at "
                   "%.1f V\n", WF_EST_IR_MIN_DI_A, WF_EST_LIMP_POINT_V);
        }
    }
    if (r->dist_points > 0) {
        printf("  distance: %.1f m fused from %.1f m of Odometer, %ld curve "
               "points hash %016llx\n",
               r->dist_m_last, r->dist_odo_last, r->dist_points,
               (unsigned long long)r->dist_hash);
    }
    {
        wf_est_out_t o;
        wf_est_get(&r->est, &o);
        if (!o.consumption_valid) {
            printf("  consumption: none - %.1f m is under the %.0f m floor and "
                   "the %.0f m window is %.0f m full\n",
                   o.alltime_m, WF_EST_CONS_MIN_DIST_M, WF_EST_CONS_WINDOW_M,
                   o.window_m);
        } else {
            printf("  consumption: %.1f Wh/km from the %s, %.2f Wh over %.1f m "
                   "all-time\n", o.consumption_wh_per_km,
                   o.consumption_windowed ? "rolling window"
                                          : "all-time average",
                   o.alltime_wh, o.alltime_m);
        }
        if (!o.range_valid) {
            printf("  range: none - there is no Consumption above the %.1f "
                   "Wh/km floor to divide %.0f Wh by\n",
                   WF_EST_RANGE_MIN_CONS_WH_PER_KM, o.remaining_wh);
        } else {
            printf("  range: %.1f -> %.1f km to a provisional %.1f V Limp "
                   "Point over %ld points, rose at most %.3f km\n",
                   r->range_km_first, r->range_km_last, WF_EST_LIMP_POINT_V,
                   r->range_points, r->range_step_up_max);
        }
    }
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--fields") == 0) {
        if (argc != 3) {
            fprintf(stderr, "usage: replay --fields <capture.wfl>\n");
            return 2;
        }
        size_t len = 0;
        uint8_t *buf = slurp(argv[2], &len);
        if (buf == NULL) {
            fprintf(stderr, "%s cannot be read\n", argv[2]);
            return 2;
        }
        dump_fields(buf, len);
        free(buf);
        return 0;
    }

    const char *dir = (argc > 1) ? argv[1] : "tests/fixtures";

    char **paths;
    int n = list_fixtures(dir, &paths);
    if (n < 0) {
        return 2;
    }
    if (n == 0) {
        fprintf(stderr, "no .wfl fixtures in %s\n", dir);
        return 2;
    }

    for (int i = 0; i < n; i++) {
        printf("%s\n", paths[i]);

        size_t len = 0;
        uint8_t *buf = slurp(paths[i], &len);
        if (buf == NULL) {
            fail(paths[i], "cannot be read");
            continue;
        }

        const odo_synth_t off = { .on = false, .start = 0 };
        wflog_hdr_t hdr;
        run_t r;
        replay(buf, len, &hdr, &r, &off, 0);
        report(&hdr, &r);
        if (hdr.version != WFLOG_VERSION) {
            /* Not a warning: a Capture from a format this build does not know
             * has been walked with the wrong layout, so everything below is
             * about to be measured out of the wrong bytes. */
            fail(paths[i], "archive version %u, this build reads %d",
                 hdr.version, WFLOG_VERSION);
        }

        /* The determinism criterion. The same bytes go through the same code a
         * second time and the whole Remaining Energy curve has to come out
         * identical - every point, bit for bit, not just the figure it ends
         * on. Anything the estimator picked up from a clock, from uninitialised
         * memory or from an accumulation whose order depended on something
         * outside the file would show up here. */
        wflog_hdr_t hdr2;
        run_t again;
        replay(buf, len, &hdr2, &again, &off, 0);
        if (again.est_hash != r.est_hash || again.est_points != r.est_points) {
            fail(paths[i], "replaying it twice produced two different "
                           "Remaining Energy curves: %ld points hash %016llx, "
                           "then %ld points hash %016llx",
                 r.est_points, (unsigned long long)r.est_hash,
                 again.est_points, (unsigned long long)again.est_hash);
        }
        if (again.dist_hash != r.dist_hash ||
            again.dist_points != r.dist_points) {
            fail(paths[i], "replaying it twice produced two different distance "
                           "curves: %ld points hash %016llx, then %ld points "
                           "hash %016llx",
                 r.dist_points, (unsigned long long)r.dist_hash,
                 again.dist_points, (unsigned long long)again.dist_hash);
        }

        /* The wrap criterion. The same ride twice more, with the Odometer's
         * count rewritten to the same ramp both times and started either side
         * of the wrap. A wrap has to be a non-event, so the two distance
         * curves are compared bit for bit and not within a tolerance. */
        const odo_synth_t clear   = { .on = true, .start = 1000 };
        const odo_synth_t wrapped = { .on = true, .start = 65530 };
        wflog_hdr_t hdr3, hdr4;
        run_t a, b;
        replay(buf, len, &hdr3, &a, &clear, 0);
        replay(buf, len, &hdr4, &b, &wrapped, 0);
        if (a.dist_points == 0) {
            fail(paths[i], "the synthesised Odometer ramp produced no distance "
                           "at all");
        } else if (a.dist_odo_last <= 0.0) {
            fail(paths[i], "the synthesised Odometer ramp never advanced, so "
                           "the wrap assertion is measuring nothing");
        } else if (b.dist_hash != a.dist_hash ||
                   b.dist_m_last != a.dist_m_last ||
                   b.dist_odo_last != a.dist_odo_last) {
            fail(paths[i], "an Odometer wrap moved distance: %.6f m from count "
                           "1000, %.6f m from count 65530, over %.0f m of "
                           "synthesised Odometer",
                 a.dist_m_last, b.dist_m_last, a.dist_odo_last);
        }
        if (a.dist_step_back_max > 0.0 || b.dist_step_back_max > 0.0) {
            fail(paths[i], "distance moved backwards across the synthesised "
                           "Odometer ramp, by up to %.6f m",
                 a.dist_step_back_max > b.dist_step_back_max
                     ? a.dist_step_back_max : b.dist_step_back_max);
        }

        /* The power-cycle criterion. The same ride again with the Monitor
         * losing power halfway through it: the estimator's state goes out
         * through the persisted bytes and comes back into a fresh estimator,
         * and the ride carries on into it.
         *
         * What this fixture can assert is the plumbing, and it is worth
         * asserting. Consumption's two all-time totals are the only state on
         * this screen that is supposed to cross a power cycle intact, so the
         * broken run has to arrive at the same energy the unbroken one did -
         * within the single-precision the blob stores them at, and not within
         * a tolerance chosen to make it pass. Distance has one metre of
         * legitimate difference in it and one only: the Odometer's count is
         * deliberately not persisted, so after the break the Anchor
         * re-acquires against the restored distance and pulls toward a
         * slightly different target for the rest of the ride. One Odometer
         * count is the budget, the same budget the unbroken invariant uses.
         *
         * What it cannot assert is the criterion's own words - that the
         * Consumption *figure* is continuous across the break - because
         * cap0007 covers 16.7 m and never produces a figure at either side of
         * it. That is driven with a synthesised ride in tests/host/unit.c,
         * against a persisted history deliberately far from the riding so that
         * a Monitor which quietly forgot its window would fail it. */
        wflog_hdr_t hdr5;
        run_t cyc;
        replay(buf, len, &hdr5, &cyc, &off, r.records / 2);
        if (!cyc.cycled) {
            fail(paths[i], "the simulated power cycle never happened, so the "
                           "continuity assertion is measuring nothing");
        } else {
            wf_est_out_t whole, broken;
            wf_est_get(&r.est, &whole);
            wf_est_get(&cyc.est, &broken);

            double d_wh = broken.alltime_wh - whole.alltime_wh;
            if (d_wh < 0.0) {
                d_wh = -d_wh;
            }
            /* The budget is one integration step, and it is a real loss and
             * not a rounding: a restored estimator has no previous timestamp
             * to integrate from, so the first power-block frame after the
             * break only starts the clock. That is the correct answer - the
             * Monitor was off across that interval and must not book energy
             * for it - and it is bounded by the longest step the estimator
             * will take, WF_EST_DT_MAX_MS, at the largest power this ride ever
             * reached. cap0007 loses 1.5 mWh of 1.73 Wh inside a 0.51 Wh
             * budget, which is still tight enough to catch a run that lost
             * half the ride. */
            double peak_a = r.ctrl_current_a_max > -r.ctrl_current_a_min
                                ? r.ctrl_current_a_max : -r.ctrl_current_a_min;
            double budget_wh = 1e-5 + 1e-6 * whole.alltime_wh;
            if (r.power_frames > 0) {
                budget_wh += r.ctrl_pack_v_max * peak_a *
                             ((double)WF_EST_DT_MAX_MS / 3600000.0);
            }
            if (d_wh > budget_wh) {
                fail(paths[i], "a power cycle mid-ride lost %.6f Wh of the "
                               "all-time total: %.6f Wh unbroken, %.6f Wh "
                               "across the break",
                     d_wh, whole.alltime_wh, broken.alltime_wh);
            }

            double d_m = broken.alltime_m - whole.alltime_m;
            if (d_m < 0.0) {
                d_m = -d_m;
            }
            if (d_m > (double)WF_CTRL_ODO_METRES_PER_COUNT) {
                fail(paths[i], "a power cycle mid-ride moved the all-time "
                               "distance by %.1f m, more than the Odometer's "
                               "own %d m quantisation: %.1f m unbroken, %.1f m "
                               "across the break",
                     d_m, WF_CTRL_ODO_METRES_PER_COUNT, whole.alltime_m,
                     broken.alltime_m);
            }
            if (cyc.dist_step_back_max > 0.0) {
                fail(paths[i], "distance moved backwards by up to %.6f m "
                               "across the simulated power cycle",
                     cyc.dist_step_back_max);
            }
            if (broken.consumption_valid != whole.consumption_valid ||
                broken.consumption_windowed != whole.consumption_windowed) {
                fail(paths[i], "a power cycle changed where the Consumption "
                               "figure comes from");
            }

            /* Range across the break. Both halves of the quotient are
             * persisted, so the figure has to come back the same one - and on
             * a fixture that produces no Range on either side, which cap0007
             * is, "no Range before and no Range after" is the whole of what
             * can honestly be asserted here. The figure's continuity across a
             * break on a ride long enough to have one is asserted in
             * tests/host/unit.c, against a persisted history deliberately
             * unlike the riding.
             *
             * The budget, when there is a figure, is the energy budget above
             * converted into kilometres by the Consumption it would be divided
             * by: the same one lost integration step, said in the units of
             * this screen. */
            if (broken.range_valid != whole.range_valid) {
                fail(paths[i], "a power cycle changed whether there is a Range "
                               "at all");
            } else if (whole.range_valid) {
                double d_km = broken.range_km - whole.range_km;
                if (d_km < 0.0) {
                    d_km = -d_km;
                }
                double budget_km = budget_wh / whole.consumption_wh_per_km;
                if (d_km > budget_km) {
                    fail(paths[i], "a power cycle mid-ride moved Range by "
                                   "%.3f km, more than the %.3f km one lost "
                                   "integration step accounts for: %.3f km "
                                   "unbroken, %.3f km across the break",
                         d_km, budget_km, whole.range_km, broken.range_km);
                }
            }
            check_invariants(paths[i], &cyc);
        }

        metrics_t ms = { .n = 0 };
        collect(&r, &ms);
        check_invariants(paths[i], &r);
        check_no_derived_records(paths[i], &r);

        char expect[512];
        snprintf(expect, sizeof(expect), "%.*s.expect",
                 (int)(strlen(paths[i]) - 4), paths[i]);
        bool found = false;
        check_expect(paths[i], expect, &ms, &found);
        if (!found) {
            fail(paths[i], "no %s: a fixture without expected values asserts "
                           "almost nothing", expect);
        }

        free(buf);
        free(paths[i]);
    }
    free(paths);

    if (failures != 0) {
        printf("\n%d failure%s\n", failures, failures == 1 ? "" : "s");
        return 1;
    }
    printf("\n%d fixture%s replayed, all assertions hold\n", n,
           n == 1 ? "" : "s");
    return 0;
}
