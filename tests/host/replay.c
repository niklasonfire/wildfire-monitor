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
#define MAX_METRICS       64

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

/* One of the eight frame types of the power block. The Field Table exports the
 * list because the eight are not an arithmetic run - the stride is 7 except
 * for 0xab -> 0xb1, which is 6 - so there is nothing to compute and everything
 * to look up. */
static bool is_power_type(uint8_t type)
{
    for (int i = 0; i < WF_CTRL_TYPE_POWER_COUNT; i++) {
        if (wf_ctrl_type_power[i] == type) {
            return true;
        }
    }
    return false;
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

/* Sampled after every record the estimator was fed, so the curve is what the
 * live screen would have shown at each of those instants. */
static void est_sample(run_t *r)
{
    wf_est_out_t o;
    wf_est_get(&r->est, &o);
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

static void run_init(run_t *r)
{
    memset(r, 0, sizeof(*r));
    r->pack_v_min = r->soc_pct_min = r->current_a_min = 1e9;
    r->pack_v_max = r->soc_pct_max = r->current_a_max = -1e9;
    r->ctrl_pack_v_min = r->ctrl_current_a_min = 1e9;
    r->ctrl_pack_v_max = r->ctrl_current_a_max = -1e9;
    r->cell_mv_min = 0xffffu;
    r->cell_count_stable = true;
    r->est_hash = 0xcbf29ce484222325ull;   /* FNV-1a offset basis */
    /* Cold: a fixture carries no persisted state, so every replay of it starts
     * from the same nothing. That is half of why replays agree. */
    wf_est_init(&r->est, NULL);
}

static void replay(const uint8_t *buf, size_t len, wflog_hdr_t *hdr, run_t *r)
{
    wfl_reader_t rd;
    if (!wfl_open(&rd, buf, len, hdr)) {
        fprintf(stderr, "not a Capture, or too short to hold a header\n");
        exit(2);
    }

    run_init(r);

    wfl_rec_t rec;
    while (wfl_next(&rd, &rec)) {
        r->duration_ms = rec.t_ms;

        switch (rec.type) {
        case WFREC_MCU: {
            r->ctrl_records++;
            wf_ctrl_frame_t f;
            if (!wf_ctrl_frame_parse(rec.data, rec.len, &f)) {
                r->ctrl_frames_bad++;
                break;
            }
            r->ctrl_frames_ok++;
            wf_ctrl_apply(&r->live, &f);
            if (r->live.b0_valid && r->live.cur_rpm > r->rpm_max) {
                r->rpm_max = r->live.cur_rpm;
            }
            if (r->live.speed_valid && f.type == WF_CTRL_TYPE_MOTION) {
                r->speed_samples++;
                if (r->live.cur_speed_kmh > r->speed_kmh_max) {
                    r->speed_kmh_max = r->live.cur_speed_kmh;
                }
            }
            /* Counted per frame rather than per type: all eight carry the
             * same two fields, and what matters downstream is how often the
             * pair arrives, not which of the eight brought it. */
            if (r->live.power_valid && is_power_type(f.type)) {
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

        wflog_hdr_t hdr;
        run_t r;
        replay(buf, len, &hdr, &r);
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
        replay(buf, len, &hdr2, &again);
        if (again.est_hash != r.est_hash || again.est_points != r.est_points) {
            fail(paths[i], "replaying it twice produced two different "
                           "Remaining Energy curves: %ld points hash %016llx, "
                           "then %ld points hash %016llx",
                 r.est_points, (unsigned long long)r.est_hash,
                 again.est_points, (unsigned long long)again.est_hash);
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
