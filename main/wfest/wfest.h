/*
 * wfest - what the decoded fields mean for the rider, as pure C99.
 *
 * This is the estimation seam. A stream of decoded fields plus persisted state
 * goes in; Remaining Energy, the Coulomb Count and the State of Charge come
 * out. Nothing else. No I/O, no wall clock, no BLE, no display, no globals, no
 * esp_*, no FreeRTOS, no logging, no malloc - the same rules main/wfdecode
 * lives by, and for the same reason: the figures the rider sees on the
 * handlebars have to be provably the figures a recorded Capture reproduces off
 * the bike, and the only way to prove that is for one piece of code to produce
 * both.
 *
 * The single most important property here is that time is a parameter. Every
 * feed function takes t_ms from the record stream. The estimator never reads a
 * clock, so replaying a Capture is deterministic: same bytes in, same curve
 * out, every time.
 *
 * Decoding lives next door in main/wfdecode and this depends on it (for
 * wf_ctrl_live_t, wf_bms_t and the Field Table's own list of which Controller
 * frame types carry the power block). Nothing here decodes anything.
 *
 * Persisted state is a plain POD struct with an explicit byte encoding. The
 * estimator does not know NVS exists; main/est_store.c is the adapter that
 * does.
 *
 * ---------------------------------------------------------------------------
 * The physics, and how much of it is guessed
 * ---------------------------------------------------------------------------
 *
 * Per ADR-0003 the split of authority is: the BMS's State of Charge is the
 * Anchor for charge remaining, and the Controller's pack voltage and line
 * current - a sample every ~200 ms, eight of its 55 frame types - are
 * integrated for resolution between the BMS's ~1 Hz answers. Neither device's
 * own energy or consumption figure is consumed; both are computed by unknown
 * methods at resolutions too coarse to defend.
 *
 * So two quantities are integrated from the Controller and both are Anchored
 * to the same BMS State of Charge:
 *
 *   Coulomb Count   charge above 0 %, in amp-hours, from integral I dt
 *   Remaining Energy  energy above the Limp Point, in watt-hours, from
 *                     integral V I dt
 *
 * The Anchor is applied as a continuous first-order pull toward the value the
 * BMS implies, never as an assignment. That is deliberate and it is what makes
 * a gap in the BMS stream degrade gracefully: while the Anchor is stale the
 * integration simply runs on its own, and when the BMS speaks again the pull
 * resumes from wherever it left off. The alternative - re-anchoring on the
 * first answer after a gap - puts a visible step on the rider's screen at
 * exactly the moment the estimate was least trustworthy.
 *
 * The one assignment there is, is the acquisition: the very first BMS answer
 * after wf_est_init(), when there is no integrated value to preserve. After
 * that, never.
 *
 * WARNING, and it is the big one: every watt-hour below is scaled by
 * WF_CTRL_CURRENT_LSB_PER_A, which is uncertain by 19 % - upstream says 4 LSB
 * per amp, regression against the BMS says 4.77. That uncertainty propagates
 * undiminished into every figure this header produces. Issue #12 closes it
 * against Ride 1. Until then, treat the watt-hours as accurate to about a
 * fifth.
 */
#ifndef WFEST_H
#define WFEST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wfdecode.h"

/* ------------------------------------------------------------ the Limp Point
 *
 * THE ONE PROVISIONAL CONSTANT. The Limp Point is a voltage event: the
 * Controller cuts speed to walking pace when Pack voltage falls below its
 * threshold. This is that threshold.
 *
 * Provenance: 28 Cells in series at 3.00 V per Cell, the conventional
 * under-load floor for this chemistry. It is NOT read from the Controller -
 * its cutback parameter appears in no Capture we hold - and it is NOT
 * measured. Issue #8, the full-discharge ride, measures the real value; issue
 * #17 makes it move with load, because Sag is what decides when it arrives.
 *
 * Change this one number and every watt-hour below moves with it. That is the
 * point of it being one number.
 */
#define WF_EST_LIMP_POINT_V           84.0

/* ------------------------------------------- the Pack, as far as we know it
 *
 * Also provisional, but a different kind of provisional: this is the model
 * that turns a State of Charge into a Pack voltage, which is what lets a
 * charge figure become an energy figure. It is a straight line through two
 * points, which is as much curve as two points can justify.
 *
 *   full     28 Cells at 4.20 V, the standard full charge for this chemistry.
 *            tests/host/replay.c's invariant quotes the same figure rounded to
 *            118 V.
 *   measured cap0007: the BMS held 66.7 % for the whole 47 s while its pack
 *            voltage register read 105.1-105.5 V, at -0.25..8.75 A. Near
 *            enough to rest that Sag is not hiding in it.
 *
 * Cross-check worth having: integrating this line from 0 % to 100 % over a
 * 50 Ah Pack gives 5.0 kWh, against the ~5.2 kWh the Pack is reckoned to hold.
 * Two independent routes to within 4 % is not proof, but a slipped decimal
 * anywhere in here would not land inside it.
 *
 * Issue #8 replaces the whole line with a measured discharge curve.
 */
#define WF_EST_PACK_CELLS             28
#define WF_EST_PACK_V_FULL            (WF_EST_PACK_CELLS * 4.20)
#define WF_EST_PACK_V_AT_REF          105.3   /* cap0007, midpoint of 105.1-105.5 */
#define WF_EST_PACK_SOC_AT_REF        66.7    /* cap0007, pinned all ride */

/* Volts per percent of State of Charge. Constant-folded; written as the
 * subtraction it came from so the two points above stay visible in it. */
#define WF_EST_PACK_V_PER_SOC_PCT \
    ((WF_EST_PACK_V_FULL - WF_EST_PACK_V_AT_REF) / (100.0 - WF_EST_PACK_SOC_AT_REF))

/* The Pack's nameplate capacity in amp-hours. Derived from cap0006/cap0007
 * rather than read off the Pack - issue #3 reads the nameplate and replaces
 * this. Usable Capacity is smaller: it stops at the Limp Point, which is what
 * everything below counts down to. */
#define WF_EST_RATED_CAPACITY_AH      50.0

/* ------------------------------------------------------------ tuning knobs
 *
 * Not physics: filter constants, chosen and adjustable.
 *
 * ANCHOR_TAU_S    how fast the Coulomb Count is pulled toward the Anchor. Long
 *                 enough that a fresh BMS answer never yanks the rider's
 *                 figure, short enough that an hour of integration drift is
 *                 gone well inside a ride.
 * ANCHOR_STALE_MS how long a BMS answer stays fresh. The BMS is polled at
 *                 about 1 Hz, so five seconds without one is a gap and not
 *                 jitter. Past this the pull stops and the integration runs
 *                 alone - see the header comment on graceful degradation.
 * DT_MAX_MS       the longest interval a single integration step may cover.
 *                 A link that dropped for a minute must not book a minute of
 *                 the last current it happened to see.
 */
#define WF_EST_ANCHOR_TAU_S           300.0
#define WF_EST_ANCHOR_STALE_MS        5000u
#define WF_EST_DT_MAX_MS              2000u

/* ----------------------------------------------------------- sign convention
 *
 * Positive current is discharge - the Controller's own convention, kept
 * unchanged. The BMS reads the opposite way round on discharge, and that never
 * has to be reconciled here because the BMS's current is not consumed at all:
 * the BMS contributes exactly one number to this file, its State of Charge.
 *
 * So: positive line current takes the Coulomb Count and Remaining Energy down,
 * negative (regeneration, or a charger) puts them back.
 */

/* --------------------------------------------------------- persisted state
 *
 * POD, and encoded to bytes explicitly. Everything the estimator wants to
 * survive a power cycle and nothing else. The adapter that puts these bytes in
 * NVS is main/est_store.c, in the firmware, outside this module.
 *
 * A restored count bridges the gap between power-on and the first BMS answer,
 * and no further: it is deliberately NOT treated as an Anchor, so the first
 * BMS answer after a restore still acquires. A Pack charged while the Monitor
 * was off would otherwise take a quarter of an hour to be believed.
 */
#define WF_EST_PERSIST_VERSION  1
#define WF_EST_PERSIST_BYTES    24

typedef struct {
    uint16_t version;
    bool     valid;             /* the counts below mean something */
    float    coulomb_ah;        /* Coulomb Count when it was saved */
    float    remaining_wh;      /* Remaining Energy when it was saved */
    float    rated_capacity_ah; /* what those were scaled against */
} wf_est_persist_t;

/* Writes exactly WF_EST_PERSIST_BYTES into buf, little endian, with a magic,
 * the version and a Modbus CRC-16 over the rest. False when cap is too small.
 * The floats travel as their IEEE-754 single bit patterns; both the Monitor
 * and the host are little-endian IEEE machines, which is the assumption. */
bool wf_est_persist_encode(const wf_est_persist_t *p, uint8_t *buf, size_t cap);

/* The reverse. False - leaving out alone - for the wrong length, the wrong
 * magic, an unknown version or a bad CRC. A blob that fails this is treated as
 * no saved state at all, which is always a safe answer: the first BMS answer
 * acquires either way. */
bool wf_est_persist_decode(const uint8_t *buf, size_t len, wf_est_persist_t *out);

/* -------------------------------------------------------------- the estimator */

/* Caller-owned. Opaque in practice - read it through wf_est_get() - but laid
 * out here so it can live on a stack or inside another struct with no
 * allocation anywhere. */
typedef struct {
    /* the integrated quantities */
    double   last_power_w;      /* the last Controller power sample, V * I */
    double   coulomb_ah;        /* charge above 0 % State of Charge */
    double   remaining_wh;      /* energy above the Limp Point */
    double   used_ah;           /* charge drawn since init, discharge positive */
    double   used_wh;           /* energy drawn since init, discharge positive */

    /* the Anchor */
    double   anchor_soc_pct;    /* the last State of Charge the BMS reported */
    bool     anchor_seen;
    bool     acquired;          /* the first Anchor has been taken */
    uint32_t anchor_t_ms;

    /* the clock, entirely as fed */
    uint32_t last_t_ms;         /* most recent t_ms of any feed */
    bool     last_t_valid;
    uint32_t power_t_ms;        /* t_ms of the last power sample integrated */
    bool     power_t_valid;

    /* provenance of the current value, and how much has gone into it */
    bool     restored;          /* seeded from persisted state, not acquired */
    uint32_t power_samples;
    uint32_t anchor_samples;
} wf_est_t;

/* What the rider, the screen and the harness get to see. */
typedef struct {
    /* True once there is a number worth showing: an Anchor has been acquired,
     * or persisted state was restored. */
    bool     valid;
    /* False while `valid` rests on restored state alone - the figure is a
     * bridge until the BMS answers, and the screen says so. */
    bool     anchored;

    double   remaining_wh;      /* Remaining Energy, down to the Limp Point */
    double   coulomb_ah;        /* Coulomb Count */
    double   soc_pct;           /* the Coulomb Count as a State of Charge */
    double   anchor_soc_pct;    /* the BMS's own last answer */
    uint32_t anchor_age_ms;     /* since that answer, at the last fed record */
    bool     anchor_fresh;      /* the pull toward the Anchor is running */

    double   used_ah;           /* since init */
    double   used_wh;           /* since init */
    double   power_w;           /* the last Controller power sample, V * I */

    uint32_t power_samples;
    uint32_t anchor_samples;
} wf_est_out_t;

/* Zeroes e and, when restored is non-NULL and valid, seeds the counts from it.
 * Must be called before any feed. */
void wf_est_init(wf_est_t *e, const wf_est_persist_t *restored);

/* Folds one decoded Controller frame in.
 *
 * frame_type is the frame's own type byte. The estimator integrates on the
 * power block's eight types and only those, because those are the only ones
 * that carry a fresh pack voltage and line current - so the integration steps
 * are the ~5.2 Hz the Pack is actually sampled at rather than the 35 Hz the
 * link runs at, and the step boundaries do not depend on which unrelated frame
 * happened to arrive in between. Which types those are comes from the Field
 * Table, per ADR-0002, not from a list repeated here.
 *
 * Short enough to run inside a spinlock, which is where the Monitor calls it
 * from - the same place it calls wf_ctrl_apply().
 */
void wf_est_feed_ctrl(wf_est_t *e, uint32_t t_ms, uint8_t frame_type,
                      const wf_ctrl_live_t *live);

/* Folds one decoded BMS response in. Takes the State of Charge and nothing
 * else: the BMS's pack voltage and current are not consumed here (ADR-0003),
 * and neither is any energy figure it offers. Updates the Anchor; it does not
 * itself move the integrated values, except on the one acquisition. */
void wf_est_feed_bms(wf_est_t *e, uint32_t t_ms, const wf_bms_t *bms);

/* The estimates. Pure: no clock, so the Anchor's age is measured against the
 * last record fed in and not against now. */
void wf_est_get(const wf_est_t *e, wf_est_out_t *out);

/* Snapshots the estimator into something worth writing down. */
void wf_est_save(const wf_est_t *e, wf_est_persist_t *out);

/* ------------------------------------------------------------ the model, alone
 *
 * Exposed so the numbers above can be checked without driving a whole ride
 * through the estimator, and so a caller can label a screen with where the
 * Limp Point sits.
 */

/* The State of Charge at which Pack voltage reaches WF_EST_LIMP_POINT_V, on
 * the provisional straight line. Around 9 % on today's constants - so a tenth
 * of the Pack sits below the Limp Point and is not counted. */
double wf_est_limp_soc_pct(void);

/* Pack voltage the line predicts at a given State of Charge. Deliberately not
 * the Controller's live reading: that sags under load, and Remaining Energy
 * must not fall by a hundred watt-hours because the rider opened the throttle.
 * Making the Limp Point move with the live voltage is issue #17. */
double wf_est_pack_v_at_soc(double soc_pct);

/* Energy above the Limp Point at a given State of Charge, in watt-hours: the
 * charge between here and the Limp Point, times the mean voltage it will be
 * delivered at along the line. Zero at and below the Limp Point. This is what
 * the Anchor pulls Remaining Energy toward. */
double wf_est_energy_above_limp_wh(double soc_pct);

#endif /* WFEST_H */
