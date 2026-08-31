/*
 * wfest - what the decoded fields mean for the rider, as pure C99.
 *
 * This is the estimation seam. A stream of decoded fields plus persisted state
 * goes in; the Coulomb Count, the State of Charge, Remaining Energy, Distance,
 * Consumption and Range come out - Range being the one the rider rides by, and
 * the reason the other five are here at all. Nothing else. No I/O, no wall
 * clock, no BLE, no display, no globals, no
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
 * ---------------------------------------------------------------------------
 * Distance, which is the same shape again
 * ---------------------------------------------------------------------------
 *
 * A third quantity is integrated, and it is here rather than beside the
 * estimator because Consumption and Range are Remaining Energy divided by it -
 * a distance the replay harness cannot reproduce is a Range the replay harness
 * cannot reproduce.
 *
 *   Distance        metres travelled since wf_est_init(), from integral v dt
 *
 * Its Anchor is the Odometer, per ADR-0003 and for exactly the reasons the BMS
 * anchors charge: road speed has resolution the Odometer has not - it arrives
 * with the motion block at ~5.2 Hz, against one Odometer count per hundred
 * metres - and it drifts, because it is rpm times a wheel circumference and a
 * gearing constant, none of which is measured. The Odometer is coarse and it
 * wraps but it does not drift.
 *
 * The fusion is the same first-order pull, with one addition: the step a
 * motion frame takes is `speed * dt + (odometer - distance) * dt/TAU`, and
 * that step is floored at zero. Distance is a quantity a rider watches
 * accumulate, and a correction that ran it backwards - even smoothly, even by
 * a metre - would be wrong in a way a charge figure is not. So the Odometer
 * can slow distance to a standstill and never reverse it: an over-reading
 * speed is absorbed by the figure sitting still until the Odometer catches up.
 *
 * The one assignment, again, is the acquisition. The first Odometer reading
 * sets the Anchor equal to whatever distance already is, so acquiring costs
 * nothing; from there the Anchor moves by wrap-safe count differences
 * (wf_ctrl_odo_delta_metres) and never by an absolute reading. That is what
 * makes a wrap a non-event - 65535 to 0 is one count, not a trip backwards -
 * and what makes a Controller link that dropped for two minutes come back
 * carrying the metres it covered, in the Odometer's own account, without
 * double-counting them.
 *
 * Before the wheel geometry has arrived (frame type 0xaf, without which
 * `speed_valid` is false) nothing is integrated from speed: an unknown speed
 * is not assumed to be zero, it is simply not integrated. The Odometer's pull
 * still runs, so distance in that window is the Odometer alone, smoothed - a
 * hundred-metre staircase turned into something continuous. `distance_valid`
 * is true as soon as either source has spoken and false before that, because
 * a confident 0 m from a link that has never come up is worse than a dash.
 *
 * ---------------------------------------------------------------------------
 * Consumption, which is a ratio and so is mostly a question of what to divide
 * by
 * ---------------------------------------------------------------------------
 *
 *   Consumption     watt-hours per kilometre - energy drawn divided by the
 *                   Distance it bought
 *
 * The numerator is the same integral V I dt the Remaining Energy count runs
 * on, and deliberately not the Anchored figure: the Anchor's pull moves
 * Remaining Energy without any energy having gone anywhere, and a Consumption
 * that moved with it would be measuring the BMS rather than the riding. So
 * Consumption is built from the raw integration steps, which makes it exactly
 * as uncertain as WF_CTRL_CURRENT_LSB_PER_A and no more.
 *
 * The denominator is where the design is. Two things are wanted at once and
 * they pull opposite ways:
 *
 *   Recency.    The number has to mean "how you are riding now", so that it
 *               rises on the motorway and falls in town. That wants a short
 *               window.
 *   Steadiness. It must not swing between coasting and climbing, which is what
 *               an instantaneous watts-over-metres-per-second does at every
 *               throttle movement. That wants a long one.
 *
 * The window is over Distance and not over time, which resolves most of it: a
 * window measured in metres holds the same amount of *riding* whatever the
 * speed, where a window measured in seconds holds ten times as much road on a
 * motorway as in traffic - and is empty at a red light, which is exactly when
 * a time-windowed figure diverges. WF_EST_CONS_WINDOW_M is that one constant.
 *
 * It is implemented as a ring of WF_EST_CONS_BUCKETS fixed-length buckets of
 * road, each accumulating the energy drawn while its metres were covered. A
 * bucket closes when it has collected its span and the oldest is dropped, so
 * the reported figure is the ratio of the totals over the last full window.
 * The figure therefore moves once per bucket and not once per frame, which is
 * the steadiness half made concrete: a fifty-metre step at 36 km/h is a new
 * number every five seconds, and one bucket can move it by at most a twentieth
 * of the difference between the road it covers and the road it replaces.
 *
 * Before the ring has filled there is no window, and that is precisely the
 * first kilometre - when the rider is deciding whether to set off and wants a
 * number most. So a persisted all-time average stands in: two running totals,
 * energy and distance, accumulated across every ride this Monitor has seen and
 * divided on demand. Totals and not a stored mean, because a mean cannot be
 * improved by a new ride without also carrying the weight it was taken over,
 * and two totals carry that for free - the tenth ride moves the average by a
 * tenth and not by all of it. The rider is told which of the two is on screen;
 * `consumption_windowed` is that flag and main/ui.c renders it.
 *
 * Both are divisions by a distance, so both are guarded. A bike standing at a
 * junction with the ignition on draws current and covers no ground, and the
 * quotient of those two is not a large Consumption figure, it is a meaningless
 * one. The window's guard is structural - the ring reports only once it holds
 * a full window of road, which a standing bike never gives it - and the
 * all-time average has an explicit floor, WF_EST_CONS_MIN_DIST_M, of one
 * Odometer count. Below that there is no figure at all and the screen shows a
 * dash. cap0007 is exactly this case: 16.7 m of fused Distance in 47 s, the
 * first eight of them stationary, and it must not produce a Consumption
 * figure.
 *
 * Energy drawn while standing still is still counted; it left the Pack. It
 * lands in whichever bucket is open when the bike moves again and washes out
 * of the window within a kilometre. The all-time totals keep it forever, which
 * is the honest answer for a lifetime average and does mean an afternoon
 * parked with the ignition on and the Monitor running biases it upward.
 *
 * ---------------------------------------------------------------------------
 * Range, which is the one number the rider actually rides by
 * ---------------------------------------------------------------------------
 *
 *   Range           kilometres still rideable before the Limp Point, at the
 *                   way the bike is being ridden now
 *
 * It is Remaining Energy divided by Consumption and nothing else. Both are
 * above; neither is recomputed for it, and there is no filter of its own -
 * which is the whole design, so it is worth saying why each half of that is
 * deliberate.
 *
 * The crawl is already excluded and must not be excluded twice. Remaining
 * Energy counts down to the Limp Point rather than to zero State of Charge -
 * wf_est_energy_above_limp_wh() is zero at and below it - so a Range built on
 * it reaches zero exactly when the Controller cuts to walking pace, which is
 * what "reaching zero means the ride is over" requires. The kilometre of
 * crawling that follows sits below the Limp Point and is therefore already
 * outside the numerator. Subtracting a kilometre here as well would count it
 * twice and would make the figure pessimistic by a kilometre at every point of
 * every ride.
 *
 * There is no smoothing on the quotient because the smoothing belongs in the
 * denominator, where it already is. Consumption moves once per closed bucket -
 * once per WF_EST_CONS_BUCKET_M of road - and one bucket can move it by at most
 * a twentieth of the difference between the road entering the window and the
 * road leaving it. So Range inherits a figure that cannot lurch on a single
 * noisy frame, and adding a second filter on top would only make it slower to
 * react without making it any steadier.
 *
 * That leaves monotonicity, and the two acceptance criteria pull against each
 * other hard enough to be worth settling here rather than in a test:
 *
 *   Range falls monotonically over a recorded ride, absent regeneration.
 *   Range reacts to a sustained change of riding style within the window.
 *
 * Strict global monotonicity is incompatible with the second one. A rider who
 * slows down is genuinely spending less per kilometre and can genuinely go
 * further, and a Range that refused to say so would be a ratchet rather than an
 * estimate. So what is implemented, and what tests/host/unit.c asserts, is:
 *
 *   under steady riding, Range is monotone non-increasing - strictly, on the
 *   double, sampled at every frame - and falls by one kilometre per kilometre
 *   ridden, which is the identity that says the numerator and the denominator
 *   are describing the same riding;
 *
 *   under a sustained change, it moves, and the motorway shows up inside one
 *   window;
 *
 *   under instrument noise it does neither. Half an LSB of line current -
 *   the whole of the quantisation error - dithered onto every sample moves the
 *   window by a fiftieth of a percent, which is less than half of what a
 *   bucket of road takes off the Range in the same instant. The riding outruns
 *   the instrument at every bucket boundary.
 *
 * That last one is a bound and not a zero, and the difference is worth being
 * straight about. Rendering a continuous figure as a whole number puts a
 * rounding boundary every kilometre, and a hundredth of a percent of wobble
 * near one of them is a displayed kilometre ticking back up: about three times
 * in ten kilometres, by one, never twice running. Removing that would take a
 * filter on Range itself, and no filter can tell it apart from the rise a
 * rider earns by easing off - which is the second criterion above, and is the
 * figure moving for exactly the right reason. The wobble is also some two
 * thousand times smaller than the 19 % this figure is uncertain by for as long
 * as WF_CTRL_CURRENT_LSB_PER_A is unsettled. So there is no filter, and
 * tests/host/unit.c asserts the bound instead of pretending to a zero.
 *
 * There is one discontinuity that is none of those three, and it is expected
 * rather than tolerated: the handover, when the ring fills and the rolling
 * window takes over from the persisted all-time average. Those are two
 * different figures - how the rider is riding now, against how they have
 * ridden ever - so Range moves by whatever they differ by, in whichever
 * direction the ride differs from the history. It is not jitter and it is not
 * a step in one figure; it is the answer to a different question, arriving
 * once, a kilometre into the ride, and the screen marks it by dropping the
 * star. Monotonicity is asserted on either side of it and not across it.
 *
 * A floor on how small that handover can be does not exist even in principle,
 * but a floor on how small it is under steady riding does: the two integrals
 * behind the window are cut on different frame types, so the first window is
 * up to one frame interval - one part in five hundred - out against the road
 * it covers. On a Range of 90 km that is under two hundred metres.
 *
 * The two divisions are guarded, and the guard is a floor on the divisor rather
 * than a cap on the quotient: WF_EST_RANGE_MIN_CONS_WH_PER_KM. Consumption can
 * legitimately come out at zero or below - a windowful of descent, or more
 * regeneration than draw - and a quotient of a real energy by nearly nothing is
 * not a large Range, it is a meaningless one. Below the floor there is no Range
 * at all and the screen shows a dash, which is the same answer Consumption
 * itself gives below its own floor.
 *
 * ---------------------------------------------------------------------------
 * Internal Resistance, and the Limp Point that moves with it
 * ---------------------------------------------------------------------------
 *
 *   Internal Resistance  the Pack's opposition to current, in ohms, as Sag per
 *                        amp: the ratio of a voltage change to the current
 *                        change that caused it, at a sharp load step
 *   Sag                  Internal Resistance times the load the rider is
 *                        holding, in volts
 *
 * The Limp Point is a voltage event, and a voltage event under load. The
 * Controller cuts to walking pace when the Pack voltage it sees falls below
 * its threshold, and the voltage it sees is the rested voltage minus Sag. So a
 * rider holding 150 A reaches that threshold with a fifth of the Pack still in
 * it, and the same rider easing off gets it back. A Range built on a fixed
 * 84.0 V cannot say either thing, and is wrong in the direction that strands
 * people.
 *
 * So the fixed Limp Point stays exactly where it was, and Sag is applied on
 * top of it as a reserve: the watt-hours lying between WF_EST_LIMP_POINT_V and
 * WF_EST_LIMP_POINT_V + Sag are subtracted from Remaining Energy at the moment
 * it is read. The integration and the Anchor are untouched - they still count
 * down to the fixed line, so no integrated state has to be re-based when the
 * load changes, and a change to the Sag model can never corrupt a Coulomb
 * Count. wf_est_sag_reserve_wh() is the whole of the model and is exposed so it
 * can be checked on its own.
 *
 * Estimating the resistance is a rejection problem, not an averaging problem.
 * Voltage arrives quantised to 0.1 V and current to a quarter of an amp, both
 * at ~5.2 Hz, and a ratio of two small quantised differences is mostly noise -
 * so the design is in what is refused. WF_EST_IR_MIN_DI_A is derived from the
 * two quantisations rather than chosen; the rules are written out where the
 * constants are.
 *
 * Two things this rate costs, and they are worth being straight about. At one
 * sample per ~192 ms the ohmic step and the Pack's fast polarisation cannot be
 * told apart, so what is measured is the resistance seen over about a fifth of
 * a second and not the pure ohmic value - larger than an instantaneous one,
 * and the right one for predicting Sag under a load the rider is holding.
 * And the peak of a transient is never seen at all, so the Sag modelled here
 * is the Sag of sustained riding rather than the dip of one throttle stab.
 *
 * Which is also, deliberately, what keeps Range steady. The load fed into the
 * Sag model is not the last frame's current but an average of it over
 * WF_EST_SAG_TAU_S, so a single frame moves the Limp Point by about a
 * hundredth of what a sustained load does. The smoothing lives in the input,
 * one level down, exactly as Consumption's does - Range still has no filter of
 * its own.
 *
 * ---------------------------------------------------------------------------
 * The weakest Cell, which is what actually ends the ride
 * ---------------------------------------------------------------------------
 *
 *   Cell spread     how far the lowest Cell sits below the Pack average, in
 *                   volts per Cell
 *   Cell band       what that puts on the Limp Point, in Pack volts
 *   Cell reserve    the watt-hours the rider gives up because of it
 *
 * Everything above this line treats the Pack as its average. It is not: it is
 * 28 Cells in series and it stops on the weakest of them. WF_EST_LIMP_POINT_V
 * is 28 x 3.00 V, so the Pack reaches the Limp Point when the *average* Cell
 * reaches 3.00 V - and by then the lowest Cell has been under 3.00 V for a
 * while. A Range built on the average is optimistic by exactly that, and the
 * sicker the Pack the more optimistic it is.
 *
 * WHAT THE MINIMUM CELL IMPLIES, and the assumption in it. The 28 Cells are
 * taken to follow one curve, offset from each other: the lowest Cell is the
 * average Cell shifted down by the spread, and it therefore reaches its own
 * 3.00 V when the average reaches 3.00 V plus the spread - that is, when the
 * Pack reaches WF_EST_LIMP_POINT_V plus 28 spreads. So the whole of the
 * minimum-Cell estimate is one more band of volts above the Limp Point, of
 * exactly the shape Sag already puts there, and it is computed by the same
 * wf_est_sag_reserve_wh().
 *
 * 28 identical Cells in series is the assumption, and it is precisely the
 * assumption a weak Cell violates - a Cell that is losing capacity does not
 * merely sit lower on the same curve, it has a shorter one. That is why this
 * is a clamp and not a replacement: it says "no more than this", which is the
 * direction that cannot strand a rider, and it says nothing about how much
 * further the shortfall goes.
 *
 * THE DIFFERENCE AND NOT THE VALUE. The spread is `avg_cell_mv - cell_min_mv`
 * and never the lowest Cell's absolute reading, and that is what makes it
 * usable at all. Cell voltages sag under load exactly as the Pack's does - at
 * 150 A a Cell reads a quarter of a volt low, which taken absolutely would
 * wipe a third off the Range every time the rider opened the throttle. Sag is
 * common to all 28 Cells, so it cancels out of the difference and what is left
 * is imbalance. (A weak Cell's own extra resistance does not cancel, and shows
 * as the spread widening under load - which is real, and is the weak Cell
 * genuinely reaching the Limp Point first when the rider pulls hard.)
 *
 * THE DEADBAND, which is why a healthy Pack pays nothing. WF_EST_PACK_V_AT_REF
 * was measured on cap0007 - a Pack whose lowest Cell ran 3 to 7 mV under its
 * average through all 34 responses - so the 84.0 V line already has a normal
 * Pack's imbalance inside it. Charging the rider for that again would be
 * double-counting it, the same mistake as subtracting the crawl below the Limp
 * Point twice. So only imbalance past WF_EST_CELL_DEADBAND_V is new
 * information, and only that is charged. A healthy Pack produces a reserve of
 * exactly zero, and cap0007's pinned figures are the figures they were.
 *
 * That also makes the clamp continuous rather than a switch: at the deadband
 * the reserve is zero and it grows smoothly from there, so nothing steps onto
 * the rider's screen at the moment a Cell crosses a threshold. The clamp is
 * visible to the rider exactly when it binds, because "binds" and "costs a
 * watt-hour" are the same condition; main/ui.c marks the hero row.
 *
 * THE WARNING, and the one thing it must not do. Divergence widens on its own
 * as the Pack empties: the Cells' charge offsets are fixed, the curve steepens
 * at the bottom, so the same imbalance in amp-hours shows as a wider and wider
 * spread in millivolts. A threshold in millivolts therefore fires on every
 * healthy Pack at low charge, the rider learns to ignore it, and it is worth
 * nothing on the day it matters.
 *
 * So the warning's threshold is not in millivolts at all. It is a ratio
 * between two spreads measured on the same Pack in the same response:
 *
 *   the gap below the lowest Cell   its second-lowest Cell minus its lowest
 *   the Pack's own fan-out          its highest Cell minus its second-lowest
 *
 * and the warning is that the first is at least WF_EST_CELL_OUTLIER_K times
 * the second. Whatever the local steepness of the discharge curve is, it
 * multiplies both of those, and it cancels out of their ratio - which is
 * exactly the physics that would otherwise defeat a fixed threshold, turned
 * into the reason this one holds. A healthy Pack at 10 % fans out; all 28
 * Cells fan out together, so the ratio is where it was at 90 %. A failing Cell
 * leaves the other 27 behind, and only the numerator grows.
 *
 * cap0007 is the evidence for the value. Over its 34 responses that ratio runs
 * 0.00 to 0.33, and the warning fires at 2.0 - six times the widest thing a
 * healthy Pack was ever seen to do. It is not a round number chosen to look
 * reasonable; it is the measured spread of a good Pack with a factor of six on
 * it, and a longer ride down to low charge is what would tighten it.
 *
 * The warning also requires the clamp to be binding at all, so a Pack whose 28
 * Cells sit within 3 mV of each other cannot raise an alarm because one of
 * them is 1 mV lower than the rest. Two failing Cells side by side defeat the
 * ratio - they raise the denominator together - and that is a real limitation
 * and not a case this can catch from one response.
 *
 * WHAT IS NOT BELIEVED. A response whose Cell block is not a plausible 28-Cell
 * Pack is refused whole, the previous reading stands, and `cell_rejected`
 * counts it - a Monitor producing nothing but rejections is a decode problem
 * and not a quiet zero. Refused: a cell count that is not
 * WF_EST_PACK_CELLS, any Cell outside WF_EST_CELL_MIN_MV..MAX_MV (which a
 * zero-filled array and a 6 V "Cell" both fail), an average register outside
 * the array's own extremes, and a spread past WF_EST_CELL_MAX_SPREAD_V, where
 * a dying Cell and a slipped register map can no longer be told apart. A
 * dropped BMS response is not even a rejection: the last plausible reading
 * stands, because imbalance is a fact about the Pack and not about the link.
 * In every one of those cases Range degrades to the unclamped Pack-average
 * estimate - never to zero.
 *
 * The three spreads are running means over the last WF_EST_CELL_SAMPLES_MAX
 * BMS answers rather than the last one alone, which is the same smoothing
 * discipline as everywhere else here: one millivolt of movement on a 1 mV
 * quantised register would otherwise be worth a tenth of a kilometre of Range,
 * twice what a bucket of road takes off it. Indexed by answers and not by
 * time, so it needs no clock of its own. The mean is not persisted - the
 * imbalance is re-acquired from the first BMS answer after a power cycle, in
 * about a second, the same as the load average behind Sag.
 *
 * ---------------------------------------------------------------------------
 * WARNING: the 19 %, and which way it goes in each figure
 * ---------------------------------------------------------------------------
 *
 * WF_CTRL_CURRENT_LSB_PER_A is uncertain by 19 % - upstream says 4 LSB per
 * amp, regression against the BMS says 4.77 - so every current this file
 * integrates may be a fifth too high. Issue #12 closes it against Ride 1.
 * Until then the magnitude is a fifth everywhere and the *direction is not the
 * same in every figure*, which is the part a reader will otherwise get wrong:
 *
 *   Consumption, used_wh, used_ah and the two all-time totals scale DIRECTLY
 *   with the constant. They are integrals of current and nothing corrects
 *   them. If the true scale is 4.77, they read 19 % high.
 *
 *   Remaining Energy is Anchored to the BMS's State of Charge, and the BMS
 *   knows nothing about the Controller's current scale. With a live Anchor the
 *   figure is therefore largely INDEPENDENT of the constant: the exposure is
 *   only what the integration accumulates between one BMS answer and the next,
 *   about a second of it, which the pull then takes back out. Through a long
 *   BMS gap that protection is gone and the figure drifts with the scale like
 *   any other integral - which is exactly the case
 *   test_a_bms_gap_produces_no_step() drives.
 *
 *   Range is Remaining Energy over Consumption, so the error enters ONCE,
 *   through the denominator alone. A 19 % overestimate of current makes
 *   Consumption 19 % high and Range 1 - 1/1.19 = 16 % PESSIMISTIC. It does not
 *   enter twice, and the two halves do not cancel: the numerator is not scaled
 *   by it at all.
 *
 *   Internal Resistance is Sag per amp, and current is in its denominator, so
 *   it sits where Range sits and not where Consumption sits: a 19 % high
 *   current scale reads a 16 % LOW resistance. Issue #21 reads this number as
 *   a State of Health signal and has to carry that bias with it.
 *
 *   Sag is the one figure here that the constant CANCELS out of, and it is the
 *   one that moves the Limp Point. The resistance is measured as dV/dI in the
 *   Controller's own amps and is then multiplied by a load measured in the
 *   same amps: the scale appears once in a denominator and once in a numerator
 *   and leaves. The Sag-corrected Limp Point, and the watt-hours Range gives
 *   up because of it, are as good as the voltage readings alone - the one part
 *   of this file that does not have to wait for Ride 1.
 */
#ifndef WFEST_H
#define WFEST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wfdecode.h"
/* Consumption against speed, as constants. Generated offline by
 * scripts/fit_consumption.py over the whole Capture archive and committed;
 * see wf_est_consumption_at_speed() at the bottom of this file. */
#include "wf_fit.h"

/* ------------------------------------------------------------ the Limp Point
 *
 * THE ONE PROVISIONAL CONSTANT. The Limp Point is a voltage event: the
 * Controller cuts speed to walking pace when Pack voltage falls below its
 * threshold. This is that threshold.
 *
 * Provenance: 28 Cells in series at 3.00 V per Cell, the conventional
 * under-load floor for this chemistry. It is NOT read from the Controller -
 * its cutback parameter appears in no Capture we hold - and it is NOT
 * measured. Issue #8, the full-discharge ride, measures the real value.
 *
 * It is the Limp Point at rest. What Range counts down to is this plus Sag at
 * the load the rider is holding, which is issue #17 and is the Internal
 * Resistance section below; the constant itself did not move and does not move
 * with load. `limp_point_v` in wf_est_out_t is the moving figure.
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
 * DIST_TAU_S      how fast distance is pulled toward the Odometer. A fifth of
 *                 the charge Anchor's, and the trade-off is worth writing out
 *                 because it is the whole of why this number is 60 and not 6
 *                 or 600.
 *
 *                 Long is what stops a tick showing. The Odometer moves in
 *                 hundred-metre steps, so the pull's target is a staircase and
 *                 not a noisy reading; one step adds `100 * dt/TAU` to the very
 *                 next frame, which at 60 s and 5.2 Hz is 0.33 m against the
 *                 ~2 m a frame covers at 36 km/h. Invisible. At 6 s it would
 *                 be 3.3 m - larger than the step itself, which is a tick you
 *                 could watch.
 *
 *                 Short is what stops the drift accumulating. A first-order
 *                 tracker settles TAU times the rate error away from its
 *                 target, and the target itself sits up to one count below the
 *                 truth, so the error against the ground is about
 *                 `TAU * speed_error - 50 m`. At 60 s a 20 % speed error at
 *                 36 km/h lands 63 m out: inside one Odometer count, which is
 *                 all the accuracy an Odometer-Anchored figure can claim.
 *                 tests/host/unit.c drives exactly that case.
 */
#define WF_EST_ANCHOR_TAU_S           300.0
#define WF_EST_ANCHOR_STALE_MS        5000u
#define WF_EST_DT_MAX_MS              2000u
#define WF_EST_DIST_TAU_S             60.0

/* --------------------------------------------------- Consumption's window
 *
 * WINDOW_M is the stated constant the acceptance criterion asks for: the
 * length of road Consumption is averaged over. One kilometre, which is one
 * unit of the figure's own denominator - a Wh/km averaged over a kilometre is
 * a sentence the rider can finish - and, at anything from town speed to
 * motorway speed, between half a minute and two minutes of riding. Short
 * enough that a motorway slip road shows up inside it, long enough that a
 * single hill does not own it.
 *
 * BUCKETS is how finely that window is chopped, and it buys two things at the
 * cost of memory. It is the granularity of the figure's movement - one bucket
 * closing swaps a twentieth of the window, so the number steps rather than
 * jitters - and it is the resolution of the window's own edge, because the
 * window is really 1000 m to 1050 m of road rather than exactly 1000.
 *
 * The cost is the ring: BUCKETS pairs of doubles, 320 bytes, inside the
 * caller's wf_est_t. That is the whole of what Consumption adds to a board
 * running a BLE stack and a FAT writer in 320 KB, and it is a fixed 320 bytes
 * - no allocation, no growth with ride length.
 *
 * MIN_DIST_M is the stationary-bike guard on the all-time average, and it is
 * one Odometer count: below a hundred metres the Monitor cannot claim to know
 * how far it has gone at all, which makes it the shortest distance worth
 * dividing by. The windowed figure needs no such constant - it cannot report
 * until it holds a full window, and a standing bike never fills one.
 */
#define WF_EST_CONS_WINDOW_M          1000.0
#define WF_EST_CONS_BUCKETS           20
#define WF_EST_CONS_BUCKET_M          (WF_EST_CONS_WINDOW_M / \
                                       WF_EST_CONS_BUCKETS)
#define WF_EST_CONS_MIN_DIST_M        100.0

/* ------------------------------------------------------- Range's one guard
 *
 * The smallest Consumption that may be divided into. It is a floor on the
 * divisor and not a cap on the answer, because a capped answer still claims to
 * be an estimate and this is the case where there is no estimate to make.
 *
 * Two reasons for the value, and they agree, which is why it is this one.
 *
 * Nothing this bike does costs less. The cheapest riding any Capture we hold
 * has shown is tens of watt-hours per kilometre, and a fifth of the cheapest of
 * those is not a riding style - it is a kilometre of descent, or a window in
 * which regeneration outweighed draw, and neither predicts the next hundred
 * kilometres. Consumption at or below zero lands here too: the comparison is
 * written the way round that rejects a negative and a NaN as well.
 *
 * And it bounds the figure structurally. A full Pack holds about 4585 Wh above
 * the Limp Point on the constants above, so the largest Range that can ever
 * come out of this division is about 917 km - three digits, which is what the
 * hero row on the live screen has room for. The screen cannot be overflowed by
 * a number the estimator is willing to produce. If the Pack model or this floor
 * ever moves, check that pair again; main/ui.c's layout comment says so too.
 */
#define WF_EST_RANGE_MIN_CONS_WH_PER_KM  5.0

/* ------------------------------------------- Internal Resistance's thresholds
 *
 * What counts as a load step sharp enough to measure a Pack with, and what a
 * measured Pack is allowed to look like. Every number here is derived from an
 * instrument's quantisation or from what a 28-Cell Pack can physically be;
 * none of them is a round number picked because it looked reasonable.
 *
 * THE TWO QUANTISATIONS. Internal Resistance is dV/dI and both are quantised,
 * so both land straight in the estimate. Pack voltage is a u16 scaled by 0.1
 * (field-table.json, Controller `pack_v`), so a reading is up to half a
 * quantum out and a *difference* of two readings up to a whole one: 0.1 V.
 * Line current is scaled by WF_CTRL_CURRENT_LSB_PER_A, so the same argument
 * gives a whole quantum on the difference: 0.25 A at 4 LSB per amp.
 *
 * THE PLAUSIBILITY BOUND. A 28-Cell, ~5 kWh lithium Pack with its cabling and
 * connectors is tens of milliohms. 20 mOhm is about as stiff as this Pack
 * could be when new and warm; 200 mOhm is a Pack that is cold, old, or being
 * measured through a bad connection, and is where a believable reading stops.
 * An estimate outside that has found noise, a decode error or a stationary
 * transient, and is thrown away rather than averaged in - which also throws
 * away every step where the voltage moved the wrong way, since that gives a
 * negative ratio and negative is below the floor.
 *
 * THE STEP THRESHOLD, which follows from the three above rather than being
 * chosen. For one step,
 *
 *   dR/R  <=  q_V / |dV|  +  q_I / |dI|  =  q_V / (R |dI|)  +  q_I / |dI|
 *
 * and the worst case over a plausible Pack is the stiffest one, R = MIN_OHM,
 * because that is where a given current step produces the smallest voltage
 * step to measure. Demanding no worse than WF_EST_IR_STEP_ERR from one step
 * rearranges to the minimum current step below - 21 A on today's numbers, and
 * it moves on its own if the current scale is corrected or the bound on the
 * Pack changes. At the Pack's likely 50 mOhm the same step carries about 11 %,
 * and the running mean takes that down further.
 *
 * Note which quantisation dominates: 0.1 V over a 20 mOhm Pack is 5 A of
 * equivalent current error against the current channel's own 0.25 A. The
 * voltage's tenth of a volt is twenty times the problem the current's quarter
 * of an amp is, and a threshold reasoned from the current channel alone would
 * be twenty times too permissive.
 *
 * MAX_DT_MS is what "sharp" means in time. Voltage and current arrive together
 * every ~192 ms (ADR-0003), so two consecutive samples is the sharpest thing
 * this stream can show; 400 ms allows one jittered interval and refuses a pair
 * that straddles a dropped frame, where a ramp would be indistinguishable from
 * a step. Together with MIN_DI_A that makes the acceptance rule "at least 21 A
 * within 400 ms", or at least 52 A/s - which a launch from rest clears easily
 * and a rider rolling on the throttle does not.
 */
#define WF_EST_IR_V_QUANTUM_V     0.1
#define WF_EST_IR_I_QUANTUM_A     (1.0 / WF_CTRL_CURRENT_LSB_PER_A)
#define WF_EST_IR_MIN_OHM         0.020
#define WF_EST_IR_MAX_OHM         0.200
#define WF_EST_IR_STEP_ERR        0.25
#define WF_EST_IR_MIN_DI_A        ((WF_EST_IR_V_QUANTUM_V / WF_EST_IR_MIN_OHM + \
                                    WF_EST_IR_I_QUANTUM_A) / WF_EST_IR_STEP_ERR)
#define WF_EST_IR_MAX_DT_MS       400u

/* ------------------------------------------------ Internal Resistance's knobs
 *
 * Not physics: how the accepted steps are turned into one number, and how fast
 * that number is allowed to move the rider's Range.
 *
 * SAMPLES_MAX is the weight cap on the running mean. Each accepted step moves
 * the estimate by 1/n of the way to itself, with n capped here, so the first
 * launch of a Monitor's life settles it and the two hundredth cannot wander it
 * - a step is worth at most a sixty-fourth once the estimate is grown up. That
 * is the "stable, not wandering with each step" criterion as arithmetic, and
 * it is a cap rather than a freeze because a Pack's resistance genuinely rises
 * with age and the estimate has to be able to follow it. Sixty-four steps is a
 * few spirited rides.
 *
 * CONFIDENT_SAMPLES is how the Sag correction fades in, and it exists to keep
 * one number off the rider's screen: a step. The first accepted step of a
 * Monitor's life arrives in the middle of a hard launch - that is what makes
 * it acceptable - and applying a full Sag reserve at that instant would take
 * several kilometres off the Range in one frame. So the reserve is scaled by
 * min(n, CONFIDENT) / CONFIDENT, which is the same instinct as the Anchor's
 * pull: never assign, always approach. A launch and the release at the end of
 * it are two steps, so eight is four launches - the first few minutes of any
 * ride that involves a throttle.
 *
 * SAG_TAU_S is the load average the Sag model is driven by, and it is the
 * whole of the anti-jitter design. At 5.2 Hz a single frame moves it by
 * dt/TAU, about 1 %, so one noisy frame moves the Limp Point by a hundredth of
 * what the same current sustained does; ten seconds of real acceleration moves
 * it by 39 % of the way. Long enough that the number is riding style and not
 * throttle position, short enough that easing off is repaid while the rider is
 * still easing off.
 */
#define WF_EST_IR_SAMPLES_MAX        64u
#define WF_EST_IR_CONFIDENT_SAMPLES  8u
#define WF_EST_SAG_TAU_S             20.0

/* ------------------------------------------------- the weakest Cell's limits
 *
 * The prose is in the header comment above; these are the five numbers it
 * settles, and none of them is a second Limp Point - the per-Cell one is
 * WF_EST_LIMP_POINT_V divided by WF_EST_PACK_CELLS and is written that way.
 *
 * MIN_MV / MAX_MV   what a lithium Cell can read at all, the same 2.0-4.5 V
 *                   bound tests/host/replay.c holds every Capture to. A
 *                   zero-filled array fails the floor and a 6 V "Cell" fails
 *                   the ceiling, which is the whole of the implausible-reading
 *                   rule.
 * DEADBAND_PCT      the imbalance already inside the Pack model, as percent of
 *                   Pack. One percent is 13.2 mV per Cell on the line above -
 *                   twice the widest imbalance cap0007 showed, and worth about
 *                   42 Wh, which is where the hero row's "%.0f" would start to
 *                   move. Below it the reserve is exactly zero.
 * MAX_SPREAD_V      where a dying Cell stops being distinguishable from a
 *                   slipped register. Half a volt below the Pack average is
 *                   14 V of band, a tenth of the Pack, and a Cell that far
 *                   down would have tripped the BMS's own protection long
 *                   before a rider read it here. Past this the response is
 *                   refused and Range is the unclamped figure, which is what
 *                   "a missing or implausible reading must not clamp Range to
 *                   zero" requires.
 * OUTLIER_K         the divergence warning, as a ratio and deliberately not as
 *                   a voltage. See the header comment: cap0007's healthy Pack
 *                   runs 0.00-0.33 and this is 2.0.
 * SAMPLES_MAX       the weight cap on the running mean over BMS answers, about
 *                   sixteen seconds at the ~1 Hz poll.
 */
#define WF_EST_CELL_MIN_MV           2000u
#define WF_EST_CELL_MAX_MV           4500u
#define WF_EST_CELL_LIMP_V           (WF_EST_LIMP_POINT_V / WF_EST_PACK_CELLS)
#define WF_EST_CELL_DEADBAND_PCT     1.0
#define WF_EST_CELL_DEADBAND_V       (WF_EST_CELL_DEADBAND_PCT * \
                                      WF_EST_PACK_V_PER_SOC_PCT / \
                                      WF_EST_PACK_CELLS)
#define WF_EST_CELL_MAX_SPREAD_V     0.5
#define WF_EST_CELL_OUTLIER_K        2.0
#define WF_EST_CELL_SAMPLES_MAX      16u

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
 *
 * Distance is the exception, and deliberately: it is restored outright and
 * carries on from where it was, because "how far have I ridden" has no
 * equivalent of a Pack being charged overnight. What it does not carry is the
 * Odometer's count. The first Odometer reading after a restore acquires
 * against the restored distance, so ground the bike covered while the Monitor
 * was off - the Monitor runs on its own battery and can be switched off under
 * a rider who keeps going - is not credited to a Capture that did not see it.
 *
 * Version 2 added distance_m and its own validity flag, in the five bytes
 * version 1 left reserved for exactly this. A version 1 blob is therefore
 * migrated rather than rejected: its layout is a prefix of this one and its
 * reserved bytes are zero by construction, so it restores its charge figures
 * and no distance at all, which is precisely what a build that did not count
 * metres knew.
 *
 * Version 3 spent the reserve and grew the blob, from 24 bytes to 40, for
 * Consumption: the two all-time totals and a summary of the rolling window.
 * A version 2 blob is 24 bytes and is still read - migrated, not reinterpreted
 * - because the alternative is throwing away a rider's Distance and charge on
 * the firmware update that added Consumption, for nothing. The length is what
 * selects the layout and the version has to agree with it, so there is no
 * reading of the new fields out of bytes an old blob never wrote: a 24-byte
 * blob carries version 1 or 2 and its CRC at byte 22, a 40-byte blob carries
 * version 3 and its CRC at byte 38, and every other combination is rejected.
 * What a migrated version 2 blob restores is no all-time average and no window
 * - which is exactly what a build that did not compute Consumption knew - so
 * the first ride after the update rebuilds both.
 *
 * Version 4 grew it again, 40 bytes to 46, for Internal Resistance: the
 * estimate and the weight it was averaged over. Same migration rule, same
 * reason - a version 3 blob restores no resistance and no Sag correction,
 * which is what a build without one knew, and the first hard launch after the
 * update measures one.
 *
 * The weight is what makes the estimate improve across rides rather than start
 * again each morning: a Monitor that comes back with sixty-four steps behind
 * its number treats the next step as a sixty-fourth, and one that comes back
 * with three treats the next as a quarter. What is deliberately NOT persisted
 * is the load average behind Sag. That is riding and not a fact about the
 * Pack; a Monitor that comes back mid-ride re-acquires it from the first power
 * frame, and one that comes back after a coffee stop should be starting from
 * the standing load it actually has.
 *
 * The window summary, window_wh over window_m, is the one piece of state here
 * that is a filter rather than a fact. It is saved only when the ring was full
 * and it is restored by spreading it evenly across the ring, so the Monitor
 * comes back from a coffee stop still knowing how the rider was riding instead
 * of falling back to a lifetime average that a single spirited ride cannot
 * move. It flushes out over the next kilometre. The partial bucket in progress
 * is not saved; at most fifty metres of road is lost with it.
 *
 * The validity flags are separate because the figures are. A Monitor that saw
 * the Controller but never the BMS has metres worth keeping and no charge
 * figure at all; the Rated Capacity guard in wf_est_init() throws away a
 * charge count and has nothing to say about a distance or a Consumption.
 */
#define WF_EST_PERSIST_VERSION      4
#define WF_EST_PERSIST_VERSION_MIN  1   /* the oldest layout still readable */
#define WF_EST_PERSIST_BYTES        46
#define WF_EST_PERSIST_BYTES_V3     40  /* the version 3 layout */
#define WF_EST_PERSIST_BYTES_V2     24  /* the version 1 and 2 layout */

typedef struct {
    uint16_t version;
    bool     valid;             /* the charge counts below mean something */
    float    coulomb_ah;        /* Coulomb Count when it was saved */
    float    remaining_wh;      /* Remaining Energy when it was saved */
    float    rated_capacity_ah; /* what those were scaled against */
    bool     distance_valid;    /* distance_m means something (version 2) */
    float    distance_m;        /* Distance when it was saved (version 2) */
    /* Consumption, version 3. The totals are all-time; the window pair is the
     * rolling window at the moment of the save, and is zero when there was no
     * full window to save. */
    float    alltime_wh;        /* energy drawn over every ride, in Wh */
    float    alltime_m;         /* Distance over every ride, in metres */
    float    window_wh;
    float    window_m;
    /* Internal Resistance, version 4: the estimate in ohms and the number of
     * accepted load steps it is the running mean of, capped at
     * WF_EST_IR_SAMPLES_MAX. Zero weight means no estimate, and then the ohms
     * mean nothing and no Sag is applied. */
    float    ir_ohm;
    uint16_t ir_weight;
} wf_est_persist_t;

/* Writes exactly WF_EST_PERSIST_BYTES into buf, little endian, with a magic,
 * the version and a Modbus CRC-16 over the rest. False when cap is too small.
 * Always writes the current version's layout; the older ones are read only.
 * The floats travel as their IEEE-754 single bit patterns; both the Monitor
 * and the host are little-endian IEEE machines, which is the assumption. */
bool wf_est_persist_encode(const wf_est_persist_t *p, uint8_t *buf, size_t cap);

/* The reverse. False - leaving out alone - for a length no layout uses, a
 * version that does not match the length, the wrong magic or a bad CRC. A blob
 * that fails this is treated as no saved state at all, which is always a safe
 * answer: the first BMS answer acquires either way. */
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

    /* Consumption's rolling window: a ring of closed buckets of road, plus the
     * one still filling. Indexed by metres covered and never by time or by
     * frame count, so it holds the same amount of riding at any speed. Fixed
     * size, 320 bytes, no allocation. */
    double   cons_wh[WF_EST_CONS_BUCKETS];
    double   cons_m[WF_EST_CONS_BUCKETS];
    double   cons_part_wh;      /* the bucket currently filling */
    double   cons_part_m;
    uint8_t  cons_head;         /* where the next closed bucket goes */
    uint8_t  cons_filled;       /* closed buckets held, capped at BUCKETS */

    /* The all-time average, as the two totals it is the ratio of. Seeded from
     * persisted state and added to for the life of the Monitor. */
    double   alltime_wh;
    double   alltime_m;

    /* distance, and the Odometer that Anchors it */
    double   distance_m;        /* metres since init, the fused figure */
    double   odo_distance_m;    /* the Odometer's own account of the same */
    uint16_t odo_counts;        /* the last raw count differenced against */
    bool     odo_seen;          /* the Odometer has been acquired */
    bool     speed_seen;        /* a road speed has been integrated */

    /* Internal Resistance, and the load average Sag is computed at. Both move
     * only on power-block frames, because both are about the Pack under load
     * and that is the only block that carries a voltage and a current.
     *
     * The previous power sample is kept in full - value and timestamp - rather
     * than leaning on power_t_ms, so that what a step is measured across is
     * visibly the pair of samples and not whatever else has been going on. */
    double   ir_ohm;            /* the running mean over accepted steps */
    double   ir_soc_pct;        /* the Anchor when that step was accepted */
    uint32_t ir_weight;         /* steps in the mean, capped at SAMPLES_MAX */
    uint32_t ir_steps;          /* accepted steps this power-on, uncapped */
    uint32_t ir_rejected;       /* sharp enough to try, refused anyway */
    double   ir_prev_v;
    double   ir_prev_a;
    uint32_t ir_prev_t_ms;
    bool     ir_prev_valid;
    double   load_a;            /* the load Sag is taken at, averaged */
    bool     load_valid;

    /* The weakest Cell, from the BMS's per-Cell registers. Three spreads and
     * not one, because two different questions are asked of them: how much the
     * lowest Cell costs (against the average) and whether it has left the Pack
     * behind (against the Pack's own fan-out). All three are running means over
     * BMS answers, weighted by cell_weight and capped, so one millivolt of
     * movement on a quantised register cannot move the rider's Range. */
    double   cell_spread_v;     /* average Cell minus lowest Cell */
    double   cell_gap_v;        /* second-lowest Cell minus lowest Cell */
    double   cell_body_v;       /* highest Cell minus second-lowest Cell */
    uint16_t cell_min_mv;       /* the last plausible reading, as it arrived */
    uint16_t cell_avg_mv;
    uint32_t cell_weight;       /* answers in the means, capped at SAMPLES_MAX */
    uint32_t cell_samples;      /* plausible Cell blocks folded in */
    uint32_t cell_rejected;     /* Cell blocks refused as not a 28-Cell Pack */

    /* the clock, entirely as fed */
    uint32_t last_t_ms;         /* most recent t_ms of any feed */
    bool     last_t_valid;
    uint32_t power_t_ms;        /* t_ms of the last power sample integrated */
    bool     power_t_valid;
    uint32_t speed_t_ms;        /* t_ms of the last motion frame integrated */
    bool     speed_t_valid;

    /* provenance of the current value, and how much has gone into it */
    bool     restored;          /* seeded from persisted state, not acquired */
    bool     distance_restored; /* distance came from persisted state */
    uint32_t power_samples;
    uint32_t anchor_samples;
    uint32_t distance_samples;  /* motion frames that took a distance step */
    uint32_t odo_samples;       /* Odometer readings folded into the Anchor */
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

    /* Distance. One figure in metres, and the thing Consumption and Range
     * divide by. `distance_valid` is false until a road speed or an Odometer
     * reading has arrived; `odo_anchored` says which of the two is holding it,
     * because integrated speed alone drifts and the screen may want to say so.
     * `odo_distance_m` is the Odometer's own account of the same journey,
     * exposed so a harness can assert the two agree to within the Odometer's
     * hundred-metre quantisation. */
    bool     distance_valid;
    bool     odo_anchored;
    double   distance_m;
    double   odo_distance_m;

    /* Consumption, in watt-hours per kilometre.
     *
     * `consumption_valid` is false until there is a distance worth dividing by
     * - the stationary-bike guard - and the screen shows a dash rather than
     * the ratio of a real energy to almost no metres.
     *
     * `consumption_windowed` says which of the two figures this is, and the
     * screen has to render the difference: true is the rolling window over the
     * last WF_EST_CONS_WINDOW_M of road, which is how the rider is riding now;
     * false is the persisted all-time average standing in until the window has
     * filled, which is how they have ridden ever.
     *
     * `window_m` is how much road the ring is holding, exposed so a harness
     * can tell "the window is not full" from "the window is full and the
     * answer happens to match the all-time average". The all-time totals are
     * exposed for the same reason.
     */
    bool     consumption_valid;
    bool     consumption_windowed;
    double   consumption_wh_per_km;
    double   window_m;
    double   alltime_m;
    double   alltime_wh;

    /* Range, in kilometres: Remaining Energy divided by Consumption, and the
     * primary figure on the live screen. Zero means the Limp Point, because
     * the numerator counts down to the Limp Point - the crawl below it is
     * already outside this figure and is not subtracted again.
     *
     * `range_valid` is false when there is no Remaining Energy to divide, no
     * Consumption to divide by, or a Consumption below
     * WF_EST_RANGE_MIN_CONS_WH_PER_KM, and then `range_km` is left at exactly
     * zero - which is the proof that the division did not happen rather than
     * that it happened and produced something small.
     *
     * Which of the two Consumptions it was built on is `consumption_windowed`
     * and is not repeated here: a Range from the persisted all-time average is
     * exactly as provisional as the average it came from, and the screen marks
     * both with the same star. */
    bool     range_valid;
    double   range_km;

    /* Internal Resistance and the Limp Point it moves.
     *
     * `ir_valid` is false until a load step has been accepted or a persisted
     * estimate restored, and then `ir_ohm` is exactly zero and no Sag is
     * applied at all - the Monitor does not invent a Pack resistance, and a
     * ride that never launches hard counts down to the fixed Limp Point
     * exactly as it did before this existed.
     *
     * `ir_ohm` is the running mean over `ir_weight` accepted steps; `ir_steps`
     * counts every step accepted since power-on and `ir_rejected` counts those
     * that were sharp enough to try and were refused anyway - a Pack that
     * produces nothing but rejections is a decode problem, not a quiet zero.
     * `ir_soc_pct` is the State of Charge the last accepted step was taken at,
     * because resistance depends on it and issue #21 compares readings over
     * months.
     *
     * `load_a` is the averaged load Sag is taken at, `sag_v` is the volts it
     * costs, and `limp_point_v` is what Range is actually counting down to:
     * WF_EST_LIMP_POINT_V plus Sag. Remaining Energy above is already net of
     * the reserve between the two, so Range needs no further correction and
     * main/ui.c needs no arithmetic. */
    bool     ir_valid;
    double   ir_ohm;
    uint32_t ir_weight;
    uint32_t ir_steps;
    uint32_t ir_rejected;
    double   ir_soc_pct;
    double   load_a;
    double   sag_v;
    double   limp_point_v;

    /* The weakest Cell, and the Range it holds down.
     *
     * `remaining_pack_wh` is the Pack-average estimate - Sag-corrected, and
     * with no Cell clamp on it. `remaining_wh` above is the clamped figure, so
     * the acceptance criterion "Range takes the lower of the Pack-average
     * estimate and the minimum-Cell estimate" is readable straight off the
     * pair, and the two are equal on a Pack whose imbalance is inside the
     * deadband.
     *
     * `cell_valid` is false until a plausible 28-Cell block has arrived, and
     * then every figure here is exactly zero and Range is the Pack-average
     * estimate untouched - the same "no invented number" rule the Internal
     * Resistance follows.
     *
     * `cell_clamped` is the clamp binding, which is the same thing as the
     * reserve being worth a watt-hour: it is true exactly when the spread has
     * passed WF_EST_CELL_DEADBAND_V, and main/ui.c marks the hero row so a
     * clamped Range cannot look like an unclamped one.
     *
     * `cell_diverged` is the warning: the lowest Cell has left the other 27
     * behind by WF_EST_CELL_OUTLIER_K times the Pack's own fan-out, while the
     * clamp is binding. A ratio and not a voltage, so it does not fire on a
     * healthy Pack at low charge; the header comment works that through.
     *
     * `cell_rejected` counts Cell blocks refused as not a 28-Cell Pack, so a
     * decode that has silently stopped producing them is visible rather than
     * looking like a Pack with no imbalance. */
    bool     cell_valid;
    bool     cell_clamped;
    bool     cell_diverged;
    uint16_t cell_min_mv;
    uint16_t cell_avg_mv;
    double   cell_spread_v;     /* average Cell minus lowest Cell */
    double   cell_band_v;       /* what that puts on the Limp Point, past the
                                 * deadband, in Pack volts */
    double   cell_reserve_wh;   /* the watt-hours it costs */
    double   remaining_pack_wh; /* Remaining Energy before the Cell clamp */
    uint32_t cell_samples;
    uint32_t cell_rejected;

    uint32_t power_samples;
    uint32_t anchor_samples;
    uint32_t distance_samples;
    uint32_t odo_samples;
} wf_est_out_t;

/* Zeroes e and, when restored is non-NULL and valid, seeds the counts from it.
 * Must be called before any feed. */
void wf_est_init(wf_est_t *e, const wf_est_persist_t *restored);

/* Folds one decoded Controller frame in.
 *
 * frame_type is the frame's own type byte, and it decides what this does:
 *
 *   the power block's eight types   integrate energy and charge, measure a
 *                                   load step against the previous sample, and
 *                                   move the load average Sag is taken at
 *   the motion block's eight types  integrate distance, and pull it toward
 *                                   the Odometer
 *   the Odometer's type, 0x94       move the Odometer Anchor, nothing else
 *   anything else                   note the time and return
 *
 * Each quantity steps only on the frames that carry a fresh reading of it, so
 * the integration steps are the ~5.2 Hz the Pack and the wheel are actually
 * sampled at rather than the 35 Hz the link runs at, and the step boundaries
 * do not depend on which unrelated frame happened to arrive in between. Which
 * types those are comes from the Field Table, per ADR-0002, not from a list
 * repeated here.
 *
 * Short enough to run inside a spinlock, which is where the Monitor calls it
 * from - the same place it calls wf_ctrl_apply().
 */
void wf_est_feed_ctrl(wf_est_t *e, uint32_t t_ms, uint8_t frame_type,
                      const wf_ctrl_live_t *live);

/* Folds one decoded BMS response in. Two things come out of it and no others:
 * the State of Charge, which is the Anchor, and the per-Cell voltages, which
 * are the only place the Monitor can learn that the Pack is not its average.
 * The BMS's pack voltage and current are not consumed here (ADR-0003), and
 * neither is any energy figure it offers, nor register 56, which is redundant.
 *
 * Neither moves an integrated value: the Anchor is a pull applied on the next
 * power frame - except on the one acquisition - and the Cell spread is applied
 * at read time in wf_est_get(), exactly as Sag is, so no accumulator is ever
 * re-based on a Cell reading. */
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
 * must not fall by a hundred watt-hours because the rider opened the throttle
 * for one frame. Sag reaches the figure through wf_est_sag_reserve_wh() below,
 * driven by an averaged load and a measured resistance, which is the same
 * physics with the noise taken out of it. */
double wf_est_pack_v_at_soc(double soc_pct);

/* The Pack volts the weakest Cell puts on the Limp Point, given how far it
 * sits below the Pack average: 28 times the spread past
 * WF_EST_CELL_DEADBAND_V, and zero at or inside the deadband. The imbalance
 * inside the deadband is already in the Pack model, which was measured on a
 * Pack that had some - charging the rider for it twice is the same mistake as
 * subtracting the crawl twice.
 *
 * Zero for a NaN and for a negative spread as well, which is a lowest Cell
 * above its own Pack average and so a decode that has gone wrong. */
double wf_est_cell_band_v(double spread_v);

/* The watt-hours lying between the Limp Point and a band of volts above it:
 * the energy a rider gives up because the Pack cannot be taken down to the
 * Limp Point after all.
 *
 * Two things put a band there and they add: Sag, which is the load the rider
 * is holding, and the weakest Cell, which is wf_est_cell_band_v() above. The
 * band is nonlinear in its own width - it carries the mean voltage the charge
 * would have come out at - so the two are summed and this is called once
 * rather than called twice and added.
 *
 * The name is Sag's because Sag was here first; the model is a band of volts
 * and knows nothing about where the volts came from.
 *
 * On the straight line above, the charge in a band of the Pack depends only on
 * how many volts wide the band is, so this is a function of Sag alone and not
 * of where the State of Charge happens to be. Zero for a Sag of zero or less -
 * regeneration raises the terminal voltage, and crediting a rider with extra
 * Range for a downhill would be optimistic in exactly the wrong direction. */
double wf_est_sag_reserve_wh(double sag_v);

/* Energy above the Limp Point at a given State of Charge, in watt-hours: the
 * charge between here and the Limp Point, times the mean voltage it will be
 * delivered at along the line. Zero at and below the Limp Point. This is what
 * the Anchor pulls Remaining Energy toward.
 *
 * Against the fixed WF_EST_LIMP_POINT_V, always. The integration and the
 * Anchor are defined against the resting line so that they do not have to be
 * re-based every time the rider's wrist moves; Sag is subtracted from the
 * result at read time instead. */
double wf_est_energy_above_limp_wh(double soc_pct);

/* ------------------------------------------- Consumption at a chosen speed
 *
 * What the archive says the bike costs per kilometre at a speed the rider is
 * not currently riding at. Everything above answers "what is happening"; this
 * answers "what would happen", and it is the only thing here that does.
 *
 * NOTHING FITS ON THE MONITOR, and this is where that is enforced rather than
 * asserted. The coefficients come out of wf_fit.h, which
 * scripts/fit_consumption.py writes offline from the whole Capture archive
 * (issue #19). This evaluates a polynomial in two constants. There is no
 * state, no accumulator, no adaptation and nothing that could become one: it
 * is a function of its argument and of two #defines, and a Monitor that has
 * been running for a month gives the same answer as one just switched on.
 *
 * THE FORM is a + c*v^2, with the quadratic drag term explicit. Energy per
 * unit distance is force; rolling resistance and driveline loss are roughly
 * constant with speed and aerodynamic drag goes as the square of it. Note it
 * is the per-kilometre figure and not power, which is force times speed and
 * would go as v^3 - two curves that look alike and are different quantities.
 *
 * THE TWO WAYS IT DECLINES TO ANSWER, and they are different:
 *
 *   `fitted` false     there is no fit at all. The archive has never held a
 *                      ride this could be fitted from - see wf_fit.h, which
 *                      says why in the words the tool refused in - and
 *                      `wh_per_km` is left at exactly zero, which is the
 *                      proof that no polynomial was evaluated rather than
 *                      that one was and came out small.
 *   `extrapolated`     there is a fit, and the speed asked about is outside
 *                      the range the data covers. The number IS produced -
 *                      an extrapolated quadratic is still the best guess
 *                      available - but it is flagged, so a caller cannot show
 *                      it as though it were supported. Issue #20 suggests a
 *                      slower speed to the rider and must only suggest one
 *                      this comes back clear for.
 *
 * A caller that ignores both flags is asking for a number the archive did not
 * earn, which is exactly what the Confidence discipline in CONTEXT.md exists
 * to make visible.
 */
typedef struct {
    bool   fitted;          /* WF_FIT_FITTED: a real fit, not the placeholder */
    bool   extrapolated;    /* asked outside the fitted speed range */
    double wh_per_km;       /* exactly zero when !fitted */
    double speed_min_kmh;   /* the range the fit is supported over, both zero */
    double speed_max_kmh;   /* when there is no fit, so nothing is inside it */
} wf_est_fit_t;

void wf_est_consumption_at_speed(double speed_kmh, wf_est_fit_t *out);

/* The polynomial alone, with the coefficients handed in. Exposed because the
 * archive cannot exercise the fitted path - there is no fit to exercise it
 * with - and a branch that only ever runs one way is a branch nobody has
 * tested. tests/host/unit.c drives it against coefficients it chose. */
double wf_est_fit_eval(double a, double b, double c, double speed_kmh);

#endif /* WFEST_H */
