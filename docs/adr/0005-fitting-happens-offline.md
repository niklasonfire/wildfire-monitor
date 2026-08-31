---
status: accepted
---

# Fitting happens offline and the Monitor consumes constants

Anything the project learns from more than one ride is fitted off the bike,
over the Capture archive, by a tool that runs on a development machine. What
reaches the Monitor is a small number of constants with the fit that produced
them recorded beside them. Nothing on the device fits, trains, adapts, or
accumulates a model across rides.

The estimator may hold state that a ride produces - a Coulomb Count, a
Consumption window, a running mean of Internal Resistance - and ADR-0004 says
where that lives. This is a different thing: a model whose *shape* was chosen
by looking at data. That is fitted in `scripts/`, never in `main/`.

## Why

Issue #1 lists it under Implementation Decisions and reinforces it in Out of
Scope by rejecting machine learning outright, and the reasons hold up
independently of that.

**A fit needs the whole archive, and the Monitor has one ride.** The device
sees the ride it is on. The interesting question - what does a kilometre cost
at 70 km/h - is answered by riding at 70 km/h, which the current ride may
never do. An on-device fit would be fitting the last hour, which is what
Consumption's rolling window already is and does better, being honest about
being a window.

**The dominant input is missing, so capacity does not help.** Without route
knowledge, gradient is unmodelled and is the largest term in what a kilometre
actually costs. A model with more parameters does not recover an input nobody
recorded; it fits the noise that input left behind and reports a better R^2
for it. Two coefficients with a physical meaning each can be wrong in a way a
reader can see. A trained model cannot.

**A fitted number must be reproducible from the evidence.** ADR-0001 keeps
Captures raw so a ride can be re-analysed with decoding that did not exist
when it was taken, and ADR-0002 makes every decoder a function of one
declared table. A coefficient that drifted on the handlebars belongs to no
archive: it could not be re-derived, could not be reviewed, and could not be
explained after a Range figure turned out wrong. Offline, the coefficient is
a pure function of the files in the archive, and `make test` asserts that the
committed constants are still what those files produce.

**Confidence is the project's discipline and it needs a provenance to
attach to.** The Field Table carries per-field Confidence so that "upstream
says so" and "we proved it" are never confused. A constant fitted offline can
carry the same thing - R^2, sample count, the speed range the data covers,
which Captures went in. A number the device fitted to itself overnight has
nowhere to put any of that.

**It keeps the seam pure.** ADR-0004's estimator has no clock, no I/O and no
allocation, which is what lets the same code run on the handlebars and in the
replay harness. A fitter needs to hold every sample of a ride and solve a
linear system. Putting that behind the seam would cost the property that
makes replay meaningful, to buy an answer that is worse.

## Consequences

The deliverable of a fitting ticket is two things, not one: the tool, and the
constants it produced, both committed. Issue #19 is the first -
`scripts/fit_consumption.py` and `main/wfest/wf_fit.h`.

Generated constants are held to the archive the way `docs/field-table.md` is
held to the Field Table: the tool writes the header, the header is committed,
and the test suite fails if it has drifted from what today's Captures
produce. Re-running after a new Capture is a file dropped into the archive
directory and one command; it is never a code change, because a tool that
needed editing per ride would stop being run.

The samples a fitter regresses come out of the estimator itself rather than
being recomputed. `tests/host/replay.c --samples` walks a Capture through the
real `wf_est_*` code and prints differences of the estimator's own totals, so
the offline figures and the Monitor's figures come from one integrator. A
Python energy-and-distance integrator would have been a second answer to a
question ADR-0004 spent four tickets giving one answer to, and the two would
have drifted - which is ADR-0002's argument, one level up.

**A constant that has not been fitted yet says so.** The archive is usually
behind the ambition: today it holds one 47-second parking-lot ride, and the
Consumption fit refuses over it. The refusal is the deliverable in that case.
`WF_FIT_FITTED` is 0, the placeholder coefficients are marked UNFITTED, the
evaluator produces nothing at any speed, and the header records the tool's
own words for why. That is the same marking `WF_EST_LIMP_POINT_V` carries as
provisional and `WF_CTRL_CURRENT_LSB_PER_A` carries as unsettled. Emitting
confident-looking coefficients from data that cannot support them would be
the one failure mode this whole discipline exists to prevent.

**Extrapolation is a property the constants have to carry.** A fit is only
supported over the speeds the data covered, so the range travels with the
coefficients and the evaluator flags a query outside it rather than answering
silently. A caller that means to advise a rider - issue #20 - has to be able
to ask "is this speed one the fit actually covers" and get an honest answer.
