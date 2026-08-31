---
status: accepted
---

# The estimator is a pure seam and time is a parameter

Everything the Monitor works out - Remaining Energy, the Coulomb Count,
Distance, Consumption, Range, and the State of Health figures still to come
- is computed in `main/wfest`, a module that takes decoded fields plus
persisted state and returns estimates. No I/O, no BLE, no display, no globals,
no `esp_*`, no FreeRTOS, no logging, no allocation.

Distance is inside that seam and not beside it, though it is about the road and
not about the Pack. Consumption is energy divided by Distance and Range follows
from Consumption, so a distance the replay harness cannot reproduce is a Range
the replay harness cannot reproduce - and a Distance computed somewhere with a
clock in it would be exactly that.

Consumption's rolling window is inside the seam for the same reason and one
more: a window is state, and state on the display is state a replay cannot
reach. It is indexed by metres rather than by time, which keeps it a function
of the recorded stream even though it behaves like a filter. It is fused from
two sources the same way charge is, integration Anchored to something slower
and more trustworthy, so it is the same code shape as well as the same seam.

The load average behind Sag is inside the seam on the same terms, and it is the
one filter here that is indexed by time rather than by metres — because load is
a time phenomenon and a rider holding the throttle open at a standstill is
still sagging their Pack. It stays replayable all the same: its step is
`dt / TAU` on a `dt` that came out of the record, so it is still a function of
the recorded stream and not of when anything was read. It is what lets the Limp
Point move with riding style without moving with individual frames.

The weakest Cell's divergence is in the seam on the same terms and is indexed a
third way again: by BMS answers, because a Cell imbalance is a fact the BMS
states once per poll and nothing else observes at all. Answers, not seconds, so
it needs no clock; answers, not metres, so a Pack that is standing still is
still being watched. It stays replayable for the same reason the other two do -
its step is one per record in the file - and it is applied at read time like
Sag, so no accumulator is ever re-based because a Cell moved.

Range is in the seam too, and it is the one figure there that holds no state of
its own: it is Remaining Energy divided by Consumption, computed where both
already are, guarded by a floor on the divisor. That is deliberate rather than
incidental. The steadiness a rider needs from it is already bought by
Consumption's window, one level down; a second filter on the quotient would buy
nothing that is not already there and would cost the thing the figure exists
for, which is reacting when the riding changes. So the only thing on the
handlebars is a `%.0f`.

And no clock. Every feed function takes an explicit `t_ms` from the record it
was handed. The estimator never asks what time it is.

## Why

The clock is the load-bearing half. ADR-0001 keeps Captures raw so that a ride
can be re-analysed with decoding that did not exist when it was taken, and that
is only worth anything if replaying a ride produces what the ride produced.
An estimator that reads a clock cannot do that: the same Capture would give a
different answer depending on how fast the machine walked the file. With time
as a parameter, the answer is a function of the bytes alone, and "replaying a
Capture produces the identical Remaining Energy curve every time" becomes an
assertion the test suite makes rather than a property we hope for.

The purity is the other half, and it is what makes ADR-0002's argument work one
level up. The decoders must not drift because they exist twice; the estimator
must not drift because it exists once and runs in two places - on the
handlebars and in `tests/host/replay.c`. That only stays true if there is
nothing in it that a host cannot run. One `esp_timer_get_time()` or one
`ESP_LOGI` and the offline harness needs a shim, the shim needs a decision, and
the decision is where the two copies start to differ.

Keeping it separate from `main/wfdecode` is deliberate too. Decoding answers
"what did the device say"; estimation answers "what does that mean for the
rider". The first is settled by evidence about a protocol, the second by a
model that will be wrong for a while - the Limp Point is assumed, the current
scale is uncertain by 19 %, the pack voltage curve is a straight line through
two points. Those churn at completely different rates, and a change to the
model must not be able to touch a decode.

Persisted state is a plain struct with an explicit byte encoding, and the NVS
read and write live in `main/est_store.c`, in the firmware. The estimator does
not know NVS exists. That is the same seam again: a host test restores state by
filling in a struct.

## Consequences

The caller owns everything. `main/capture.c` holds the lock, holds the state,
and chooses which clock to feed in - uptime, so the live screen works whether or
not a capture is running. Anything that wants an estimate takes a snapshot
through `cap_est_get()`.

Callers may not compute. `main/ui.c` formats what the estimator produced and
does no arithmetic on it, because arithmetic on the display is arithmetic no
replay can reach.

Floating point has to be pinned down, which is not free. Both builds compile
the seam with `-ffp-contract=off`: GCC is otherwise free to fuse a multiply and
an add into one rounding, which is a better answer and a different one
depending on what it chose to fuse. Accumulators are `double`, in a fixed
order, and an integration step is taken on a power-block frame and on nothing
else - so the step boundaries are a property of the recorded stream rather than
of what else happened to arrive.

Nothing the estimator concludes goes into a Capture, per ADR-0001. It goes to
the screen and to NVS, which is the Monitor's own scratchpad and not an archive
of the ride.
