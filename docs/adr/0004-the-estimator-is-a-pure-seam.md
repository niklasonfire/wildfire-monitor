---
status: accepted
---

# The estimator is a pure seam and time is a parameter

Everything the Monitor works out about the Pack - Remaining Energy, the Coulomb
Count, and the Consumption, Range and State of Health figures still to come -
is computed in `main/wfest`, a module that takes decoded fields plus persisted
state and returns estimates. No I/O, no BLE, no display, no globals, no
`esp_*`, no FreeRTOS, no logging, no allocation.

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
