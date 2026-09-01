---
status: accepted
---

# A Track is a second instrument beside the Capture, not part of it

A GPS Track is recorded alongside a ride and stored as its own file in
`tracks/`, named for the Capture it accompanies. It is never merged into the
`.wfl`, and its alignment to that Capture is **measured from the data every
time it is used**, never stored and never taken from either device's clock.

## Why

**A Track is not device output.** ADR-0001 says a Capture holds exactly the
bytes the Controller and the BMS sent. A phone's opinion of where the bike was
is neither, so it does not belong inside one — and the Monitor could not write
it there anyway, since nothing on the bike can see the phone.

**A stored alignment would be a derived value in the archive**, which is the
one thing ADR-0001 exists to keep out. The Monitor's RTC is disciplined by
nothing: it ran 12 s slow against the GPS over cap0002, and 12 s at 86 km/h is
290 m of road attributed to the wrong place. Writing that 12 s into the file
would freeze one afternoon's answer into the archive permanently, and every
re-analysis afterwards would inherit it without being able to see it. Measuring
it costs a cross-correlation and about a second of CPU.

**The two files answer different questions and degrade differently.** A Track
can be lost, re-exported at a different sample rate, or replaced by a better
one off the same phone, without the Capture changing by a byte. A Capture is
re-decodable forever. Coupling them would make each only as good as the other.

## Consequences

**A ride has to contain a sharp speed change**, or the alignment cannot be
measured and `scripts/gps.py` refuses rather than guessing. A constant cruise
correlates with itself at every lag. The ride protocol therefore asks for a
full-throttle launch early in the ride, which is the sharpest feature a bike
can put in a speed curve — it is now a requirement and not just a measurement.

**The Marker protocol becomes optional.** Rides #6, #7 and #8 are written
around counting button presses, and #36 is that button losing them silently. A
Track finds rests, holds, launches, rollouts and regen decelerations from the
speed curve and the Capture's own throttle and current, so nothing downstream
depends on a rider pressing anything. Markers are still recorded and still
read where they exist.

**Tracks are not fixtures.** `tests/fixtures/` is replayed by `make test` and
by the Consumption fit, and both discover `*.wfl`. A Track has no place in
either, so it lives in `tracks/` and `tests/test_gps.py` synthesises what it
needs.

## What this does not change

ADR-0001 stands unaltered: this adds a file beside the archive, it does not put
anything new inside it. ADR-0005 stands too — the fit still happens offline and
still comes out of the estimator, and a rollout measured here is a check on its
coefficients from outside, not a second way of producing them.
