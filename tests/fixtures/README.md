# Replay fixtures

Real Captures, in the archive format the Monitor writes, replayed by
`make test`. Each is a `.wfl` holding exactly what the Controller and the BMS
sent (ADR-0001), next to a `.expect` file listing what replaying it has to
produce.

## `cap0007.wfl`

The seventh Capture, taken on 2026-08-30 (`unix_start=1788080992`,
`note=wildfire idf v6.1`). A 47 s parking-lot ride: ignition on, both links up
the whole time, rolling at what the ride notes call 4-7 km/h with no marked
manoeuvres. Peak decoded speed is 7.31 km/h - just outside those notes, which
are a rider's estimate rather than an instrument; see the `.expect` file. The
Pack sat at 105.1-105.5 V and the State of Charge did not move off 66.7 %.

It is also the ride that caught the `motion` block being declared at one frame
type instead of eight (#13). That peak read 7.11 km/h until then, not because
the decoding of rpm was wrong but because seven frames in eight were being
skipped and the fastest moment of the ride fell in one of them.

Every speed here is 30 % higher than it was before cap0002: that ride's GPS
track measured `WF_CTRL_GEARING_CORRECTION`, and this Controller is configured
for a reduction the bike does not have. The peak read 5.64 km/h until then.

This is the ride the BMS register map in `docs/field-table.md` was
decoded from, which is why it is the fixture: everything CONFIRMED there is
something this file has to keep producing.

2668 records: 1674 Controller frames, 34 BMS responses, 8 events, 10
telemetry, 942 IMU samples, 0 dropped.

Rebuilt from `captures/cap0007_dump.log` by `scripts/dump2wfl.py` (`make
fixtures`), because that console dump is the form of the ride this repository
has: a serial line was all that was attached when it was read off the board.
The dump carries every record's raw bytes, so the archive file is the
same bytes back; two header fields are not in the dump and are not invented:
`duration_ms` stays 0 (the archive's "unknown", which is also what a Capture
whose bike cut power carries) and `hdr_len` is written as the size the
firmware has always written.

## Adding another

Drop the `.wfl` in here with a `.expect` beside it and a paragraph above. The
runner discovers `*.wfl` in this directory; it needs no change, and a fixture
without a `.expect` fails rather than passing quietly.

Then run `make fit` and commit `main/wfest/wf_fit.h` with it. This directory is
also the archive issue #19's fit is taken over: `./replay --samples` walks
every `.wfl` here through the real estimator and prints Consumption against
speed span by span, and `scripts/fit_consumption.py` regresses those. It
discovers the Captures the same way this runner does, so a new ride reaches
the fit by being dropped in here and by nothing else. `make test` fails if
`wf_fit.h` has drifted from what the archive now produces.

Today every span of every Capture here is excluded - the one ride we hold
crawls 21.7 m at under 8 km/h - and the fit refuses. That is the honest answer
until the calibration ride (#6) lands, and the generated header says so in
those words.

`.expect` is `name value` lines, compared to within ±0.05, `#` starts a
comment. The names are the measurements `tests/host/replay.c` collects
(`ctrl_frames_bad`, `pack_v_min`, `speed_kmh_max`, …); a name it does not
measure is a failure, so a typo cannot assert nothing. Values that hold for
*any* Capture - checksums verifying, cells inside lithium's voltage range,
pack voltage matching the sum of the cells, distance never running backwards -
are asserted by the runner itself and do not belong in a `.expect` file.

Every Capture is replayed once more with one Cell of every BMS response
rewritten 300 mV under the rest of its Pack, and re-checksummed - the whole Cell
block is recomputed with it, so the highest, lowest, average and delta registers
still describe the array and the response is one a BMS could have sent. Every
Pack recorded here is healthy, which is what proves half of #18's criterion (a
healthy Pack produces neither the Range clamp nor the divergence warning) and is
exactly why the other half has to be synthesised. The rewritten run is held to
the same invariants as the real one, so a synthesis that produced an impossible
Pack fails rather than passing.

Every Capture is also replayed twice more with its Odometer frames rewritten
to a synthesised count ramp, started either side of the u16 wrap, and the two
distance curves have to come out bit-identical. No ride we hold crosses the
wrap - it happens once every 8520 km - so this is how a real ride's timing and
speeds get to exercise it. Nothing is written back: the rewrite happens in a
scratch buffer on the way into the parser, and the `.wfl` on disk stays the
bytes the devices sent (ADR-0001).
