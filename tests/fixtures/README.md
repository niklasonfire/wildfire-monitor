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

## `cap0002.wfl`

The second Capture, taken on 2026-09-01 (`unix_start=1788245939`,
`note=wildfire idf v6.1`). **The archive's first road ride**: 38.6 minutes and
21.28 km, with a GPS track recorded alongside it - the first instrument outside
this project any entry in `docs/field-table.md` has been held against.

It is the ride three constants were measured from, which is why it is a
fixture: `WF_CTRL_GEARING_CORRECTION` (this Controller is configured for a
4.000:1 reduction the bike does not have, so every speed it reports is 30 %
low), `WF_CTRL_ODO_METRES_PER_COUNT`, and `WF_CTRL_CURRENT_LSB_PER_A` at 4.25.
The check worth remembering is in the `.expect`: 2972 rpm corrects to
85.88 km/h against a GPS peak of 85.7-86.4 over the same twenty seconds.

Where cap0007 is parked and 47 s long, this one moves everything. The State of
Charge falls 65.9 % to 47.3 %, the line current reaches 71.8 A and -13.6 A of
regen, Internal Resistance is measurable for the first time (59.5 mOhm over 64
accepted load steps), and 904.57 Wh over 21.28 km is the archive's first real
Consumption figure. `main/wfest/wf_fit.h` is fitted from this ride and no
other.

82464 records: 81777 Controller frames, 1546 BMS responses, 240 events, 447
telemetry, 46377 IMU samples, 0 dropped.

Three things it is worth knowing this fixture does **not** hold:

- **No Markers.** It predates the Marker protocol the test rides use, so
  `marker_records` is 0 and nothing here exercises Marker handling.
- **Ten rejected BMS responses**, and they are the honest count rather than a
  decode defect: this ride lost its BMS link 57 times on a ~40 s cycle, and a
  response cut short by a link going down is one the decoder must refuse. That
  is issue #34, and this file is its evidence.
- **A pulled `.wfl`, not a rebuilt one.** Unlike cap0007 it came off the board
  over the Wi-Fi readout rather than down a serial line, so it carries its own
  `duration_ms` and needs no `dump2wfl.py` step. There is no
  `captures/cap0002_dump.log`.

### What it cost the assertions

Two of this runner's invariants were parked-Capture thresholds and this ride
broke both. Neither was a bug, and neither was widened by picking a number that
made it pass:

- **The Cell block and the aggregate registers are not one snapshot.** The
  whole Cell array shifts common-mode against registers 40, 43 and 44 by up to
  90.6 mV per Cell, and 90.6 mV times 28 Cells is 2.537 V - exactly the worst
  Pack-sum gap in the ride. One skew explains both the sum check and the
  highest/lowest check, and it is *not* a function of current (92 mV below 5 A,
  99 mV above 40 A), so `CELL_SNAPSHOT_SKEW_MV` budgets it per Cell and
  `cell_skew_mv_max` is pinned in both `.expect` files so the budget cannot
  hide a decode regression.
- **The Controller and the BMS are sampled up to one poll apart.** Their Pack
  voltages differ by up to 4.90 V here, flat across current bands, because a
  load step can land entirely between the two readings. The budget is now twice
  the ride's own measured Sag, so a parked Capture keeps the 2 V this check has
  always used and cap0007 is held to exactly what it was.

It also found a third thing, in the build rather than the data: `wf_fit.h` was
not a prerequisite of `wfest.o`, so `make fit` followed by `make test` measured
a fit nobody had compiled. The Makefile names it now.

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
