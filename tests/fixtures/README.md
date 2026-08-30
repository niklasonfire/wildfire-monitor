# Replay fixtures

Real Captures, in the archive format the Monitor writes, replayed by
`make test`. Each is a `.wfl` holding exactly what the Controller and the BMS
sent (ADR-0001), next to a `.expect` file listing what replaying it has to
produce.

## `cap0007.wfl`

The seventh Capture, taken on 2026-08-30 (`unix_start=1788080992`,
`note=wildfire idf v6.1`). A 47 s parking-lot ride: ignition on, both links up
the whole time, rolling at 4-7 km/h with no marked manoeuvres. Peak decoded
speed is 5.48 km/h, the Pack sat at 105.1-105.5 V and the State of Charge did
not move off 66.7 %.

This is the ride the Daly register map in `docs/fardriver-fields.md` was
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

`.expect` is `name value` lines, compared to within ±0.05, `#` starts a
comment. The names are the measurements `tests/host/replay.c` collects
(`ctrl_frames_bad`, `pack_v_min`, `speed_kmh_max`, …); a name it does not
measure is a failure, so a typo cannot assert nothing. Values that hold for
*any* Capture - checksums verifying, cells inside lithium's voltage range,
pack voltage matching the sum of the cells - are asserted by the runner
itself and do not belong in a `.expect` file.
