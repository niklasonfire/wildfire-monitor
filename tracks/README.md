# Tracks

GPS recordings taken alongside a ride, one file per Capture, named for the
Capture they belong to: `cap0008.gpx` beside `cap0008.wfl`.

A Track is the only instrument this project has that stands outside it. It
measures speed and distance against the ground, and that is the whole of what
it settles — see [`docs/gps.md`](../docs/gps.md) for what that reaches and what
it cannot touch.

Per ADR-0007 a Track is never merged into a Capture and its clock offset is
never stored: `scripts/gps.py` measures the offset from the data each time,
because the Monitor's RTC ran 12 s slow over cap0002 and nothing on the bike
would have known.

```bash
scripts/gps.py tracks/cap0008.gpx tests/fixtures/cap0008.wfl --mass-kg 195
```

## Recording

1 Hz, raw, unsmoothed, with Doppler speed if the app records it. Keep recording
through the stationary minute at each end of the ride.

## Not in git yet

cap0002's Track is the one this project has already used — it measured
`WF_CTRL_GEARING_CORRECTION`, `WF_CTRL_ODO_METRES_PER_COUNT` and the clock
offset — and it is not here. It was recorded on a phone and analysed by hand
before this directory existed. If it turns up, it belongs here as
`cap0002.gpx`, and `scripts/gps.py` should reproduce those three numbers off
it.

## Privacy

A Track is a list of places the rider was, at times they were there, which is a
different kind of file from a Capture. `captures/README.md` records that the
BLE scan tables were stripped out of this repository's history before it was
published, for the same sort of reason. Think before committing one, and prefer
a ride that starts and ends somewhere that is not home.
