# GPS Tracks

A Capture is the bike talking to itself. A **Track** is a second instrument
standing outside it, and it is the only reason anything in
[`field-table.md`](field-table.md) is `proven` rather than `consistent`.

This describes how to record one, what it settles, what it cannot touch, and
how `scripts/gps.py` turns one into a labelled ride.

## What a Track settles

Exactly what the Field Table says, and it is worth being blunt about the edges
because this is now load-bearing:

- **It measures speed and distance against the ground.** That reaches
  `cur_rpm`, `odometer_raw`, and the constants that turn either into metres -
  `WF_CTRL_GEARING_CORRECTION` and `WF_CTRL_ODO_METRES_PER_COUNT`.
- **It says nothing about volts, amps, charge or temperature.** Those need a
  meter. No amount of GPS makes `WF_CTRL_CURRENT_LSB_PER_A` more certain.
- **One exception, by subtraction:** a rollout measures the bike's drag with
  the electrical chain switched off, which reaches the two coefficients in
  `main/wfest/wf_fit.h` without an ammeter. See [Rollouts](#rollouts).

## Why this replaces the Marker protocol

Rides #6, #7 and #8 are all written around *"press B immediately before and
after each step, and count your presses"*. That is close to impossible at speed
in gloves, and #36 is the firmware half of the problem: a B press held past
800 ms opens the menu and writes no Marker at all, silently. A Marker sequence
is positional, so one lost press does not cost one step - it misattributes
every step after it.

A Track needs the rider to do nothing. The manoeuvres label themselves:

| Manoeuvre | What it looks like without a Marker |
| --- | --- |
| Rest | ground speed under 1 km/h for 30 s |
| Steady hold | speed inside ±2.5 km/h of its own mean — or ±5 %, whichever is larger — for 45 s |
| Launch | rising from under 3 km/h with `throttle_raw` at the stop |
| Rollout | falling with the throttle shut and the current at nothing |
| Regen | falling with the throttle shut and the current negative |

The last two are the same shape in the speed curve and are told apart only by
the Capture's current, which is why the Track has to be aligned before either
can be labelled. **The Track says what the bike did; the Capture says what the
rider's hand was doing.**

Markers still work and are still read. Nothing here needs them.

## Recording one

- **1 Hz, raw, unsmoothed.** cap0002's track was on a 6 s grid and that grid,
  not the Controller, was the dominant scatter in the Odometer measurement. A
  6 s sample cannot resolve a launch at all.
- **Doppler speed if the app will record it.** It comes off the carrier phase
  and is good to a few cm/s; differenced positions inherit the position error
  twice. `gps.py` uses `<speed>` wherever the file puts it - GPX 1.0's element,
  or any extension namespace - and falls back to differencing, saying which it
  used.
- **Keep recording through the stationary bookends** at each end of the ride.
- **Write down what the log cannot know:** bike + rider + luggage mass, tyre
  pressures, air temperature, wind and its direction, which stretch the
  rollouts were ridden on, and anything the bike's own dashboard showed.

Put the file in `tracks/`, named for the Capture it belongs to.

## The clock

The Monitor's RTC is disciplined by nothing. It ran **12 s slow** against the
GPS over cap0002, and 12 s at 86 km/h is 290 m of misattributed road. So the
offset is *measured every time*, by cross-correlating the Track's speed against
the Controller's raw `cur_rpm` at 1 Hz over integer-second lags. That is how
the 12 s was found: r = 0.9972, against 0.9960 and 0.9938 a second either side.

Raw rpm and not decoded speed, deliberately - a correlation is scale-invariant,
and using decoded speed would put a constant the same Track is about to
re-measure inside the measurement of the clock.

**This needs the ride to change speed.** A constant cruise correlates with
itself at every lag and the peak means nothing; the tool refuses rather than
shifting a whole Capture by a number it does not believe. A full-throttle
launch is the sharpest feature a bike can put in a speed curve, so put one
early in the ride.

## Rollouts

Throttle shut, no brake, rolling. The only forces left are rolling resistance
and drag:

    dv/dt = -(A + C v²)

`A` is in m/s² and `C` in 1/m; with the bike's mass they become a rolling force
and a drag area, and a force in newtons divided by 3.6 is watt-hours per
kilometre - the same units as `WF_FIT_A_WH_PER_KM` and
`WF_FIT_C_WH_PER_KM_PER_KMH2`, which today are fitted from cap0002 alone and
have never been checked against anything outside this project.

**They are not the same number.** A rollout measures the mechanical force at
the tyre. The fit measures what came out of the Pack, which is that force plus
everything the driveline, the motor, the Controller and the Monitor's own draw
lost on the way. The rollout is the floor, the fit is the bill, and the ratio
is a drivetrain efficiency nobody has measured yet.

The fit is done on the integrated form rather than on `dv/dt`, because
differentiating a 1 Hz GPS speed and regressing the result fits noise.

### Ride them in pairs

A constant gradient adds `g·sin(θ)` to `A` with the sign of travel. **One part
in a hundred of slope is 0.098 m/s², against a whole rolling term of about
0.15** - a lone run down a grade you cannot feel is wrong by most of the
answer. Two runs over the same road in opposite directions average it away
exactly, and wind out of `C` to first order.

So: same stretch, back to back, alternating direction, at least two pairs.
Coast from as high as the road allows down to about 20 km/h - the low end is
where the rolling term separates from the drag term. GPS altitude is not used
for this and should not be trusted for it; it is the worst channel a receiver
has.

## Running it

```bash
scripts/gps.py tracks/cap0008.gpx tests/fixtures/cap0008.wfl --mass-kg 195
```

Useful flags:

| Flag | What for |
| --- | --- |
| `--mass-kg KG` | bike + rider + luggage; without it a rollout stops at `A` and `C` |
| `--rho KG_M3` | air density, default 1.20 |
| `--csv OUT` | the merged 1 Hz trace: position, ground speed, rpm, throttle, current, volts, Odometer, State of Charge, one row a second |
| `--manoeuvres OUT` | the manoeuvre table as CSV |
| `--offset S` | use a known clock offset instead of measuring one |
| `--track-only` | describe a Track with no Capture beside it |

The report ends with **AGAINST THE RIDE PROTOCOL**, which counts what the ride
actually contained against what #6 asks for and names what is missing. That is
the thing to read first after a ride: it says whether the ride is worth
analysing before any time is spent analysing it.

## What to do after a ride

Per #9, and none of it changes:

1. Pull the Capture over the Wi-Fi readout. Drop the Track in `tracks/`.
2. Run `scripts/gps.py` and read the protocol checklist.
3. Re-run the offline decoder over the **whole archive**, not just the new
   file. A decode fix found today applies to every ride ever taken (ADR-0001).
4. Update the Field Table with whatever the ride closed, including the new
   Confidence. Per ADR-0002 that is the single edit.

## Thresholds

Every number the tool decides anything with is in `Rules` in `scripts/gps.py`,
with the reason it is that and not something else, in one place and out of the
functions that apply them.

One is worth knowing about here: the steady-hold tolerance is **tighter** than
`fit_consumption.py`'s steady rule, deliberately, and `tests/test_gps.py`
asserts it stays that way. They are different questions. The fitter asks
whether a span of road put enough energy into acceleration to bias a Wh/km
figure; this asks whether a rider deliberately held a speed. A stretch that is
not a hold is very often still a span the fit will happily take, so **finding
no holds is not finding no fittable road**.
