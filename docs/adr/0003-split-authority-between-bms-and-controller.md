---
status: accepted
---

# The BMS owns charge, the Controller owns power

Both devices report pack voltage and current, and their numbers disagree. The
BMS's state of charge is authoritative for how much charge remains. The
Controller's voltage and current drive everything instantaneous: power,
consumption, and the internal resistance estimate.

## Why

They are good at different things. The BMS answers once per second but its
state of charge is already calibrated and already corrected against resting
voltage — and at 0.1% resolution on a 50 Ah pack, that is 0.05 Ah, finer than
anything downstream needs. The Controller pushes voltage and current at
**5.2 Hz**, which is still the fastest source there is and the only one that
sees an acceleration transient at all. Polling the BMS faster would not fix
that; it averages internally.

An earlier version of this decision said 35 Hz, and that was wrong by about
seven times. 35.5 Hz is the Controller's *total* frame rate across all 55 of
its frame types; pack voltage and line current arrive in a block of eight of
those types, which is 244 frames in the 47.1 s of cap0007 — 5.2 Hz. The
decision itself is unchanged: 5.2 Hz is still five times the BMS's ~1 Hz poll,
so the Controller is still where instantaneous power comes from and the BMS is
still where charge comes from. What changes is what can be built on it. An
acceleration transient is resolved to roughly 200 ms, not 30 ms, so the
internal resistance estimate from load steps has to be designed for a sample
every fifth of a second.

So the Controller's stream is integrated for short-term resolution and Anchored
to the BMS's state of charge for long-term truth. The same pattern applies to
distance: integrated speed for resolution, Odometer as the Anchor.

Speed arrives at 5.2 Hz too, from its own block of eight frame types — the same
shape as the power block, and the same correction, found the same way. The
Field Table declared that block at one type until #13, which had road speed
updating every 1.5 s. Integrating a speed that stale is a distance error, and
distance is what Consumption and Range are divided by, so the two rates being
equal is not a coincidence worth glossing over: one sample of voltage, current
and speed arrives together every ~193 ms, and everything below is built on that
being one sample.

We compute from raw voltage and current rather than consuming the numbers both
devices already offer. The Controller reports its own state of charge at 1%
resolution and its own consumption quantised to 4 Wh/km, both by unknown and
unverifiable methods. Raw voltage and current at 5.2 Hz is strictly more
information, and it is the only route to state of health and to a range figure
we can defend.

## Consequences

Everything now rests on the Controller's current scale factor, which is
uncertain by 19% — upstream sources say 4 LSB per amp, regression against the
BMS says 4.77. Current scale is the gain on every watt-hour ever counted, so
that error propagates undiminished into range. Closing it is the single most
valuable measurement outstanding; it is Ride 1, tracked as an issue under the
`test-ride` label.
