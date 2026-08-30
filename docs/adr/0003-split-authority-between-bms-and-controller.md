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
anything downstream needs. The Controller pushes voltage and current at 35 Hz,
which is the only source fast enough to see an acceleration transient. Polling
the BMS faster would not fix that; it averages internally.

So the Controller's stream is integrated for short-term resolution and Anchored
to the BMS's state of charge for long-term truth. The same pattern applies to
distance: integrated speed for resolution, Odometer as the Anchor.

We compute from raw voltage and current rather than consuming the numbers both
devices already offer. The Controller reports its own state of charge at 1%
resolution and its own consumption quantised to 4 Wh/km, both by unknown and
unverifiable methods. Raw voltage and current at 35 Hz is strictly more
information, and it is the only route to state of health and to a range figure
we can defend.

## Consequences

Everything now rests on the Controller's current scale factor, which is
uncertain by 19% — upstream sources say 4 LSB per amp, regression against the
BMS says 4.77. Current scale is the gain on every watt-hour ever counted, so
that error propagates undiminished into range. Closing it is the single most
valuable measurement outstanding; it is Ride 1, tracked as an issue under the
`test-ride` label.
