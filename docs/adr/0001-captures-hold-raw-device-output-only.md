---
status: accepted
---

# Captures hold raw device output only

A Capture stores exactly the bytes the Controller and the BMS sent, and never a
value the Monitor computed from them — no state of charge, no consumption, no
range. The Monitor still computes all of those and shows them live; it just
does not archive its own answers.

## Why

Our decoding is wrong today and will be wrong again tomorrow. Raw Captures can
be re-decoded when we learn better; derived values freeze the bug that produced
them into the archive permanently.

This is not theoretical. Pack voltage, line current and the 50 Ah Rated
Capacity were all recovered from `cap0006` and `cap0007` using decoding that
did not exist when those rides were taken. Had the Monitor logged its own
conclusions instead, all three would have been lost with the rides.

Calibration needs it too. The current scale is uncertain by 19%, and settling it
means fitting integrated current against the BMS's state-of-charge delta across
a whole ride — then re-running the corrected scale over every ride already
recorded. Neither is possible against an archive of derived numbers.

## Consequences

The decoding logic must exist twice: once on the Monitor for the live screen,
once offline for the archive. That duplication is the price of this decision,
and it will drift unless actively prevented — see ADR-0002.

Captures are larger than they would be otherwise, against a 5.7 MB flash
partition holding roughly two hours of riding. Accepted: rides are short, and
storage is the cheapest thing we can spend to keep past rides re-analysable.
