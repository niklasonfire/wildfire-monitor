# Wildfire Monitor

Firmware for a Blacktea Motorbikes Wildfire electric motorbike. It listens to
the bike's two devices, records what they say, and turns that into battery
capacity monitoring and range estimation of the standard a rider expects from a
2026 EV.

## Language

### The bike

**Controller**:
The motor controller. Reports voltage, current, speed, throttle and motor
temperature, unprompted and often.
_Avoid_: MCU, Fardriver, ESC

**BMS**:
The battery management system. Reports per-cell voltages, pack current, state
of charge and pack temperature, and answers only when asked.
_Avoid_: Daly, battery monitor

**Monitor**:
Our own device: the thing that listens to the Controller and the BMS, records
them, and shows the rider a screen.
_Avoid_: M5Stick, logger, dongle

**Pack**:
The traction battery as a whole. Distinct from the Monitor's own internal
battery, which is never called a pack.
_Avoid_: battery (ambiguous — could be either)

**Cell**:
One series element of the Pack. The Pack's usable bottom is set by its weakest
Cell, not by its average.

### Charge and energy

**State of Charge**:
The fraction of the Pack's charge remaining, as a percentage. The BMS is the
authority on it.
_Avoid_: SOC in prose (fine in code and tables), charge level, battery percent

**State of Health**:
How much the Pack has degraded from new, measured both as capacity lost and as
internal resistance gained.
_Avoid_: SOH in prose, degradation, wear

**Rated Capacity**:
The Pack's nameplate capacity when new, in amp-hours.
_Avoid_: capacity (unqualified), nominal capacity

**Usable Capacity**:
The charge actually available to the rider — from full down to the Limp Point,
not down to zero. Always less than Rated Capacity, and shrinks with age.
_Avoid_: real capacity, actual capacity

**Remaining Energy**:
Energy left before the Limp Point, in watt-hours. Not the same as State of
Charge, because energy depends on voltage and voltage falls as the Pack
empties.
_Avoid_: remaining charge, energy left

**Coulomb Count**:
Charge tracked by integrating current over time. Precise in the short term,
drifts over hours, and so is always corrected against an Anchor.
_Avoid_: current integration, amp-hour counting

**Anchor**:
A slow, trustworthy measurement used to correct a fast, drifting one. The BMS's
State of Charge anchors the Coulomb Count; the Odometer anchors integrated
speed.
_Avoid_: reference, correction, ground truth

### Riding and range

**Distance**:
How far the Monitor has seen the bike travel, in metres. One figure, fused from
two sources that are each insufficient alone: integrated speed for resolution
between Odometer counts, the Odometer as the Anchor for truth. Not the same
thing as the Odometer, which is the bike's own lifetime total.
_Avoid_: trip, mileage, odometer (that is the Anchor, not this)

**Consumption**:
Energy used per unit Distance, in watt-hours per kilometre. The figure that
converts Remaining Energy into Range.
_Avoid_: efficiency (it is the inverse), economy, usage

**Range**:
Distance still rideable at the current Consumption before reaching the Limp
Point. Always relative to a riding style, never absolute.
_Avoid_: distance to empty, remaining distance

**Limp Point**:
The moment the Controller cuts speed to walking pace because Pack voltage fell
below its threshold. The true end of usable range — roughly a kilometre of
crawling follows it.
_Avoid_: empty, cutoff, zero, cutback

**Sag**:
The drop in Pack voltage caused by current draw, proportional to Internal
Resistance. Why the Limp Point arrives sooner under hard acceleration and why
easing off pushes it back.
_Avoid_: voltage drop, dip

**Internal Resistance**:
The Pack's opposition to current, seen as Sag per amp. Rises as the Pack ages,
which makes it a State of Health signal independent of capacity.
_Avoid_: impedance, ESR

**Odometer**:
The Controller's cumulative distance count. Coarse and it wraps, but it does
not drift, so it Anchors distance.
_Avoid_: trip meter, mileage

### Recording

**Capture**:
One recording session, from starting to stopping the Monitor. Holds only what
the devices actually said, never anything the Monitor worked out.
_Avoid_: log, session, dump, recording

**Marker**:
A rider-pressed timestamp inside a Capture, marking the start or end of a
deliberate manoeuvre. What separates an experiment from a blob of frames.
_Avoid_: annotation, event, flag

**Field Table**:
The single description of what every byte the two devices send actually means,
including how confident we are in each entry. Everything that decodes — the
Monitor, the offline tools, the documentation — is generated from it.
_Avoid_: protocol spec, field map, decoder

**Confidence**:
How well a Field Table entry is established: taken on trust from elsewhere,
consistent with our own Captures, or proven against an independent measurement.
_Avoid_: certainty, status, reliability

### Modes

**Readout Mode**:
The Monitor stops listening, puts up its own access point, and serves the
Captures over HTTP so a phone or a laptop can pull them off. It serves the
settings page too, which is where the networks Update Mode may join are edited
without a cable. Bluetooth is down throughout, so the only way back to
capturing is a reboot.
_Avoid_: download mode, webdump, AP mode, dump mode

**Update Mode**:
The Monitor stops listening, joins a hotspot it knows, and installs a
published firmware into its spare app slot. Shares Readout Mode's shape — one
radio at a time, entered from the menu, left by a reboot — and, like it, is
refused while a Capture is running.
_Avoid_: OTA mode, upgrade mode, flashing
