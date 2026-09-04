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

**Track**:
A GPS recording taken alongside a Capture, in its own file. The only
instrument this project has that stands outside it, which is why it is the
only thing that can move a Field Table entry to `proven`. It measures speed and
distance against the ground and nothing else — never volts, amps, charge or
temperature.
_Avoid_: GPS log, GPX, trace, route

**Manoeuvre**:
A deliberate thing the rider did that a measurement is taken over — a rest, a
steady hold, a full-throttle launch, a rollout, a regen deceleration. A Marker
labels one by hand; a Track lets one be found without the rider labelling
anything.
_Avoid_: step, segment, span (a span is the fit's unit of road, not this),
event

**Rollout**:
A Manoeuvre: throttle shut, brake untouched, the bike left to slow down on its
own. The only measurement of the bike's drag that does not go through the
current scale, so the only independent check on Consumption.
_Avoid_: coastdown in prose (fine in code), freewheel, glide

**Marker**:
A rider-pressed timestamp inside a Capture, marking the start or end of a
Manoeuvre. What used to be the only thing separating an experiment from a blob
of frames; a Track does the same job without a button, and Markers are now
optional rather than load-bearing.
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

**Service Mode**:
The Monitor stops listening and puts itself where a phone can reach it: it
joins the strongest network it knows, or, knowing none or reaching none, puts
up its own access point instead. Either way it serves the Captures and the
settings page over HTTP; only over a joined network does that page also offer
an update, because only there is there an upstream to fetch one from, and both
the check and the install are asked for on the page rather than on the panel.
The mode itself looks for nothing on its way in. Entered
from the menu, refused while a Capture is running, and left by a reboot —
Bluetooth is down throughout, so there is no other way back to capturing.
_Avoid_: readout mode, update mode, webdump, AP mode, download mode, OTA mode

**Fallback**:
Service Mode's access point, and the rule that reaches it: any failure to join
lands there rather than stopping. It is what keeps a rotated hotspot key from
costing a cable, because the settings page that repairs the list is served
from the failure the list caused. A Monitor that has never been told a network
is the same case and takes the same path, without a scan first.
_Avoid_: AP mode, soft AP in prose (fine in code), recovery mode
