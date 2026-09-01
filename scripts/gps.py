#!/usr/bin/env python3
"""Read a GPS Track, align it to a Capture, and find the Manoeuvres in it.

WHY THIS EXISTS

A Capture is the bike talking to itself. Everything in it - rpm, the Odometer,
volts, amps - is the Controller's own opinion, and an opinion cannot check
itself. A Track is a second instrument: it measures distance and speed against
the ground, which is why cap0002's track is the only reason anything in
docs/field-table.md is `proven` rather than `consistent`.

It is also the way out of the Marker protocol. The test rides (#6, #7, #8) all
say "press B immediately before and after each step, and count your presses",
and that is close to impossible at speed in gloves - #36 is the button losing
presses silently, and a Marker sequence is positional, so one lost press
misattributes the whole tail of the ride. A Track does not need the rider to
do anything. A steady-speed hold *is* a flat stretch of the speed curve; a
full-throttle launch *is* a rise from zero with the throttle wide; a rollout
*is* a fall with the throttle shut. This tool finds them, so the ride protocol
becomes "ride these manoeuvres", not "ride these manoeuvres and label them".

WHAT A TRACK CAN AND CANNOT PROVE

Exactly what docs/field-table.md says, and the split is worth repeating here
because this tool is where it gets acted on. A Track settles anything that
converts to distance or speed - rpm, the Odometer, and the constants between
them and metres. It says nothing whatever about volts, amps, charge or
temperature. Those need a meter, and nothing this file prints should be read
as evidence about them.

The one exception is by subtraction, and it is the reason to ride rollouts: a
coastdown is the bike's drag measured with the electrical chain switched off,
so it reaches `a` and `c` in main/wfest/wf_fit.h without ever touching a
current scale. See COASTDOWNS below.

THE CLOCK

The Monitor's RTC is not disciplined by anything. It ran 12 s slow against the
GPS over cap0002, and a 12 s error over a ride that reaches 86 km/h is 290 m of
misattributed road, so the offset is measured every time rather than assumed.
The measurement is a cross-correlation of the Track's speed against the
Controller's raw `cur_rpm` at 1 Hz over integer-second lags, which is what
found the 12 s (r = 0.9972, against 0.9960 and 0.9938 a second either side).

That correlation needs the ride to *change speed*. A Track of a constant cruise
correlates with everything equally and the peak means nothing, which is why the
refusal below exists and why the ride protocol wants a full-throttle launch
early: a launch is the sharpest feature a bike can put in a speed curve.

SPEED, AND WHERE IT COMES FROM

Doppler speed if the Track carries it, differenced positions if it does not,
and the report says which. They are not equivalent. Doppler speed comes off the
carrier phase and is good to a few cm/s; differenced positions inherit the
position error twice and, on the 6 s sample grid cap0002's track was recorded
at, cannot resolve a launch at all. Record at 1 Hz, raw, unsmoothed.

COASTDOWNS

Throttle shut, no brake, rolling: the only forces left are rolling resistance
and drag, so

    dv/dt = -(A + C * v^2)

with A in m/s^2 and C in 1/m. Given the bike's mass those become a rolling
force and a drag area, and dividing by 3.6 turns a force in newtons into
watt-hours per kilometre - the same units as `WF_FIT_A_WH_PER_KM` and
`WF_FIT_C_WH_PER_KM_PER_KMH2`, which today are fitted from cap0002 and have
never been checked against anything outside this project.

They are not the same number and the report says so. A coastdown measures the
mechanical force at the tyre; the fit measures what came out of the Pack, which
is that force plus everything the driveline, the motor, the Controller and the
Monitor's own draw lost on the way. The coastdown is the floor, the fit is the
bill, and the ratio is a drivetrain efficiency nobody has measured yet.

The fit is done on the integrated form rather than on dv/dt, because
differentiating a 1 Hz GPS speed and regressing the result is a way of fitting
noise. Integrating v^2 by trapezoid and regressing (v0 - v) against elapsed
time and that integral uses each speed reading as it was measured.

GRADIENT AND WIND make a coastdown, and only a coastdown, worth riding in
pairs. A constant slope adds g*sin(theta) to A, with the sign of the direction
of travel, so two runs over the same road in opposite directions average it
away exactly. Wind does the same to C to first order. Runs are paired here by
opposing heading, and an unpaired run is reported as unpaired rather than
silently averaged with nothing. GPS altitude is not used for this: it is the
worst channel a GPS has, and two runs are better than a correction.

USAGE

  scripts/gps.py TRACK.gpx CAPTURE.wfl            # align, measure, segment
  scripts/gps.py T.gpx C.wfl --mass-kg 195        # ...and fit the coastdowns
  scripts/gps.py T.gpx C.wfl --csv merged.csv     # the 1 Hz merged trace
  scripts/gps.py T.gpx C.wfl --manoeuvres m.csv   # the manoeuvre table
  scripts/gps.py T.gpx C.wfl --offset 12          # skip the alignment search
  scripts/gps.py T.gpx --track-only               # inspect a Track alone

Python standard library only, like everything else in scripts/. The Capture is
read through scripts/wfl.py and the generated Field Table decoder, so this file
knows the archive format and the two frame envelopes through them and holds no
second description of what any byte means (ADR-0002).
"""
import argparse
import csv
import datetime
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wfl                                          # noqa: E402
from field_table import load as _load_fields        # noqa: E402

fields = _load_fields()

EARTH_R_M = 6371008.8       # IUGG mean radius; the ellipsoid is not worth it here
G = 9.80665                 # m/s^2
RHO_DEFAULT = 1.20          # kg/m^3, dry air near 20 C at sea level

# The rpm -> km/h constant out of wf_ctrl_speed_kmh(), so this tool can say
# what the Controller would have reported for a wheel geometry and compare it
# against the ground. The formula is hand-written in both languages by the
# Field Table's own admission - it needs the motion block and 0xaf at once -
# and this is its third copy. It is here rather than imported because the C is
# not importable, and it is asserted against the C in tests/test_gps.py.
CTRL_SPEED_K = 0.00376991136


class Refusal(Exception):
    """Something the data cannot support. Refusing beats printing a number."""


# ----------------------------------------------------------------- the Rules

class Rules:
    """Every threshold, with the reason it is that and not something else.

    Kept in one place and out of the functions for the same reason
    fit_consumption.py keeps its own: a rule that decides what a ride means
    should be readable without reading the code that applies it.
    """

    def __init__(self):
        # --- alignment
        # How far the Monitor's clock is allowed to be out. cap0002 was 12 s;
        # three minutes is room for an RTC that lost a battery, and it is
        # still short enough that the search cannot walk onto a different lap
        # of a circuit and correlate with that instead.
        self.align_max_lag_s = 180
        # Below this there is not enough ride to correlate. Two minutes of
        # overlap at 1 Hz is 120 points against a 361-lag search.
        self.align_min_overlap_s = 120
        # A cross-correlation always has a peak. This is the value below which
        # the peak is not evidence of anything, and the tool refuses rather
        # than shifting the Capture by a number it does not believe. cap0002
        # made 0.9972 with a 6 s track; a 1 Hz track of a ride with a launch in
        # it should beat that comfortably.
        self.align_min_r = 0.90
        # And a peak that its neighbours match is a plateau, not a peak: a
        # constant cruise correlates with itself at every lag. The peak has to
        # stand this far above the correlation a second either side of it.
        self.align_min_prominence = 0.002
        # A ride that never moves cannot time itself.
        self.align_min_moving_frac = 0.10
        self.align_moving_kmh = 5.0

        # --- resampling
        # A Track sample this far from the grid second it would fill leaves a
        # hole instead. 15 s covers a 6 s recorder and a tunnel; it does not
        # cover a receiver that lost the sky for a minute.
        self.grid_max_gap_s = 15.0

        # --- manoeuvres
        # Standing still. GPS speed does not sit at zero when a bike does, and
        # 1 km/h is under any receiver's noise floor while being well under
        # walking pace.
        self.rest_kmh = 1.0
        self.rest_min_s = 30
        # A steady-speed hold: #6 step 3 asks for 60 s at each of three
        # speeds. 45 s accepts one the rider cut short, and the tolerance is
        # the larger of an absolute and a fraction so that a hold at 70 km/h
        # is not held to a tighter relative standard than one at 20.
        #
        # This is much TIGHTER than fit_consumption.py's steady rule (3 km/h
        # or 20 %), and the two are not the same question. That one asks
        # whether a 50 m span put enough energy into acceleration to bias a
        # Wh/km figure; this one asks whether the rider deliberately held a
        # speed, which is a thing a person does with a throttle and not a
        # threshold on a residual. So a stretch that fails to be a hold here
        # is very often still a span the fit will happily take, and finding
        # no holds does not mean finding no fittable road.
        #
        # 2.5 km/h is what a rider can actually hold on a road bike with no
        # cruise control; cap0002 - 38 minutes of ordinary riding with no
        # protocol behind it - contains no window that meets it, which is the
        # detector working rather than failing.
        self.hold_min_s = 45
        self.hold_tol_kmh = 2.5
        self.hold_tol_frac = 0.05
        self.hold_min_kmh = 15.0
        # A launch. The throttle threshold is a fraction of the widest the
        # ride itself ever saw rather than ride0's 484, because the stop is a
        # mechanical position and a different rider or a different cable puts
        # it somewhere else.
        self.launch_from_kmh = 3.0
        self.launch_throttle_frac = 0.90
        self.launch_min_s = 4
        self.launch_min_delta_kmh = 20.0
        # A rollout. The throttle has to be shut - ride0 measured shut at 29
        # counts and the stop at 484, so a tenth of the ride's own maximum is
        # comfortably inside the closed position - and the current has to be
        # near nothing, which is what separates a rollout from a regen
        # deceleration that looks identical in the speed curve.
        self.coast_throttle_frac = 0.10
        self.coast_max_abs_a = 3.0
        self.coast_min_s = 8
        self.coast_min_delta_kmh = 15.0
        # A regen deceleration is the same shape drawing current backwards.
        self.regen_min_a = 5.0
        self.regen_min_s = 4
        self.regen_min_delta_kmh = 10.0
        # What counts as speeding up or slowing down, over a +-2 s window.
        self.accel_kmh_per_s = 0.5

        # --- coastdown pairing
        # Two runs cancel a gradient when they cover the same road the other
        # way. 30 degrees of slack is a real road that bends.
        self.pair_heading_tol_deg = 30.0
        # And they have to be the same road, which in practice means back to
        # back: ten minutes apart is a different stretch and a different wind.
        self.pair_max_gap_s = 600.0

        # --- measurements
        # rpm below this is the Controller's own quantisation and the bike
        # crawling; regressing ground speed on it fits the noise.
        self.factor_min_rpm = 200
        # The Odometer scale needs steps to divide by. Under ten counts the
        # quantisation is the measurement.
        self.odo_min_counts = 10


# ------------------------------------------------------------------ the Track

class Point:
    """One GPS fix. `speed_mps` is None unless the Track carried one."""

    __slots__ = ("t", "lat", "lon", "ele", "speed_mps")

    def __init__(self, t, lat, lon, ele=None, speed_mps=None):
        self.t = t
        self.lat = lat
        self.lon = lon
        self.ele = ele
        self.speed_mps = speed_mps


def haversine_m(a_lat, a_lon, b_lat, b_lon):
    p1, p2 = math.radians(a_lat), math.radians(b_lat)
    dp = p2 - p1
    dl = math.radians(b_lon - a_lon)
    h = (math.sin(dp / 2) ** 2 +
         math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2 * EARTH_R_M * math.asin(min(1.0, math.sqrt(h)))


def bearing_deg(a_lat, a_lon, b_lat, b_lon):
    """Initial bearing, degrees clockwise from north."""
    p1, p2 = math.radians(a_lat), math.radians(b_lat)
    dl = math.radians(b_lon - a_lon)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


_ISO_Z = re.compile(r"Z$", re.I)


def parse_time(text):
    """GPX time to a unix float.

    Written out rather than left to datetime.fromisoformat() because that
    only learned to read a trailing Z in 3.11, and the machines this has to
    run on are whatever is installed.
    """
    s = text.strip()
    s = _ISO_Z.sub("+00:00", s)
    # Fractional seconds are optional and may be any width; normalise to six
    # digits, which is what %f and fromisoformat both want.
    m = re.match(r"^(.*?T[\d:]+)(?:\.(\d+))?([+-]\d\d:?\d\d)?$", s)
    if not m:
        raise Refusal("unreadable <time> in the Track: %r" % text)
    base, frac, tz = m.group(1), m.group(2) or "0", m.group(3) or "+00:00"
    tz = tz if ":" in tz else tz[:3] + ":" + tz[3:]
    frac = (frac + "000000")[:6]
    dt = datetime.datetime.strptime(base + "." + frac + tz.replace(":", ""),
                                    "%Y-%m-%dT%H:%M:%S.%f%z")
    return dt.timestamp()


def _tag(elem):
    """The local name, so GPX 1.0, GPX 1.1 and every extension namespace read
    the same. Namespaces here identify the vendor, not the meaning."""
    return elem.tag.rsplit("}", 1)[-1]


def _find_speed(trkpt):
    """Doppler speed, wherever this recorder decided to put it.

    GPX 1.0 has <speed> on the trkpt in m/s. GPX 1.1 dropped it, so 1.1
    writers put it in <extensions>, under a namespace that depends on who
    wrote the file - gpxtpx from Garmin, gpxdata from others, and several apps
    that just write <speed>. All of them mean metres per second; none of them
    agree on where it lives. So: any descendant whose local name is `speed`.
    """
    for elem in trkpt.iter():
        if _tag(elem) == "speed" and elem.text:
            try:
                return float(elem.text)
            except ValueError:
                return None
    return None


class Track:
    """A GPS recording, taken alongside a Capture and never inside one."""

    def __init__(self, points, speed_source, name=""):
        self.points = points
        self.speed_source = speed_source     # "doppler" or "differenced"
        self.name = name
        # Cumulative ground distance, so a distance between two instants is a
        # subtraction. Positions, not the speed channel: "distance against the
        # ground" is what a Track is for and what the Odometer scale means.
        self.cum_m = [0.0]
        for a, b in zip(points, points[1:]):
            self.cum_m.append(self.cum_m[-1] +
                              haversine_m(a.lat, a.lon, b.lat, b.lon))

    @property
    def t0(self):
        return self.points[0].t

    @property
    def t1(self):
        return self.points[-1].t

    @property
    def duration_s(self):
        return self.t1 - self.t0

    @property
    def distance_m(self):
        return self.cum_m[-1]

    @property
    def median_dt_s(self):
        gaps = sorted(b.t - a.t for a, b in zip(self.points, self.points[1:]))
        return gaps[len(gaps) // 2] if gaps else 0.0

    def _bracket(self, t):
        """The index i with points[i].t <= t < points[i+1].t, or None."""
        lo, hi = 0, len(self.points) - 1
        if t < self.points[0].t or t > self.points[-1].t:
            return None
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if self.points[mid].t <= t:
                lo = mid
            else:
                hi = mid
        return lo

    def _interp(self, t, get):
        i = self._bracket(t)
        if i is None:
            return None
        a, b = self.points[i], self.points[i + 1]
        va, vb = get(i), get(i + 1)
        if va is None or vb is None:
            return None
        return va + (vb - va) * (t - a.t) / (b.t - a.t)

    def gap_at(self, t):
        i = self._bracket(t)
        if i is None:
            return None
        return self.points[i + 1].t - self.points[i].t

    def speed_kmh_at(self, t):
        v = self._interp(t, lambda i: self._speed_mps(i))
        return None if v is None else v * 3.6

    def distance_at(self, t):
        return self._interp(t, lambda i: self.cum_m[i])

    def position_at(self, t):
        lat = self._interp(t, lambda i: self.points[i].lat)
        lon = self._interp(t, lambda i: self.points[i].lon)
        ele = self._interp(t, lambda i: self.points[i].ele)
        return lat, lon, ele

    def _speed_mps(self, i):
        p = self.points[i]
        if p.speed_mps is not None:
            return p.speed_mps
        # Differenced positions: centred where there is a point either side,
        # one-sided at the ends. Centred rather than forward because a forward
        # difference reports the speed of the *next* second at this second's
        # timestamp, which is half a sample of lag walked straight into the
        # clock measurement this Track exists to make.
        lo = max(0, i - 1)
        hi = min(len(self.points) - 1, i + 1)
        if hi == lo:
            return 0.0
        dt = self.points[hi].t - self.points[lo].t
        if dt <= 0:
            return 0.0
        return (self.cum_m[hi] - self.cum_m[lo]) / dt


def read_gpx(source, name=""):
    """A Track out of a GPX file, or a Refusal saying what is wrong with it."""
    try:
        tree = ET.parse(source)
    except ET.ParseError as e:
        raise Refusal("not readable as XML: %s" % e)
    root = tree.getroot()
    if _tag(root) != "gpx":
        raise Refusal("root element is <%s>, not <gpx>" % _tag(root))

    points = []
    doppler = 0
    for trkpt in root.iter():
        if _tag(trkpt) != "trkpt":
            continue
        lat, lon = trkpt.get("lat"), trkpt.get("lon")
        if lat is None or lon is None:
            continue
        when = None
        ele = None
        for child in trkpt:
            if _tag(child) == "time" and child.text:
                when = parse_time(child.text)
            elif _tag(child) == "ele" and child.text:
                try:
                    ele = float(child.text)
                except ValueError:
                    ele = None
        if when is None:
            # A trkpt with no time is a shape, not a measurement. Nothing here
            # can use it and interpolating a time for it would invent one.
            continue
        speed = _find_speed(trkpt)
        if speed is not None:
            doppler += 1
        points.append(Point(when, float(lat), float(lon), ele, speed))

    if len(points) < 2:
        raise Refusal("fewer than two timed <trkpt> in the Track")

    points.sort(key=lambda p: p.t)
    # A duplicated timestamp is a recorder writing the same fix twice, and it
    # makes an interpolation divide by zero. Keep the first.
    deduped = [points[0]]
    for p in points[1:]:
        if p.t > deduped[-1].t:
            deduped.append(p)
    points = deduped

    # All or nothing: a Track where some points carry Doppler speed and some
    # do not would silently mix two instruments of different quality into one
    # channel. If it is not on every point, differencing is used throughout
    # and the report says so.
    source_name = "doppler" if doppler == len(points) else "differenced"
    if source_name == "differenced":
        for p in points:
            p.speed_mps = None
    return Track(points, source_name, name)


# ---------------------------------------------------------------- the Capture

class Channels:
    """What a Capture says, as time series in the Monitor's own clock.

    One series per thing rather than one row per frame, because the Controller
    sends the blocks separately and at different rates: rpm arrives eight
    frame types at 5.2 Hz, the power block at another, the Odometer at 0.65 Hz.
    Interleaving them into rows would either invent readings or drop them.
    """

    def __init__(self):
        self.rpm = []            # (t_s, rpm)
        self.throttle = []       # (t_s, raw)
        self.current = []        # (t_s, amps)
        self.pack_v = []         # (t_s, volts)
        self.odo = []            # (t_s, raw count)
        self.soc = []            # (t_s, percent)
        self.remaining_ah = []   # (t_s, Ah)
        self.geometry = None     # (radius, width, ratio, rate_ratio)
        self.header = None
        self.markers = []        # (t_s, text) - honoured if the rider used them
        self.name = ""

    @property
    def unix_start(self):
        return self.header["unix_start"]

    def ctrl_speed_kmh(self, rpm):
        """What the Monitor would have shown for this rpm, or None with no
        0xaf frame to say what wheel the Controller thinks it is turning."""
        if not self.geometry:
            return None
        radius, width, ratio, rate = self.geometry
        if rate == 0:
            return None
        return (rpm * CTRL_SPEED_K * fields.WF_CTRL_GEARING_CORRECTION *
                (radius * 1270.0 + width * ratio) / rate)


def read_capture(path):
    """Every channel this tool needs, in one pass over the archive."""
    chans = Channels()
    chans.name = os.path.basename(path)
    with open(path, "rb") as f:
        chans.header = wfl.read_header(f)
        base = chans.header["unix_start"]
        live = {}
        for rtype, t_ms, payload in wfl.records(f):
            t = base + t_ms / 1000.0
            if rtype == wfl.WFREC_MCU:
                parsed = wfl.decode_ctrl(payload)
                if parsed is None:
                    continue
                ftype, body = parsed
                fields.ctrl_apply(live, ftype, body)
                if ftype in fields.WF_CTRL_TYPE_MOTION:
                    chans.rpm.append((t, live["cur_rpm"]))
                elif ftype in fields.WF_CTRL_TYPE_POWER:
                    chans.current.append((t, live["line_current_a"]))
                    chans.pack_v.append((t, live["pack_v"]))
                elif ftype == fields.WF_CTRL_TYPE_THROTTLE:
                    chans.throttle.append((t, live["throttle_raw"]))
                elif ftype == fields.WF_CTRL_TYPE_ODO:
                    chans.odo.append((t, live["odometer_raw"]))
                elif ftype == fields.WF_CTRL_TYPE_WHEEL:
                    chans.geometry = (live["wheel_radius"], live["wheel_width"],
                                      live["wheel_ratio"], live["rate_ratio"])
            elif rtype == wfl.WFREC_BMS:
                decoded = wfl.decode_daly(payload)
                if decoded is None:
                    continue
                chans.soc.append((t, decoded["soc_pct"]))
                chans.remaining_ah.append((t, decoded["remaining_ah"]))
            elif rtype == wfl.WFREC_EVENT:
                text = payload.split(b"\0")[0].decode(errors="replace")
                if text.startswith("marker"):
                    chans.markers.append((t, text))
    if not chans.rpm:
        raise Refusal("%s carries no motion frames, so it cannot be aligned "
                      "to anything" % chans.name)
    return chans


def bin_seconds(series, how="mean"):
    """A series into whole seconds of its own clock.

    The Controller sends rpm at 5.2 Hz and the Track arrives at 1 Hz, so
    something has to reduce. The mean of a second is the right reduction for a
    correlation - it is a matched filter for the 1 Hz sample it will be
    compared against - and `last` is the right one for a counter like the
    Odometer, where a mean of two counts is a count that never happened.
    """
    buckets = {}
    for t, v in series:
        buckets.setdefault(int(math.floor(t)), []).append(v)
    if how == "last":
        return {k: v[-1] for k, v in buckets.items()}
    return {k: sum(v) / len(v) for k, v in buckets.items()}


# ---------------------------------------------------------------- alignment

def pearson(xs, ys):
    n = len(xs)
    if n < 2:
        return 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    sxy = sxx = syy = 0.0
    for x, y in zip(xs, ys):
        dx, dy = x - mx, y - my
        sxy += dx * dy
        sxx += dx * dx
        syy += dy * dy
    if sxx <= 0.0 or syy <= 0.0:
        return 0.0
    return sxy / math.sqrt(sxx * syy)


class Alignment:
    """The Monitor's clock error, and how much the data believes it.

    `offset_s` is added to the Monitor's wall clock to land on GPS time, so a
    positive offset is a Monitor running slow - cap0002's +12.
    """

    def __init__(self, offset_s, r, before, after, n, refined_s, measured):
        self.offset_s = offset_s
        self.r = r
        self.before = before
        self.after = after
        self.n = n
        self.refined_s = refined_s
        self.measured = measured


def align(track, chans, rules, forced=None):
    """Cross-correlate ground speed against raw rpm over integer-second lags.

    Raw rpm and not decoded speed, deliberately. The rpm -> km/h conversion is
    a scale, a correlation coefficient is scale-invariant, and using the
    decoded speed would put WF_CTRL_GEARING_CORRECTION - a constant this same
    Track is about to re-measure - inside the measurement of the clock. The
    two stay independent.
    """
    rpm_sec = bin_seconds(chans.rpm)
    if not rpm_sec:
        raise Refusal("no rpm to correlate against")

    gps_sec = {}
    t = math.ceil(track.t0)
    while t <= track.t1:
        v = track.speed_kmh_at(t)
        gap = track.gap_at(t)
        if v is not None and gap is not None and gap <= rules.grid_max_gap_s:
            gps_sec[int(t)] = v
        t += 1

    if forced is not None:
        n = len(set(gps_sec) & {k + forced for k in rpm_sec})
        r = _corr_at(gps_sec, rpm_sec, forced)
        return Alignment(forced, r, None, None, n, float(forced), False)

    lo_lag = -rules.align_max_lag_s
    hi_lag = rules.align_max_lag_s
    scored = {}
    for lag in range(lo_lag, hi_lag + 1):
        scored[lag] = _corr_at(gps_sec, rpm_sec, lag)
    best = max(scored, key=lambda k: scored[k])
    r = scored[best]
    before = scored.get(best - 1, 0.0)
    after = scored.get(best + 1, 0.0)
    n = len(set(gps_sec) & {k + best for k in rpm_sec})

    moving = sum(1 for v in gps_sec.values() if v >= rules.align_moving_kmh)
    if n < rules.align_min_overlap_s:
        raise Refusal(
            "the Track and the Capture overlap for %d s, and %d s is the "
            "least this can align on. Check they are the same ride, and that "
            "the Monitor's RTC is not out by more than %d s."
            % (n, rules.align_min_overlap_s, rules.align_max_lag_s))
    if gps_sec and moving / len(gps_sec) < rules.align_min_moving_frac:
        raise Refusal(
            "only %.0f %% of the Track is above %.0f km/h. A ride that barely "
            "moves has no feature to align on."
            % (100.0 * moving / len(gps_sec), rules.align_moving_kmh))
    if r < rules.align_min_r:
        raise Refusal(
            "the best correlation is r = %.4f at %+d s, under the %.2f this "
            "will act on. Either these are two different rides, or the Track "
            "is too coarse to resolve the speed changes in the Capture."
            % (r, best, rules.align_min_r))
    if r - max(before, after) < rules.align_min_prominence:
        raise Refusal(
            "the peak at %+d s (r = %.4f) is no better than its neighbours "
            "(%.4f, %.4f), so it is a plateau and not a peak. A ride needs a "
            "sharp speed change - a full-throttle launch is the best one - "
            "before a clock offset measured this way means anything."
            % (best, r, before, after))

    # Sub-second refinement by fitting a parabola through the peak and its two
    # neighbours. Reported, not used: the grid below is whole seconds, and a
    # fraction of a second is inside what a 1 Hz Track can resolve anyway.
    denom = (before - 2 * r + after)
    refined = float(best)
    if denom != 0.0:
        refined = best + 0.5 * (before - after) / denom
    return Alignment(best, r, before, after, n, refined, True)


def _corr_at(gps_sec, rpm_sec, lag):
    xs, ys = [], []
    for t, v in gps_sec.items():
        rpm = rpm_sec.get(t - lag)
        if rpm is not None:
            xs.append(v)
            ys.append(rpm)
    if len(xs) < 10:
        return 0.0
    return pearson(xs, ys)


# --------------------------------------------------------------- the 1 Hz grid

class Row:
    """One second of the ride, with both instruments on it.

    This is the whole point of aligning: the Track says what the bike did and
    the Capture says what the rider did, and until they share a clock neither
    can label the other.
    """

    __slots__ = ("t", "gps_kmh", "dist_m", "lat", "lon", "ele", "rpm",
                 "ctrl_kmh", "throttle", "current_a", "pack_v", "odo", "soc")

    def __init__(self, t):
        self.t = t
        self.gps_kmh = None
        self.dist_m = None
        self.lat = self.lon = self.ele = None
        self.rpm = None
        self.ctrl_kmh = None
        self.throttle = None
        self.current_a = None
        self.pack_v = None
        self.odo = None
        self.soc = None


def build_grid(track, chans, offset_s, rules):
    """The merged 1 Hz trace, in GPS time, over the overlap only."""
    rpm_sec = bin_seconds(chans.rpm)
    thr_sec = bin_seconds(chans.throttle)
    cur_sec = bin_seconds(chans.current)
    vlt_sec = bin_seconds(chans.pack_v)
    odo_sec = bin_seconds(chans.odo, how="last")
    soc_sec = bin_seconds(chans.soc, how="last")

    rows = []
    t = int(math.ceil(track.t0))
    while t <= track.t1:
        gap = track.gap_at(t)
        if gap is None or gap > rules.grid_max_gap_s:
            t += 1
            continue
        m = t - offset_s          # the same instant on the Monitor's clock
        if m not in rpm_sec:
            t += 1
            continue
        row = Row(t)
        row.gps_kmh = track.speed_kmh_at(t)
        row.dist_m = track.distance_at(t)
        if row.gps_kmh is None or row.dist_m is None:
            # No ground speed for this second. Every rule below compares one,
            # so a row without it is not a second of ride this can say
            # anything about.
            t += 1
            continue
        row.lat, row.lon, row.ele = track.position_at(t)
        row.rpm = rpm_sec[m]
        row.ctrl_kmh = chans.ctrl_speed_kmh(row.rpm)
        row.throttle = thr_sec.get(m)
        row.current_a = cur_sec.get(m)
        row.pack_v = vlt_sec.get(m)
        row.odo = odo_sec.get(m)
        row.soc = soc_sec.get(m)
        rows.append(row)
        t += 1
    if len(rows) < rules.align_min_overlap_s:
        raise Refusal("only %d s of the ride has both instruments on it"
                      % len(rows))
    return rows


def carry_forward(rows, attr):
    """Fill a channel's holes with its last reading, in place.

    The Odometer arrives at 0.65 Hz and the BMS at 0.7 Hz, so most seconds of
    a 1 Hz grid have no reading of either and a hole is not a missing
    measurement - it is the last one still standing. Speed and rpm are never
    filled this way: a hole in those is a link that went down, and inventing a
    speed for it would put distance on a road the bike may not have covered.
    """
    last = None
    for row in rows:
        v = getattr(row, attr)
        if v is None:
            setattr(row, attr, last)
        else:
            last = v


# -------------------------------------------------------------- measurements

class SpeedFactor:
    def __init__(self, kmh_per_rpm, r2, n, ctrl_kmh_per_rpm, correction):
        self.kmh_per_rpm = kmh_per_rpm
        self.r2 = r2
        self.n = n
        self.ctrl_kmh_per_rpm = ctrl_kmh_per_rpm
        self.correction = correction


def measure_speed_factor(rows, chans, rules):
    """Ground speed regressed on raw rpm, through the origin.

    Through the origin because zero rpm is zero road speed and there is no
    physical intercept to fit; an intercept here would be a free parameter
    absorbing whatever lag the alignment left behind, which is exactly the
    error it should be exposing instead.

    The slope is km/h per rpm measured against the ground. Divided by what
    wf_ctrl_speed_kmh() produces for the same rpm it gives the factor the
    Controller's own answer is out by - which over cap0002 was 1.2969, the
    Controller being configured for a 4.000:1 reduction the bike does not
    have.
    """
    sxy = sxx = 0.0
    ys = []
    xs = []
    for row in rows:
        if row.rpm is None or row.gps_kmh is None:
            continue
        if row.rpm < rules.factor_min_rpm:
            continue
        sxy += row.rpm * row.gps_kmh
        sxx += row.rpm * row.rpm
        xs.append(row.rpm)
        ys.append(row.gps_kmh)
    if sxx <= 0.0 or len(xs) < 30:
        return None
    k = sxy / sxx
    # R^2 about the origin-fitted line, against the mean - the ordinary
    # definition, so it is comparable with any other R^2 in this project.
    mean = sum(ys) / len(ys)
    ss_res = sum((y - k * x) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - mean) ** 2 for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

    ctrl_k = None
    correction = None
    if chans.geometry:
        one = chans.ctrl_speed_kmh(1.0)
        if one:
            ctrl_k = one
            correction = fields.WF_CTRL_GEARING_CORRECTION * k / one
    return SpeedFactor(k, r2, len(xs), ctrl_k, correction)


class OdometerScale:
    def __init__(self, metres_per_count, counts, distance_m, median_m, mad_m):
        self.metres_per_count = metres_per_count
        self.counts = counts
        self.distance_m = distance_m
        self.median_m = median_m
        self.mad_m = mad_m


def measure_odometer(rows, rules):
    """Ground distance between the first Odometer step and the last, per count.

    Between steps and not over the whole ride: the count the Odometer holds at
    the start of a Capture is a count it reached some unknown distance before
    the Capture began, so the ride's first and last *readings* bracket an
    unknown amount of road. Its first and last *steps* do not.
    """
    steps = []
    last = None
    for row in rows:
        if row.odo is None or row.dist_m is None:
            continue
        if last is None:
            last = row.odo
            continue
        if row.odo != last:
            steps.append((row.t, row.dist_m, row.odo))
            last = row.odo
    if len(steps) < 2:
        return None
    counts = (steps[-1][2] - steps[0][2]) & 0xFFFF
    if counts < rules.odo_min_counts:
        return None
    distance = steps[-1][1] - steps[0][1]
    per = [(b[1] - a[1]) / (((b[2] - a[2]) & 0xFFFF) or 1)
           for a, b in zip(steps, steps[1:])]
    per.sort()
    median = per[len(per) // 2]
    # Median absolute deviation rather than the range. The Odometer is
    # reported at 0.65 Hz, so a count that stepped is seen up to 1.5 s later -
    # at 80 km/h that is 33 m of slack on a 130 m step, and one long stop
    # between two counts puts an outlier in the range that says nothing about
    # the scale. The endpoints of the whole measurement carry the same delay
    # and so mostly cancel; the per-step figure is what it costs in scatter.
    dev = sorted(abs(x - median) for x in per)
    return OdometerScale(distance / counts, counts, distance, median,
                         dev[len(dev) // 2])


# ---------------------------------------------------------------- manoeuvres

# The identifiers are the shorthand the code has always used; the strings are
# the words CONTEXT.md defines, which are what the report and the CSV print.
REST, HOLD, LAUNCH, COAST, REGEN, RIDING = (
    "rest", "hold", "launch", "rollout", "regen", "riding")


class Manoeuvre:
    """A stretch of ride that means something, found rather than marked."""

    def __init__(self, kind, rows):
        self.kind = kind
        self.rows = rows

    @property
    def t0(self):
        return self.rows[0].t

    @property
    def t1(self):
        return self.rows[-1].t

    @property
    def duration_s(self):
        return self.t1 - self.t0 + 1

    @property
    def v0(self):
        return self.rows[0].gps_kmh

    @property
    def v1(self):
        return self.rows[-1].gps_kmh

    @property
    def mean_kmh(self):
        return sum(r.gps_kmh for r in self.rows) / len(self.rows)

    @property
    def spread_kmh(self):
        vs = [r.gps_kmh for r in self.rows]
        return max(vs) - min(vs)

    @property
    def distance_m(self):
        return self.rows[-1].dist_m - self.rows[0].dist_m

    def mean_of(self, attr):
        vs = [getattr(r, attr) for r in self.rows if getattr(r, attr) is not None]
        return sum(vs) / len(vs) if vs else None

    @property
    def heading_deg(self):
        a, b = self.rows[0], self.rows[-1]
        if None in (a.lat, a.lon, b.lat, b.lon):
            return None
        return bearing_deg(a.lat, a.lon, b.lat, b.lon)


def smoothed_accel(rows, i):
    """km/h per second over +-2 s, one-sided at the ends."""
    lo = max(0, i - 2)
    hi = min(len(rows) - 1, i + 2)
    dt = rows[hi].t - rows[lo].t
    if dt <= 0:
        return 0.0
    return (rows[hi].gps_kmh - rows[lo].gps_kmh) / dt


def find_manoeuvres(rows, rules):
    """Label every second, then keep the runs long enough to mean anything.

    Per-second first and runs second, rather than hunting for shapes: a
    per-second rule is one sentence about one instant and can be read against
    the physics, where a shape matcher is a set of thresholds that only makes
    sense as a whole. Holds are the exception and are found afterwards, over
    what is left, because "steady" is a property of a window and not of a
    second.
    """
    if not rows:
        return []
    throttles = [r.throttle for r in rows if r.throttle is not None]
    throttle_max = max(throttles) if throttles else None
    wide = (throttle_max * rules.launch_throttle_frac
            if throttle_max else None)
    shut = (throttle_max * rules.coast_throttle_frac
            if throttle_max else None)

    labels = []
    for i, row in enumerate(rows):
        a = smoothed_accel(rows, i)
        thr = row.throttle
        cur = row.current_a
        if row.gps_kmh < rules.rest_kmh:
            labels.append(REST)
        elif (wide is not None and thr is not None and thr >= wide
              and a > rules.accel_kmh_per_s):
            labels.append(LAUNCH)
        elif (shut is not None and thr is not None and thr <= shut
              and a < -rules.accel_kmh_per_s):
            if cur is not None and cur <= -rules.regen_min_a:
                labels.append(REGEN)
            elif cur is not None and abs(cur) <= rules.coast_max_abs_a:
                labels.append(COAST)
            else:
                # Slowing with the throttle shut and current that is neither
                # nothing nor regen. Something else is happening - a brake, a
                # hill, the Controller doing something we have not decoded -
                # and calling it either would put it in a measurement.
                labels.append(RIDING)
        else:
            labels.append(RIDING)

    runs = []
    start = 0
    for i in range(1, len(labels) + 1):
        if i == len(labels) or labels[i] != labels[start]:
            runs.append((labels[start], start, i - 1))
            start = i

    minimums = {
        REST: (rules.rest_min_s, 0.0),
        LAUNCH: (rules.launch_min_s, rules.launch_min_delta_kmh),
        COAST: (rules.coast_min_s, rules.coast_min_delta_kmh),
        REGEN: (rules.regen_min_s, rules.regen_min_delta_kmh),
    }
    found = []
    for kind, lo, hi in runs:
        if kind == RIDING:
            continue
        min_s, min_delta = minimums[kind]
        m = Manoeuvre(kind, rows[lo:hi + 1])
        if m.duration_s < min_s:
            labels[lo:hi + 1] = [RIDING] * (hi - lo + 1)
            continue
        if abs(m.v1 - m.v0) < min_delta:
            labels[lo:hi + 1] = [RIDING] * (hi - lo + 1)
            continue
        found.append(m)

    found.extend(_find_holds(rows, labels, rules))
    found.sort(key=lambda m: m.t0)
    return found


def _find_holds(rows, labels, rules):
    """Steady-speed windows over the seconds nothing else claimed.

    Greedy and longest-first: start at each unclaimed second, extend while
    every reading stays inside the tolerance of the window's own mean, and
    keep the window if it lasted. A hold that a launch or a rollout overlaps
    is not a hold - the bike was doing something else and this would be
    measuring the flat top of it.
    """
    holds = []
    i = 0
    while i < len(rows):
        if labels[i] != RIDING or rows[i].gps_kmh < rules.hold_min_kmh:
            i += 1
            continue
        j = i
        vs = []
        while j < len(rows) and labels[j] == RIDING:
            trial = vs + [rows[j].gps_kmh]
            mean = sum(trial) / len(trial)
            tol = max(rules.hold_tol_kmh, rules.hold_tol_frac * mean)
            if max(trial) - mean > tol or mean - min(trial) > tol:
                break
            if rows[j].t - rows[i].t != j - i:
                break        # a hole in the grid; a hold has to be continuous
            vs = trial
            j += 1
        m = Manoeuvre(HOLD, rows[i:j])
        if len(vs) >= rules.hold_min_s and m.mean_kmh >= rules.hold_min_kmh:
            holds.append(m)
            for k in range(i, j):
                labels[k] = HOLD
            i = j
        else:
            i += 1
    return holds


# ----------------------------------------------------------------- coastdowns

class Coast:
    """One rollout, fitted: dv/dt = -(A + C v^2)."""

    def __init__(self, manoeuvre, a, c, r2, n):
        self.m = manoeuvre
        self.a = a               # m/s^2
        self.c = c               # 1/m
        self.r2 = r2
        self.n = n
        self.pair = None

    def forces(self, mass_kg, rho):
        """(rolling force N, Crr, CdA m^2) for a mass and an air density."""
        f0 = mass_kg * self.a
        return f0, self.a / G, 2.0 * mass_kg * self.c / rho

    def wh_per_km(self, mass_kg, rho):
        """The same two numbers in wf_fit.h's units.

        A force in newtons is joules per metre, so a newton is 1000/3600
        watt-hours per kilometre. The drag term converts per (m/s)^2 to per
        (km/h)^2 by dividing by 3.6^2 as well.
        """
        f0, _crr, cda = self.forces(mass_kg, rho)
        a_wh = f0 / 3.6
        c_wh = (0.5 * rho * cda) / 3.6 / (3.6 ** 2)
        return a_wh, c_wh


def fit_coastdown(m):
    """Two coefficients out of one rollout, without differentiating anything.

    Integrating dv/dt = -(A + C v^2) from the start of the run gives

        v0 - v(t) = A * t + C * integral of v^2 dt

    which is linear in A and C, uses every speed reading as it was measured,
    and needs no derivative of a 1 Hz GPS channel. The integral is a
    trapezoid, which is exact for the straight lines the grid interpolates
    between anyway.
    """
    vs = [r.gps_kmh / 3.6 for r in m.rows]
    ts = [r.t - m.rows[0].t for r in m.rows]
    if len(vs) < 5:
        return None
    v0 = vs[0]
    integral = [0.0]
    for k in range(1, len(vs)):
        dt = ts[k] - ts[k - 1]
        integral.append(integral[-1] + 0.5 * (vs[k] ** 2 + vs[k - 1] ** 2) * dt)

    # Normal equations for y = A*x1 + C*x2, no intercept.
    s11 = s12 = s22 = s1y = s2y = 0.0
    ys = []
    for k in range(1, len(vs)):
        x1, x2, y = ts[k], integral[k], v0 - vs[k]
        s11 += x1 * x1
        s12 += x1 * x2
        s22 += x2 * x2
        s1y += x1 * y
        s2y += x2 * y
        ys.append(y)
    det = s11 * s22 - s12 * s12
    if det == 0.0:
        return None
    a = (s1y * s22 - s2y * s12) / det
    c = (s11 * s2y - s12 * s1y) / det
    if a <= 0.0 or c <= 0.0:
        # A rollout that says the bike accelerates on its own, or that drag
        # helps it along. Downhill, or a rider who touched the throttle. It is
        # not a measurement of anything and is reported as a refusal, not
        # averaged into one.
        return Coast(m, a, c, float("nan"), len(ys))
    mean = sum(ys) / len(ys)
    ss_res = 0.0
    for k in range(1, len(vs)):
        pred = a * ts[k] + c * integral[k]
        ss_res += (v0 - vs[k] - pred) ** 2
    ss_tot = sum((y - mean) ** 2 for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return Coast(m, a, c, r2, len(ys))


def pair_coastdowns(coasts, rules):
    """Match each rollout with one going the other way, and average.

    A constant gradient adds g*sin(theta) to A with the sign of travel, so the
    mean of two opposing runs has it gone. Wind does the same to C to first
    order. Nothing here corrects a lone run: an unpaired rollout keeps its own
    coefficients and is reported as unpaired, because a gradient of one part
    in a hundred is 0.098 m/s^2 and a bike's whole rolling term is about
    0.15 - a single run down a slope you cannot feel is wrong by most of the
    answer.
    """
    pairs = []
    used = set()
    for i, one in enumerate(coasts):
        if i in used or one.m.heading_deg is None:
            continue
        best = None
        for j, other in enumerate(coasts):
            if j <= i or j in used or other.m.heading_deg is None:
                continue
            delta = abs((one.m.heading_deg - other.m.heading_deg) % 360.0)
            opposed = abs(delta - 180.0)
            gap = abs(other.m.t0 - one.m.t1)
            if opposed <= rules.pair_heading_tol_deg and gap <= rules.pair_max_gap_s:
                if best is None or gap < best[1]:
                    best = (j, gap)
        if best is not None:
            j = best[0]
            used.add(i)
            used.add(j)
            one.pair = coasts[j]
            coasts[j].pair = one
            pairs.append((one, coasts[j]))
    return pairs


# --------------------------------------------------------------------- report

def _fmt_clock(t):
    return datetime.datetime.fromtimestamp(
        t, datetime.timezone.utc).strftime("%H:%M:%S")


def report(track, chans, alignment, rows, factor, odo, manoeuvres, coasts,
           pairs, rules, args):
    out = []
    w = out.append

    w("TRACK")
    w("  file            %s" % (track.name or "-"))
    w("  points          %d over %.1f min, median sample %.1f s"
      % (len(track.points), track.duration_s / 60.0, track.median_dt_s))
    w("  speed           %s" % (
        "Doppler, off the receiver" if track.speed_source == "doppler"
        else "differenced positions - no <speed> in the file, so this Track "
             "is the weaker instrument"))
    w("  ground distance %.3f km" % (track.distance_m / 1000.0))

    w("")
    w("CAPTURE")
    w("  file            %s" % chans.name)
    w("  seq %d, note %r, %d motion frames"
      % (chans.header["seq"], chans.header["note"], len(chans.rpm)))
    throttles = [v for _t, v in chans.throttle]
    if throttles:
        # The number the rider wants to see first. ride0 measured this bike's
        # throttle shut at 29 counts and hard against the stop at 484, so a
        # ride that never passed 300 is a ride that never asked the Controller
        # for everything it had - which is what a Pack too flat to give it
        # looks like from here. cap0002 peaked at 295.
        w("  throttle        %.0f of ride0's 484 at the stop, %.0f shut"
          % (max(throttles), min(throttles)))
    w("  markers         %d%s" % (
        len(chans.markers),
        "" if chans.markers else "  (none - nothing here needs them)"))

    w("")
    w("ALIGNMENT")
    if alignment.measured:
        w("  offset          %+d s  (add to the Monitor's clock to reach GPS "
          "time; positive is a Monitor running slow)" % alignment.offset_s)
        w("  correlation     r = %.4f, against %.4f and %.4f a second either "
          "side" % (alignment.r, alignment.before, alignment.after))
        w("  peak refines to %+.2f s, which is inside what a %.0f s Track can "
          "resolve" % (alignment.refined_s, track.median_dt_s))
    else:
        w("  offset          %+d s, given on the command line and not measured"
          % alignment.offset_s)
        w("  correlation     r = %.4f at that offset" % alignment.r)
    w("  overlap         %d s with both instruments on it" % len(rows))

    w("")
    w("MEASURED AGAINST THE GROUND")
    if factor is None:
        w("  nothing: the ride never held enough rpm to regress on")
    else:
        w("  speed           %.6f km/h per rpm, R2 = %.5f over %d s"
          % (factor.kmh_per_rpm, factor.r2, factor.n))
        if factor.ctrl_kmh_per_rpm:
            w("  the Controller   %.6f km/h per rpm for this ride's wheel "
              "geometry" % factor.ctrl_kmh_per_rpm)
            w("  implied WF_CTRL_GEARING_CORRECTION = %.4f, against %.4f in "
              "the Field Table" % (factor.correction,
                                   fields.WF_CTRL_GEARING_CORRECTION))
        else:
            w("  no 0xaf frame in the Capture, so there is no wheel geometry "
              "to compare against")
    if odo is None:
        w("  odometer        too few steps to divide by")
    else:
        w("  odometer        %.2f m per count over %d counts and %.0f m"
          % (odo.metres_per_count, odo.counts, odo.distance_m))
        w("                  median step %.2f m +-%.2f (MAD), against %d in "
          "the Field Table"
          % (odo.median_m, odo.mad_m, fields.WF_CTRL_ODO_METRES_PER_COUNT))
    w("  and nothing else. A Track measures ground speed and ground distance;")
    w("  it is not evidence about volts, amps, charge or temperature.")

    w("")
    w("MANOEUVRES")
    if not manoeuvres:
        w("  none found. Either the ride held nothing this looks for, or the")
        w("  thresholds in Rules are wrong for it - both are worth knowing.")
    else:
        w("  %-9s %8s %6s  %7s  %s" % ("kind", "start", "for", "km/h", "what"))
        for m in manoeuvres:
            w("  %-9s %8s %5ds  %7.1f  %s"
              % (m.kind, _fmt_clock(m.t0), m.duration_s, m.mean_kmh,
                 _describe(m)))
        w("  over %d s of ride with both instruments on it" % len(rows))

    w("")
    w("ROLLOUTS")
    if not coasts:
        w("  none. A rollout is the throttle shut, the brake untouched and the")
        w("  bike left to slow down on its own for %ds and %.0f km/h; it is"
          % (rules.coast_min_s, rules.coast_min_delta_kmh))
        w("  the only thing here that measures drag without an ammeter.")
    else:
        for coast in coasts:
            head = coast.m.heading_deg
            w("  %s  %ds  %.1f -> %.1f km/h  heading %s"
              % (_fmt_clock(coast.m.t0), coast.m.duration_s, coast.m.v0,
                 coast.m.v1, "%.0f deg" % head if head is not None else "?"))
            if not math.isfinite(coast.r2):
                w("      refused: the fit says A = %.4f, C = %.6f. A rollout "
                  "cannot have a negative term; this one was downhill, or the "
                  "throttle moved." % (coast.a, coast.c))
                continue
            w("      A = %.4f m/s2   C = %.6f 1/m   R2 = %.4f over %d s"
              % (coast.a, coast.c, coast.r2, coast.n))
            if args.mass_kg:
                f0, crr, cda = coast.forces(args.mass_kg, args.rho)
                a_wh, c_wh = coast.wh_per_km(args.mass_kg, args.rho)
                w("      rolling %.1f N (Crr %.4f), CdA %.3f m2 at %.2f kg/m3"
                  % (f0, crr, cda, args.rho))
                w("      = %.1f + %.5f v^2 Wh/km at the tyre" % (a_wh, c_wh))
        if pairs:
            w("")
            for one, other in pairs:
                a = (one.a + other.a) / 2.0
                c = (one.c + other.c) / 2.0
                w("  paired %s and %s, %.0f and %.0f deg apart: gradient and "
                  "wind cancel"
                  % (_fmt_clock(one.m.t0), _fmt_clock(other.m.t0),
                     one.m.heading_deg, other.m.heading_deg))
                w("      A = %.4f m/s2   C = %.6f 1/m   (the mean of the two)"
                  % (a, c))
                if args.mass_kg:
                    merged = Coast(one.m, a, c, 1.0, one.n + other.n)
                    f0, crr, cda = merged.forces(args.mass_kg, args.rho)
                    a_wh, c_wh = merged.wh_per_km(args.mass_kg, args.rho)
                    w("      rolling %.1f N (Crr %.4f), CdA %.3f m2"
                      % (f0, crr, cda))
                    w("      = %.1f + %.5f v^2 Wh/km at the tyre, against "
                      "main/wfest/wf_fit.h's figure out of the Pack. The gap "
                      "is the driveline, the motor, the Controller and the "
                      "Monitor's own draw." % (a_wh, c_wh))
        unpaired = [c for c in coasts if c.pair is None]
        if unpaired and len(coasts) > 1:
            w("")
            w("  %d rollout(s) unpaired. A lone run carries whatever gradient "
              "it ran down; ride each stretch both ways." % len(unpaired))
        elif not pairs:
            w("")
            w("  Nothing paired. Ride each rollout stretch in both directions, "
              "back to back, or a slope you cannot feel is most of the answer.")

    if not args.mass_kg and coasts:
        w("")
        w("  Pass --mass-kg (bike + rider + luggage) to turn A and C into a "
          "rolling force, a CdA and Wh/km.")

    w("")
    w("AGAINST THE RIDE PROTOCOL")
    counted = {}
    for m in manoeuvres:
        counted[m.kind] = counted.get(m.kind, 0) + 1
    wanted = [
        (REST, 2, "start and end at rest, ignition on, 60 s each (#6.1, #6.7)"),
        (HOLD, 3, "60 s each at three speeds, flat road (#6.3)"),
        (LAUNCH, 3, "standstill to full throttle (#6.4)"),
        (COAST, 4, "throttle shut, no brake, both directions (the drag "
                   "measurement)"),
        (REGEN, 3, "hard engine braking, no friction brake (#6.5)"),
    ]
    missing = []
    for kind, want, why in wanted:
        got = counted.get(kind, 0)
        extra = ""
        if kind == COAST and coasts:
            extra = ", %d paired" % (2 * len(pairs))
        w("  %-10s %2d found%-12s want %d - %s"
          % (kind, got, extra, want, why))
        if got < want:
            missing.append(kind)
    if missing:
        w("  Short of the protocol on: %s. Those manoeuvres are not in this "
          "ride, so nothing here measures them." % ", ".join(missing))
    else:
        w("  Every manoeuvre the protocol asks for is in this ride.")

    # The Pack's own account of the ride. Not a GPS measurement and not
    # offered as one - it is here because #6 is only worth analysing if the
    # ride actually moved 10 Ah, and that is a question the rider needs
    # answered before doing anything else with the file.
    if chans.soc and chans.remaining_ah:
        w("  Pack (the Capture's own words, not the Track's): State of Charge "
          "%.1f -> %.1f %%, %.1f -> %.1f Ah, so %.2f Ah moved"
          % (chans.soc[0][1], chans.soc[-1][1],
             chans.remaining_ah[0][1], chans.remaining_ah[-1][1],
             chans.remaining_ah[0][1] - chans.remaining_ah[-1][1]))
    return "\n".join(out)


def _describe(m):
    if m.kind == HOLD:
        return "%.1f km/h +-%.2f" % (m.mean_kmh, m.spread_kmh / 2.0)
    if m.kind == LAUNCH:
        thr = m.mean_of("throttle")
        return "%.0f -> %.0f km/h, throttle %s" % (
            m.v0, m.v1, "%.0f" % thr if thr is not None else "?")
    if m.kind in (COAST, REGEN):
        cur = m.mean_of("current_a")
        return "%.1f -> %.1f km/h, %s" % (
            m.v0, m.v1, "%+.1f A mean" % cur if cur is not None else "no current")
    if m.kind == REST:
        soc = m.mean_of("soc")
        return "stopped%s" % ("" if soc is None else ", %.1f %% SoC" % soc)
    return ""


# ---------------------------------------------------------------------- output

def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["unix", "clock", "lat", "lon", "ele_m", "gps_kmh",
                      "dist_m", "cur_rpm", "ctrl_kmh", "throttle_raw",
                      "line_current_a", "pack_v", "odometer_raw", "soc_pct"])
        for r in rows:
            out.writerow([
                "%d" % r.t, _fmt_clock(r.t),
                _n(r.lat, 7), _n(r.lon, 7), _n(r.ele, 1), _n(r.gps_kmh, 3),
                _n(r.dist_m, 1), _n(r.rpm, 1), _n(r.ctrl_kmh, 3),
                _n(r.throttle, 0), _n(r.current_a, 2), _n(r.pack_v, 1),
                _n(r.odo, 0), _n(r.soc, 1)])


def write_manoeuvres(path, manoeuvres):
    with open(path, "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["kind", "t0_unix", "t1_unix", "duration_s", "v0_kmh",
                      "v1_kmh", "mean_kmh", "spread_kmh", "distance_m",
                      "heading_deg", "mean_throttle_raw", "mean_current_a",
                      "mean_pack_v"])
        for m in manoeuvres:
            out.writerow([
                m.kind, "%d" % m.t0, "%d" % m.t1, m.duration_s,
                _n(m.v0, 2), _n(m.v1, 2), _n(m.mean_kmh, 2),
                _n(m.spread_kmh, 2), _n(m.distance_m, 1),
                _n(m.heading_deg, 1), _n(m.mean_of("throttle"), 0),
                _n(m.mean_of("current_a"), 2), _n(m.mean_of("pack_v"), 2)])


def _n(v, decimals):
    return "" if v is None else "%.*f" % (decimals, v)


# ------------------------------------------------------------------------ main

def analyse(track, chans, rules, args):
    """Everything between reading the two files and printing. Returns the
    tuple report() wants, so a test can assert on the pieces."""
    alignment = align(track, chans, rules, forced=args.offset)
    rows = build_grid(track, chans, alignment.offset_s, rules)
    for attr in ("throttle", "current_a", "pack_v", "odo", "soc"):
        carry_forward(rows, attr)
    factor = measure_speed_factor(rows, chans, rules)
    odo = measure_odometer(rows, rules)
    manoeuvres = find_manoeuvres(rows, rules)
    coasts = [c for c in (fit_coastdown(m) for m in manoeuvres
                          if m.kind == COAST) if c is not None]
    # Only runs that fitted may pair. A run whose fit came back with a
    # negative coefficient is reported and then left out of everything else -
    # averaging it with a good run would launder it into the answer, which is
    # the opposite of what pairing is for.
    pairs = pair_coastdowns([c for c in coasts if math.isfinite(c.r2)], rules)
    return alignment, rows, factor, odo, manoeuvres, coasts, pairs


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Align a GPS Track to a Capture and find the Manoeuvres.",
        epilog="A Track measures speed and distance against the ground and "
               "nothing else; see the module docstring.")
    ap.add_argument("track", help="the .gpx recorded alongside the ride")
    ap.add_argument("capture", nargs="?", help="the .wfl it belongs to")
    ap.add_argument("--track-only", action="store_true",
                    help="describe the Track and stop")
    ap.add_argument("--offset", type=int, metavar="S",
                    help="use this clock offset instead of measuring one")
    ap.add_argument("--mass-kg", type=float, metavar="KG",
                    help="bike + rider + luggage, which is what turns a "
                         "rollout into a force")
    ap.add_argument("--rho", type=float, default=RHO_DEFAULT, metavar="KG_M3",
                    help="air density (default: %(default)s)")
    ap.add_argument("--csv", metavar="OUT",
                    help="write the merged 1 Hz trace here")
    ap.add_argument("--manoeuvres", metavar="OUT",
                    help="write the manoeuvre table here")
    args = ap.parse_args(argv)

    try:
        with open(args.track, "rb") as f:
            track = read_gpx(f, os.path.basename(args.track))
    except Refusal as e:
        sys.exit("%s: %s" % (args.track, e))

    if args.track_only or not args.capture:
        print("TRACK")
        print("  file            %s" % track.name)
        print("  points          %d over %.1f min, median sample %.1f s"
              % (len(track.points), track.duration_s / 60.0,
                 track.median_dt_s))
        print("  speed           %s" % track.speed_source)
        print("  ground distance %.3f km" % (track.distance_m / 1000.0))
        print("  from            %s to %s UTC"
              % (_fmt_clock(track.t0), _fmt_clock(track.t1)))
        return 0

    rules = Rules()
    try:
        chans = read_capture(args.capture)
        parts = analyse(track, chans, rules, args)
    except Refusal as e:
        sys.exit("cannot analyse this ride: %s" % e)

    alignment, rows, factor, odo, manoeuvres, coasts, pairs = parts
    print(report(track, chans, alignment, rows, factor, odo, manoeuvres,
                 coasts, pairs, rules, args))

    if args.csv:
        write_csv(args.csv, rows)
        print("\nWrote %s, %d rows." % (args.csv, len(rows)))
    if args.manoeuvres:
        write_manoeuvres(args.manoeuvres, manoeuvres)
        print("Wrote %s, %d manoeuvres." % (args.manoeuvres, len(manoeuvres)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
