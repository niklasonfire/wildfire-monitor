"""The GPS ingest, proven against rides built to a known answer.

WHY THE RIDES ARE SYNTHETIC

The archive holds one road ride and no Track at all: cap0002's was recorded on
a phone and analysed by hand, and it is not in this repository. So there is
nothing here to check a clock offset, a speed factor or a coastdown against -
which is the same position tests/test_fit_consumption.py is in, and it takes
the same way out. Every ride below is generated from numbers written down
first, so what is being asserted is that the tool recovers what it was not
told, rather than that it agrees with itself.

The exception is EndToEndOnCap0002, which runs the whole pipeline over the
real fixture with a Track synthesised from that Capture's own decoded speed.
That test is CIRCULAR for the two scale measurements by construction - a Track
built out of the Controller can only ever agree with the Controller - and it
is here for everything that is not circular: that 82464 real records parse,
that a real ride's rests and rollouts are found in the real throttle and
current channels, that the real BMS registers still say the ride moved
9.30 Ah, and that none of it takes long enough to be annoying.

The refusals are tested as hard as the measurements. A clock offset asserted
from a correlation that means nothing would shift a whole Capture against the
ground and every number taken off it afterwards; the tool declining to guess
is the feature.
"""
import argparse
import datetime
import io
import math
import os
import random
import re
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import fit_consumption as fc      # noqa: E402
import gps                        # noqa: E402
import wfl                        # noqa: E402
from field_table import load as _load_fields      # noqa: E402

fields = _load_fields()

FIXTURE = os.path.join(ROOT, "tests", "fixtures", "cap0002.wfl")
WFDECODE_C = os.path.join(ROOT, "main", "wfdecode", "wfdecode.c")

T0 = 1788245939        # cap0002's unix_start, so the clocks look like a ride
LAT0, LON0 = 48.0, 11.0
# The same sphere gps.py measures on. A different one here - 111320, say, off
# the WGS84 ellipsoid at the equator - would put a tenth of a percent between
# the speeds these tests write and the distances the tool reads back, which is
# the sort of error a test should not be quietly carrying.
M_PER_DEG_LAT = gps.EARTH_R_M * math.pi / 180.0


# ------------------------------------------------------------------ building

def gpx(points, doppler=True, gpx_version="1.1", extensions=True):
    """A GPX document from (unix, lat, lon, speed_mps) tuples."""
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<gpx version="%s" xmlns="http://www.topografix.com/GPX/1/%s">'
           % (gpx_version, gpx_version[0]),
           "<trk><trkseg>"]
    for t, lat, lon, mps in points:
        iso = datetime.datetime.fromtimestamp(
            t, datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        speed = ""
        if doppler and extensions:
            speed = "<extensions><speed>%.6f</speed></extensions>" % mps
        elif doppler:
            speed = "<speed>%.6f</speed>" % mps
        out.append('<trkpt lat="%.7f" lon="%.7f"><ele>500.0</ele>'
                   "<time>%s</time>%s</trkpt>" % (lat, lon, iso, speed))
    out.append("</trkseg></trk></gpx>")
    return "\n".join(out)


def track_from(speeds, t0=T0, heading=0.0, **kw):
    """A Track that really travels at the speeds given, one per second.

    The positions are integrated from the speeds rather than invented, so the
    Track's ground distance and its speed channel agree - a Track where they
    did not would let a distance test pass on a speed bug.
    """
    lat, lon = LAT0, LON0
    points = []
    for i, kmh in enumerate(speeds):
        points.append((t0 + i, lat, lon, kmh / 3.6))
        step = (kmh / 3.6) / M_PER_DEG_LAT
        lat += step * math.cos(math.radians(heading))
        lon += (step * math.sin(math.radians(heading)) /
                math.cos(math.radians(lat)))
    return gps.read_gpx(io.StringIO(gpx(points, **kw)), "synth.gpx")


# The wheel geometry cap0002 recorded, so the Controller's own speed formula
# has something real to be compared against.
GEOMETRY = (60, 120, 70, 4000)


def channels_from(speeds, kmh_per_rpm=None, throttle=None, current=None,
                  t0=T0, odo_metres_per_count=None, soc=None):
    """A Channels as read_capture() would have built it, from known speeds.

    rpm is derived from the speed by a factor the test knows, which is what
    lets the speed-factor measurement be checked against an answer instead of
    against the Controller.
    """
    chans = gps.Channels()
    chans.name = "synth.wfl"
    chans.header = dict(seq=1, unix_start=t0, note="synth", version=1,
                        hdr_len=0, boot_ms=0, duration_ms=0, mcu="", bms="")
    chans.geometry = GEOMETRY
    if kmh_per_rpm is None:
        # Default to the Controller's own factor for this geometry, so a test
        # that does not care about the scale gets a correction of 1.0.
        kmh_per_rpm = chans.ctrl_speed_kmh(1.0)

    odo_count = 1000
    covered = 0.0
    for i, kmh in enumerate(speeds):
        t = t0 + i
        # The Controller sends the motion block at 5.2 Hz; five samples a
        # second is close enough that bin_seconds() has something to average.
        for k in range(5):
            chans.rpm.append((t + k / 5.0, kmh / kmh_per_rpm))
        chans.throttle.append((t, throttle(i) if callable(throttle)
                               else (0 if throttle is None else throttle)))
        chans.current.append((t, current(i) if callable(current)
                              else (0.0 if current is None else current)))
        chans.pack_v.append((t, 105.0))
        if odo_metres_per_count:
            covered += kmh / 3.6
            while covered >= odo_metres_per_count:
                covered -= odo_metres_per_count
                odo_count += 1
            # 0.65 Hz, like the real thing, so the reporting delay is in the
            # test rather than assumed away.
            if i % 3 == 0:
                chans.odo.append((t, odo_count))
        if soc is not None and i % 3 == 0:
            chans.soc.append((t, soc(i) if callable(soc) else soc))
            chans.remaining_ah.append((t, 30.0))
    return chans


def opts(offset=None, mass_kg=None, rho=gps.RHO_DEFAULT):
    return argparse.Namespace(offset=offset, mass_kg=mass_kg, rho=rho)


def ramp(a, b, n):
    return [a + (b - a) * i / (n - 1) for i in range(n)]


# ----------------------------------------------------------------- the Track

class ReadingGpx(unittest.TestCase):
    def test_gpx_11_extensions_speed_is_doppler(self):
        t = track_from([30.0] * 10)
        self.assertEqual(t.speed_source, "doppler")
        self.assertAlmostEqual(t.speed_kmh_at(T0 + 5), 30.0, delta=0.001)

    def test_gpx_10_child_speed_is_doppler(self):
        t = track_from([30.0] * 10, gpx_version="1.0", extensions=False)
        self.assertEqual(t.speed_source, "doppler")
        self.assertAlmostEqual(t.speed_kmh_at(T0 + 5), 30.0, delta=0.001)

    def test_no_speed_falls_back_to_differenced_positions(self):
        t = track_from([36.0] * 20, doppler=False)
        self.assertEqual(t.speed_source, "differenced")
        # 36 km/h is 10 m/s, and the positions were integrated from it.
        self.assertAlmostEqual(t.speed_kmh_at(T0 + 10), 36.0, places=1)

    def test_partial_doppler_is_treated_as_none(self):
        """A file where only some points carry speed mixes two instruments of
        different quality into one channel, so it uses neither."""
        doc = gpx([(T0 + i, LAT0 + i * 1e-4, LON0, 10.0) for i in range(10)])
        doc = re.sub(r"<extensions>.*?</extensions>", "", doc, count=3)
        t = gps.read_gpx(io.StringIO(doc))
        self.assertEqual(t.speed_source, "differenced")

    def test_times_in_every_spelling_are_the_same_instant(self):
        base = gps.parse_time("2026-09-01T10:18:59Z")
        self.assertEqual(gps.parse_time("2026-09-01T10:18:59.000Z"), base)
        self.assertEqual(gps.parse_time("2026-09-01T10:18:59.123456Z"),
                         base + 0.123456)
        self.assertEqual(gps.parse_time("2026-09-01T12:18:59+02:00"), base)
        self.assertEqual(gps.parse_time("2026-09-01T12:18:59+0200"), base)

    def test_points_are_sorted_and_duplicate_times_dropped(self):
        pts = [(T0 + 2, LAT0, LON0, 1.0), (T0, LAT0, LON0, 1.0),
               (T0 + 2, LAT0, LON0, 1.0), (T0 + 1, LAT0, LON0, 1.0)]
        t = gps.read_gpx(io.StringIO(gpx(pts)))
        self.assertEqual([p.t for p in t.points], [T0, T0 + 1, T0 + 2])

    def test_distance_is_ground_distance(self):
        t = track_from([36.0] * 101)     # 10 m/s for 100 s of travel
        self.assertAlmostEqual(t.distance_m, 1000.0, delta=1.0)

    def test_refuses_what_is_not_a_track(self):
        for bad, why in (("<html></html>", "root element"),
                         ("not xml at all", "readable as XML"),
                         (gpx([(T0, LAT0, LON0, 1.0)]), "fewer than two")):
            with self.assertRaises(gps.Refusal) as caught:
                gps.read_gpx(io.StringIO(bad))
            self.assertIn(why, str(caught.exception))

    def test_untimed_points_are_not_measurements(self):
        doc = gpx([(T0 + i, LAT0 + i * 1e-4, LON0, 10.0) for i in range(5)])
        doc = doc.replace("<trkseg>",
                          '<trkseg><trkpt lat="48.5" lon="11.5"></trkpt>')
        t = gps.read_gpx(io.StringIO(doc))
        self.assertEqual(len(t.points), 5)


# ------------------------------------------------------------------ alignment

def rising_ride(n=400):
    """A ride with real speed changes in it, which is what a correlation
    needs. Three launches and three stops, roughly."""
    out = []
    while len(out) < n:
        out += ramp(0, 70, 25) + [70.0] * 20 + ramp(70, 0, 25) + [0.0] * 15
    return out[:n]


class Alignment(unittest.TestCase):
    def test_recovers_a_known_clock_offset(self):
        speeds = rising_ride()
        for offset in (0, 12, -7, 95):
            chans = channels_from(speeds)
            track = track_from(speeds, t0=T0 + offset)
            found = gps.align(track, chans, gps.Rules())
            self.assertEqual(found.offset_s, offset,
                             "offset %+d came back %+d" % (offset,
                                                           found.offset_s))
            self.assertGreater(found.r, 0.99)

    def test_the_offset_is_added_to_the_monitors_clock(self):
        """The sign convention, asserted rather than left to the docstring: a
        positive offset is a Monitor running slow, which is cap0002's +12."""
        speeds = rising_ride()
        chans = channels_from(speeds)
        track = track_from(speeds, t0=T0 + 12)
        found = gps.align(track, chans, gps.Rules())
        self.assertEqual(found.offset_s, 12)
        rows = gps.build_grid(track, chans, found.offset_s, gps.Rules())
        # The grid is in GPS time, and the speed on each row is the speed the
        # Capture reported 12 s earlier on its own clock.
        row = rows[len(rows) // 2]
        self.assertAlmostEqual(row.gps_kmh, track.speed_kmh_at(row.t),
                               delta=0.001)

    def test_refuses_a_plateau(self):
        """A cruise that only drifts correlates just as well a second either
        side of the peak, so the peak is not evidence of a time."""
        speeds = [45.0 + 5.0 * math.sin(i / 130.0) for i in range(400)]
        with self.assertRaises(gps.Refusal) as caught:
            gps.align(track_from(speeds), channels_from(speeds), gps.Rules())
        self.assertIn("plateau", str(caught.exception))

    def test_refuses_a_speed_that_never_changes(self):
        """And a cruise that does not even drift has no correlation at all."""
        speeds = [45.0] * 400
        with self.assertRaises(gps.Refusal) as caught:
            gps.align(track_from(speeds), channels_from(speeds), gps.Rules())
        self.assertIn("correlation", str(caught.exception))

    def test_refuses_too_little_overlap(self):
        speeds = rising_ride(60)
        with self.assertRaises(gps.Refusal) as caught:
            gps.align(track_from(speeds), channels_from(speeds), gps.Rules())
        self.assertIn("overlap", str(caught.exception))

    def test_refuses_a_ride_that_barely_moves(self):
        speeds = [0.5] * 300 + ramp(0, 20, 20)
        with self.assertRaises(gps.Refusal) as caught:
            gps.align(track_from(speeds), channels_from(speeds), gps.Rules())
        self.assertIn("barely", str(caught.exception).replace(
            "A ride that barely moves", "barely"))

    def test_refuses_two_different_rides(self):
        """rising_ride() is periodic, so its own reverse would still line up
        somewhere; this is a different ride and not a shifted one."""
        walk, v = [], 40.0
        rnd = random.Random(0xb1ce)
        for _ in range(400):
            v = min(80.0, max(0.0, v + rnd.uniform(-6.0, 6.0)))
            walk.append(v)
        with self.assertRaises(gps.Refusal):
            gps.align(track_from(walk), channels_from(rising_ride()),
                      gps.Rules())

    def test_a_forced_offset_skips_the_search(self):
        speeds = [45.0] * 400          # would refuse if it were measured
        found = gps.align(track_from(speeds), channels_from(speeds),
                          gps.Rules(), forced=3)
        self.assertEqual(found.offset_s, 3)
        self.assertFalse(found.measured)


# --------------------------------------------------------------- measurements

class MeasuringAgainstTheGround(unittest.TestCase):
    def analyse(self, speeds, offset=0, **kw):
        chans = channels_from(speeds, **kw)
        track = track_from(speeds, t0=T0 + offset)
        return chans, gps.analyse(track, chans, gps.Rules(), opts())

    def test_recovers_a_known_speed_factor(self):
        speeds = rising_ride(600)
        wanted = 0.031
        _chans, parts = self.analyse(speeds, kmh_per_rpm=wanted)
        factor = parts[2]
        self.assertAlmostEqual(factor.kmh_per_rpm, wanted, places=6)
        self.assertGreater(factor.r2, 0.999)

    def test_a_correct_controller_needs_no_correction(self):
        """A bike whose rpm really does convert the way wf_ctrl_speed_kmh()
        says comes back at the Field Table's own factor, not at 1.0: the
        constant is already in the formula, so measuring no discrepancy means
        measuring the constant back."""
        speeds = rising_ride(600)
        _chans, parts = self.analyse(speeds)
        self.assertAlmostEqual(parts[2].correction,
                               fields.WF_CTRL_GEARING_CORRECTION, places=4)

    def test_finds_a_controller_that_is_out_by_a_known_factor(self):
        speeds = rising_ride(600)
        chans = gps.Channels()
        chans.geometry = GEOMETRY
        true_k = chans.ctrl_speed_kmh(1.0) * 1.30
        _chans, parts = self.analyse(speeds, kmh_per_rpm=true_k)
        self.assertAlmostEqual(
            parts[2].correction,
            fields.WF_CTRL_GEARING_CORRECTION * 1.30, places=3)

    def test_recovers_a_known_odometer_scale(self):
        speeds = [50.0] * 900          # 12.5 km at 13.9 m/s
        for wanted in (100.0, 129.8, 130.0):
            _chans, parts = self.analyse(speeds, odo_metres_per_count=wanted)
            odo = parts[3]
            self.assertIsNotNone(odo, "no scale at %.1f m/count" % wanted)
            # The Odometer is reported at 0.65 Hz, so a step is seen up to
            # 1.5 s after it happened - 21 m at this speed. Over a hundred
            # counts the endpoints' delays largely cancel; 2 % is the room
            # that leaves and it is far tighter than the 30 % error this
            # measurement existed to catch.
            self.assertAlmostEqual(odo.metres_per_count / wanted, 1.0,
                                   delta=0.02)

    def test_refuses_a_scale_from_too_few_counts(self):
        _chans, parts = self.analyse([50.0] * 300, odo_metres_per_count=5000.0)
        self.assertIsNone(parts[3])

    def test_the_speed_constant_matches_the_c(self):
        """gps.py holds a third copy of the rpm -> km/h constant, because the
        formula is hand-written in both languages by the Field Table's own
        admission and C is not importable. This is what stops the third copy
        drifting from the other two."""
        with open(WFDECODE_C, encoding="utf-8") as f:
            source = f.read()
        found = re.search(r"rpm \* ([0-9.]+)f", source)
        self.assertIsNotNone(found, "wf_ctrl_speed_kmh() no longer looks like "
                                    "`rpm * <constant>f`; check gps.py's copy "
                                    "by hand and fix this test")
        self.assertEqual(float(found.group(1)), gps.CTRL_SPEED_K)


# ---------------------------------------------------------------- manoeuvres

THROTTLE_SHUT, THROTTLE_STOP = 29, 484


def protocol_ride():
    """One of everything the ride protocol asks for, in order.

    rest, launch, hold, rollout, regen, rest - built as speed, throttle and
    current together, because that is how the detector sees them: the Track
    says how fast, the Capture says what the rider's hand was doing.
    """
    speeds, throttle, current = [], [], []

    def add(vs, thr, cur):
        speeds.extend(vs)
        throttle.extend([thr] * len(vs))
        current.extend([cur] * len(vs))

    add([0.0] * 60, THROTTLE_SHUT, 0.0)                 # rest
    add(ramp(0, 60, 12), THROTTLE_STOP, 90.0)           # launch
    add([60.0] * 60, 300, 25.0)                         # hold
    add(ramp(60, 20, 20), THROTTLE_SHUT, -0.5)          # rollout
    add([20.0] * 30, 260, 12.0)                         # ordinary riding
    add(ramp(50, 20, 10), THROTTLE_SHUT, -25.0)         # regen
    add([0.0] * 60, THROTTLE_SHUT, 0.0)                 # rest
    return speeds, throttle, current


class FindingManoeuvres(unittest.TestCase):
    def found(self, speeds, throttle, current, rules=None):
        chans = channels_from(speeds, throttle=lambda i: throttle[i],
                              current=lambda i: current[i])
        track = track_from(speeds)
        rules = rules or gps.Rules()
        alignment = gps.align(track, chans, rules, forced=0)
        rows = gps.build_grid(track, chans, alignment.offset_s, rules)
        for attr in ("throttle", "current_a", "pack_v", "odo", "soc"):
            gps.carry_forward(rows, attr)
        return gps.find_manoeuvres(rows, rules)

    def test_finds_one_of_each(self):
        manoeuvres = self.found(*protocol_ride())
        kinds = [m.kind for m in manoeuvres]
        self.assertEqual(kinds, [gps.REST, gps.LAUNCH, gps.HOLD, gps.COAST,
                                 gps.REGEN, gps.REST])

    def test_a_hold_reports_the_speed_it_held(self):
        manoeuvres = self.found(*protocol_ride())
        hold = [m for m in manoeuvres if m.kind == gps.HOLD][0]
        self.assertAlmostEqual(hold.mean_kmh, 60.0, places=3)
        self.assertLess(hold.spread_kmh, 0.001)
        self.assertGreaterEqual(hold.duration_s, gps.Rules().hold_min_s)

    def test_a_rollout_and_a_regen_are_not_the_same_thing(self):
        """Both are the throttle shut and the speed falling. Only the current
        separates them, which is the whole reason the Capture has to be
        aligned before either can be labelled."""
        manoeuvres = self.found(*protocol_ride())
        coast = [m for m in manoeuvres if m.kind == gps.COAST][0]
        regen = [m for m in manoeuvres if m.kind == gps.REGEN][0]
        self.assertGreater(coast.mean_of("current_a"), -1.0)
        self.assertLess(regen.mean_of("current_a"), -20.0)

    def test_slowing_with_current_that_is_neither_is_left_alone(self):
        """Throttle shut, slowing, and drawing 15 A: that is not a rollout and
        not regen, and guessing which would put it in a drag measurement."""
        speeds, throttle, current = protocol_ride()
        start = speeds.index(60.0) + 60
        for i in range(start, start + 20):
            current[i] = -15.0
        manoeuvres = self.found(speeds, throttle, current)
        self.assertNotIn(gps.COAST, [m.kind for m in manoeuvres])

    def test_a_short_manoeuvre_is_not_one(self):
        speeds, throttle, current = protocol_ride()
        rules = gps.Rules()
        rules.coast_min_s = 60
        kinds = [m.kind for m in self.found(speeds, throttle, current, rules)]
        self.assertNotIn(gps.COAST, kinds)

    def test_a_launch_that_never_gets_anywhere_is_not_one(self):
        speeds, throttle, current = protocol_ride()
        rules = gps.Rules()
        rules.launch_min_delta_kmh = 90.0
        kinds = [m.kind for m in self.found(speeds, throttle, current, rules)]
        self.assertNotIn(gps.LAUNCH, kinds)

    def test_ordinary_riding_holds_nothing(self):
        """A speed that wanders is not a hold, however long it goes on."""
        speeds = [40.0 + 6.0 * math.sin(i / 7.0) for i in range(400)]
        manoeuvres = self.found(speeds, [200] * 400, [10.0] * 400)
        self.assertEqual([m for m in manoeuvres if m.kind == gps.HOLD], [])

    def test_the_hold_rule_is_tighter_than_the_fits(self):
        """Deliberately, and this is here so that unifying them fails.

        fit_consumption.py asks whether a span put energy into acceleration;
        this asks whether a rider held a speed on purpose. The second is a
        stricter question and a stretch that fails it is very often still a
        span the fit accepts.
        """
        self.assertLess(gps.Rules().hold_tol_kmh, fc.Rules().steady_abs_kmh)
        self.assertLess(gps.Rules().hold_tol_frac, fc.Rules().steady_frac)


# ----------------------------------------------------------------- coastdowns

def coastdown_speeds(v0_kmh, a, c, seconds, grade=0.0, step=0.001):
    """dv/dt = -(A + C v^2) integrated forward, sampled at 1 Hz.

    Integrated rather than solved in closed form even though a closed form
    exists, because the closed form is only real while A is positive and a
    gradient steep enough to make the bike speed up is one of the cases worth
    generating. A gradient adds g*sin(theta) to A with the sign of travel,
    which is the whole reason rollouts are ridden in pairs.

    The step is a thousandth of the sample interval, so what comes back is the
    continuous curve to far better than the fit under test can resolve.
    """
    a = a + gps.G * grade
    v = v0_kmh / 3.6
    out = []
    for whole in range(seconds):
        out.append(v * 3.6)
        for _ in range(int(1.0 / step)):
            v += -(a + c * v * v) * step
            if v <= 0.0:
                return out
    return out


class FittingRollouts(unittest.TestCase):
    # A light electric motorbike and a rider: nothing measured, just numbers
    # of the right order so a recovered coefficient is checked against
    # something other than itself.
    A = 0.15          # m/s^2, about Crr 0.015
    C = 0.0018        # 1/m, about CdA 0.6 m^2 at 195 kg

    def coast(self, **kw):
        speeds = coastdown_speeds(80.0, self.A, self.C, 40, **kw)
        rows = []
        for i, kmh in enumerate(speeds):
            row = gps.Row(T0 + i)
            row.gps_kmh = kmh
            row.lat, row.lon = LAT0 + i * 1e-4, LON0
            rows.append(row)
        return gps.fit_coastdown(gps.Manoeuvre(gps.COAST, rows))

    def test_recovers_coefficients_it_was_not_told(self):
        """To 0.2 %, which is the trapezoid's own error at 1 Hz and nothing
        to do with the fit. A GPS speed reading is not that good, so this is
        the discretisation being made small enough to disappear behind the
        instrument rather than an accuracy claim."""
        fitted = self.coast()
        self.assertAlmostEqual(fitted.a / self.A, 1.0, delta=0.002)
        self.assertAlmostEqual(fitted.c / self.C, 1.0, delta=0.002)
        self.assertGreater(fitted.r2, 0.9999)

    def test_turns_them_into_a_force_and_an_area(self):
        fitted = self.coast()
        f0, crr, cda = fitted.forces(195.0, 1.20)
        self.assertAlmostEqual(f0, 195.0 * fitted.a, places=9)
        self.assertAlmostEqual(crr, fitted.a / gps.G, places=9)
        self.assertAlmostEqual(cda, 2 * 195.0 * fitted.c / 1.20, places=9)
        self.assertAlmostEqual(f0 / (195.0 * self.A), 1.0, delta=0.002)

    def test_wh_per_km_is_the_force_in_wf_fits_units(self):
        """A newton is a joule per metre, so 1000/3600 watt-hours per
        kilometre. This is the number that gets compared with wf_fit.h - and
        it is the force at the tyre, which is why it must come out lower."""
        fitted = self.coast()
        a_wh, c_wh = fitted.wh_per_km(195.0, 1.20)
        self.assertAlmostEqual(a_wh, 195.0 * fitted.a / 3.6, places=9)
        self.assertAlmostEqual(a_wh / (195.0 * self.A / 3.6), 1.0, delta=0.002)
        at_90 = a_wh + c_wh * 90.0 ** 2
        self.assertGreater(at_90, a_wh)
        # Sanity against the shape of the thing, and the bound is wide: at
        # 90 km/h a 0.6 m^2 bike is pushing about 250 N, which is 69 Wh/km at
        # the tyre before the driveline has taken anything. cap0002's whole
        # ride averaged 42.5 Wh/km out of the Pack at mixed speeds, and the
        # two are consistent - drag goes as the square and most of that ride
        # was not at 90.
        self.assertTrue(5.0 < at_90 < 120.0, at_90)

    def test_a_lone_run_is_wrong_by_the_gradient(self):
        """1 % of slope is 0.098 m/s^2 against a rolling term of 0.15. This
        test exists to make the size of that visible."""
        downhill = self.coast(grade=-0.01)
        self.assertLess(downhill.a, self.A * 0.5)

    def test_a_pair_cancels_the_gradient(self):
        one, other = self.coast(grade=+0.01), self.coast(grade=-0.01)
        self.assertAlmostEqual((one.a + other.a) / (2.0 * self.A), 1.0,
                               delta=0.01)
        self.assertAlmostEqual((one.c + other.c) / (2.0 * self.C), 1.0,
                               delta=0.05)

    def test_refuses_a_rollout_that_speeds_up(self):
        """Steeply downhill, or the rider touched the throttle. A negative
        coefficient is not a drag measurement and is not averaged into one."""
        fitted = self.coast(grade=-0.05)
        self.assertTrue(fitted.a <= 0.0 or math.isnan(fitted.r2))

    def test_pairs_by_opposing_heading(self):
        rules = gps.Rules()
        north = self.coast()
        south = self.coast()
        for i, row in enumerate(south.m.rows):
            row.lat, row.lon = LAT0 - i * 1e-4, LON0
            row.t = north.m.rows[-1].t + 30 + i
        pairs = gps.pair_coastdowns([north, south], rules)
        self.assertEqual(len(pairs), 1)
        self.assertIs(north.pair, south)

    def test_does_not_pair_two_runs_the_same_way(self):
        north, also_north = self.coast(), self.coast()
        for i, row in enumerate(also_north.m.rows):
            row.t = north.m.rows[-1].t + 30 + i
        self.assertEqual(gps.pair_coastdowns([north, also_north], gps.Rules()),
                         [])

    def test_a_refused_run_is_not_paired_into_the_answer(self):
        """analyse() keeps a refused rollout in the report and out of every
        average. Pairing exists to cancel a gradient, and pairing a good run
        with a broken one would launder the broken one into the result."""
        speeds, throttle, current = protocol_ride()
        chans = channels_from(speeds, throttle=lambda i: throttle[i],
                              current=lambda i: current[i])
        track = track_from(speeds)
        parts = gps.analyse(track, chans, gps.Rules(), opts())
        coasts, pairs = parts[5], parts[6]
        for one, other in pairs:
            self.assertTrue(math.isfinite(one.r2) and math.isfinite(other.r2))
        self.assertTrue(all(c.pair is None for c in coasts
                            if not math.isfinite(c.r2)))

    def test_does_not_pair_across_half_an_hour(self):
        rules = gps.Rules()
        north, south = self.coast(), self.coast()
        for i, row in enumerate(south.m.rows):
            row.lat, row.lon = LAT0 - i * 1e-4, LON0
            row.t = north.m.rows[-1].t + rules.pair_max_gap_s + 60 + i
        self.assertEqual(gps.pair_coastdowns([north, south], rules), [])


# --------------------------------------------------------------- end to end

def synthesise_track_from(capture_path, offset_s):
    """A 1 Hz Track built out of a Capture's own decoded speed.

    Circular for anything that measures a scale, and said so at the top of
    this file. What it is not circular for is the shape of a real ride: 38
    minutes of real rpm, real throttle, real current and real BMS responses,
    which is data no synthesiser here produces.
    """
    chans = gps.read_capture(capture_path)
    binned = gps.bin_seconds(chans.rpm)
    lat, points = LAT0, []
    previous = None
    for t in sorted(binned):
        kmh = chans.ctrl_speed_kmh(binned[t])
        if kmh is None:
            continue
        if previous is not None:
            lat += (kmh / 3.6) * (t - previous) / M_PER_DEG_LAT
        previous = t
        points.append((t + offset_s, lat, LON0, kmh / 3.6))
    return chans, gps.read_gpx(io.StringIO(gpx(points)), "synth.gpx")


@unittest.skipUnless(os.path.exists(FIXTURE), "cap0002.wfl is not present")
class EndToEndOnCap0002(unittest.TestCase):
    """The whole pipeline over the archive's only road ride."""

    OFFSET = 12          # the real one, so the test looks like the ride

    @classmethod
    def setUpClass(cls):
        cls.chans, cls.track = synthesise_track_from(FIXTURE, cls.OFFSET)
        cls.parts = gps.analyse(cls.track, cls.chans, gps.Rules(),
                                opts(mass_kg=195.0))

    def test_the_capture_parses(self):
        self.assertEqual(self.chans.header["seq"], 2)
        self.assertEqual(self.chans.header["note"], "wildfire idf v6.1")
        self.assertGreater(len(self.chans.rpm), 10000)
        self.assertEqual(self.chans.geometry[3], 4000)   # rate_ratio

    def test_recovers_the_offset(self):
        alignment = self.parts[0]
        self.assertEqual(alignment.offset_s, self.OFFSET)
        self.assertGreater(alignment.r, 0.99)
        self.assertGreater(alignment.r - max(alignment.before, alignment.after),
                           gps.Rules().align_min_prominence)

    def test_the_odometer_scale_lands_where_the_field_table_says(self):
        """Circular for the scale, but not for the arithmetic: the count of
        steps and the metres between them are the real ride's."""
        odo = self.parts[3]
        self.assertEqual(odo.counts, 163)      # as docs/field-table.md says
        self.assertAlmostEqual(odo.metres_per_count,
                               fields.WF_CTRL_ODO_METRES_PER_COUNT, delta=2.0)

    def test_finds_the_rides_real_stops(self):
        manoeuvres = self.parts[4]
        rests = [m for m in manoeuvres if m.kind == gps.REST]
        self.assertGreaterEqual(len(rests), 2)
        self.assertLess(rests[0].t0, rests[-1].t0)

    def test_finds_rollouts_with_physically_possible_drag_in_them(self):
        """cap0002 rode no protocol, so these are rollouts that happened
        rather than rollouts that were ridden - uncorrected for gradient and
        wind, and unpaired. The assertion is only that a real ride's coasting
        produces a drag area of the order a naked motorbike has, which is the
        difference between this fitting physics and fitting noise."""
        coasts = [c for c in self.parts[5] if math.isfinite(c.r2)]
        self.assertGreaterEqual(len(coasts), 2)
        areas = sorted(c.forces(195.0, 1.20)[2] for c in coasts)
        self.assertTrue(0.15 < areas[len(areas) // 2] < 1.2, areas)

    def test_the_throttle_never_reached_the_stop(self):
        """The reason this ride could not answer #6 step 4: the Pack was at
        65.9 % and would not give the Controller everything it had. ride0
        measured the stop at 484."""
        self.assertLess(max(v for _t, v in self.chans.throttle), 400)

    def test_the_pack_moved_what_the_field_table_says_it_moved(self):
        first, last = self.chans.remaining_ah[0][1], self.chans.remaining_ah[-1][1]
        self.assertAlmostEqual(first - last, 9.3, delta=0.05)
        self.assertAlmostEqual(self.chans.soc[0][1], 65.9, delta=0.1)
        self.assertAlmostEqual(self.chans.soc[-1][1], 47.3, delta=0.1)

    def test_the_report_renders(self):
        text = gps.report(self.track, self.chans, *self.parts,
                          rules=gps.Rules(), args=opts(mass_kg=195.0))
        for expected in ("ALIGNMENT", "MEASURED AGAINST THE GROUND",
                         "MANOEUVRES", "ROLLOUTS",
                         "AGAINST THE RIDE PROTOCOL",
                         "not evidence about volts"):
            self.assertIn(expected, text)


if __name__ == "__main__":
    unittest.main()
