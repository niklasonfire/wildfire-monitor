#!/usr/bin/env python3
"""Fit Consumption against speed across the Capture archive (issue #19).

WHAT THIS IS, AND WHAT IT IS NOT

It is a curve fitter. It reads spans of road that `tests/host/replay.c
--samples` produced by walking every Capture through the real `wf_est_*` code,
and it solves a two-parameter least-squares problem over them. It holds no
model of the bike, integrates nothing, and cannot tell you what the bike did -
only what shape the numbers the estimator produced take against speed.

It is emphatically not a trained model, and issue #1's Out of Scope says so.
Two coefficients with a physical meaning each, fitted by normal equations, is
the whole of it. There is no capacity to add, because the thing that would
make a prediction better is route knowledge, and no amount of model capacity
substitutes for an input that is missing.

THE FORM, AND WHY IT IS THIS ONE

Energy per unit distance is force. Over a span of road at steady speed the
tractive force balances two things:

  * rolling resistance, tyre and bearing and driveline loss, which is roughly
    constant with speed, and
  * aerodynamic drag, which is 1/2 * rho * Cd * A * v^2.

Integrating force over distance and dividing by that distance gives back the
force, so

  Wh/km(v) = a + c * v^2

with `a` the constant term in watt-hours per kilometre and `c` the drag term
in watt-hours per kilometre per (km/h)^2. That is the whole model.

BE CAREFUL WHICH ONE THIS IS. Fitting POWER against speed is a different
problem with a different exponent: power is force times speed, so it goes as
v^3, and a cubic fitted to watts would be the same physics in different units.
This fits the per-kilometre figure - the one the rider reads, the one
Consumption is defined as in CONTEXT.md, and the one Range divides Remaining
Energy by. Nothing here is a power curve.

A LINEAR TERM is available behind --linear and is off by default. There is a
real physical mechanism for one - the rolling coefficient of a tyre creeps up
with speed, and some driveline losses go with rotational rate - but it is a
small effect next to the two terms above, and a third parameter always fits
the residual better whether or not it means anything. The justification for
turning it on has to come from the data: a linear fit whose middle coefficient
is many times its own uncertainty, over a speed range wide enough to separate
v from v^2. Wanting a better R^2 is not a justification.

WHY THE SAMPLES COME OUT OF C

Walking the archive in Python and integrating energy and distance here would
be a second implementation of exactly what ADR-0004 spent four tickets making
single-source, and the two would drift. So the estimator stays the only
integrator in the project: `replay --samples` differences the estimator's own
totals over each span, and this file does arithmetic no heavier than a 2x2
solve.

USAGE

  scripts/fit_consumption.py                  # fit the archive, write the header
  scripts/fit_consumption.py --dry-run        # fit and report, write nothing
  scripts/fit_consumption.py --samples FILE   # fit a samples file (- is stdin)
  scripts/fit_consumption.py --captures DIR   # a different archive directory

Re-running after a new Capture needs no change to any code: the Captures are
discovered from the directory, the way the replay harness discovers fixtures.
Drop the .wfl in and run it again.

Python standard library only, deliberately - there is no numpy on the machines
this has to run on, and a 2x2 least-squares fit does not need one.
"""
import argparse
import math
import os
import subprocess
import sys
import textwrap

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPLAY = os.path.join(ROOT, "build-host", "replay")
CAPTURE_DIR = os.path.join(ROOT, "tests", "fixtures")
HEADER = os.path.join(ROOT, "main", "wfest", "wf_fit.h")

SAMPLES_VERSION = "# wf-samples 1"


# --------------------------------------------------------------- the rules

class Rules:
    """What makes a span fit to fit on.

    Extraction is a property of the recording and happens in C; acceptance is
    a judgement about the fit and happens here, where the tests for it are.

    MIN_SPEED_KMH is the stationary guard the acceptance criterion names. It
    is mostly redundant - a span that closed on distance already covered 50 m
    inside 30 s, which is 6 km/h - and it is written down anyway, because a
    rule that matters should be visible rather than implied by two constants
    in another file.

    STEADY_FRAC and STEADY_ABS are the acceleration guard, and they are not
    tidiness. Wh/km = a + c*v^2 is a steady-state force balance. A span the
    bike accelerated across put some of its energy into kinetic energy and not
    into drag, so its Wh/km is high for its speed by an amount that depends on
    how hard the rider pulled and not on how fast they were going. Left in,
    those spans bias `a` upward and blur `c`. The threshold is the larger of a
    fraction of the mean and an absolute floor, so that a slow span is not
    rejected for a spread that is the speed signal's own resolution.

    MIN_SPEED_SAMPLES is what makes the spread mean anything: two readings
    cannot say a span was steady.

    MIN_SAMPLES and MIN_SPAN_KMH are the two ways the archive can be too thin
    to fit at all, and they are separate on purpose. A thousand samples all at
    30 km/h cannot separate `a` from `c` however many there are, and twenty
    samples spread from 20 to 90 km/h can - so one counts and the other
    measures.
    """

    def __init__(self):
        self.min_speed_kmh = 5.0
        self.steady_frac = 0.20
        self.steady_abs_kmh = 3.0
        self.min_speed_samples = 4
        self.min_samples = 20
        self.min_span_kmh = 10.0


class Sample:
    """One span of road, as `replay --samples` described it."""

    def __init__(self, fields):
        (self.capture, self.closed, t0, t1, dist, wh, mean, lo, hi,
         nspeed, gap, invalid) = fields
        self.t0_ms = int(t0)
        self.t1_ms = int(t1)
        self.dist_m = float(dist)
        self.energy_wh = float(wh)
        self.mean_kmh = float(mean)
        self.min_kmh = float(lo)
        self.max_kmh = float(hi)
        self.speed_samples = int(nspeed)
        self.gap_ms = int(gap)
        self.invalid = int(invalid)

    @property
    def wh_per_km(self):
        return self.energy_wh / (self.dist_m / 1000.0)

    @property
    def spread_kmh(self):
        return self.max_kmh - self.min_kmh


class Capture:
    """One Capture's totals, printed whether or not it contributed a sample."""

    def __init__(self, fields):
        (self.name, records, duration, dist, wh, vmax, segments) = fields
        self.records = int(records)
        self.duration_ms = int(duration)
        self.distance_m = float(dist)
        self.energy_wh = float(wh)
        self.speed_kmh_max = float(vmax)
        self.segments = int(segments)


def reject_reason(s, rules):
    """Why this span may not be fitted on, or None when it may.

    Ordered from the most structural to the most subtle, so the tally in the
    report reads as a funnel rather than as an arbitrary split of the same
    span between two reasons.
    """
    if not all(math.isfinite(x) for x in (s.dist_m, s.energy_wh, s.mean_kmh,
                                          s.min_kmh, s.max_kmh)):
        # An infinity or a NaN in a span means the estimator produced one, or
        # the samples file was not written by the extractor. Either way it is
        # not a span of road, and one of them left in poisons every
        # coefficient in the solve rather than just its own residual.
        return "not a number: the span carries a value that is not finite"
    if s.closed == "time":
        return "stationary: never covered its span of road"
    if s.closed != "dist":
        return "incomplete: the Capture ended inside it"
    if s.gap_ms > 0:
        return "link down: a gap in the Controller's stream inside it"
    if s.invalid > 0:
        return ("link incomplete: frames arrived before the Controller had "
                "reported both blocks")
    if s.dist_m <= 0.0:
        return "no distance to divide by"
    if s.speed_samples < rules.min_speed_samples:
        return "too few speed readings to call it steady"
    if s.mean_kmh < rules.min_speed_kmh:
        return "stationary: below %.1f km/h" % rules.min_speed_kmh
    limit = max(rules.steady_abs_kmh, rules.steady_frac * s.mean_kmh)
    if s.spread_kmh > limit:
        return "accelerating: %.1f km/h of spread over a %.1f km/h limit" % (
            s.spread_kmh, limit)
    return None


# ------------------------------------------------------------ the samples

def parse_samples(lines):
    """Reads what `replay --samples` printed. Returns (samples, captures)."""
    if isinstance(lines, str):
        lines = lines.splitlines()
    samples, captures = [], []
    seen_version = False
    for lineno, line in enumerate(lines, start=1):
        line = line.strip()
        if line == SAMPLES_VERSION:
            seen_version = True
            continue
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if parts[0] == "sample" and len(parts) == 13:
            samples.append(Sample(parts[1:]))
        elif parts[0] == "capture" and len(parts) == 8:
            captures.append(Capture(parts[1:]))
        else:
            raise ValueError("line %d is not a sample or a capture: %r"
                             % (lineno, line))
    if not seen_version:
        raise ValueError("not a samples file: no %r line" % SAMPLES_VERSION)
    return samples, captures


def extract(replay, capture_dir):
    """Runs the replay harness over the archive and returns its output."""
    if not os.path.exists(replay):
        raise SystemExit("%s is not built - run `make` first" % replay)
    done = subprocess.run([replay, "--samples", capture_dir],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          text=True)
    if done.returncode != 0:
        raise SystemExit("%s --samples %s failed (%d):\n%s"
                         % (replay, capture_dir, done.returncode, done.stderr))
    return done.stdout.splitlines()


# ------------------------------------------------------- the least squares

class Singular(Exception):
    """The samples do not distinguish the terms being fitted."""


# How small a pivot may get, relative to the largest entry of the matrix,
# before the solve is refused. The normal equations square the condition
# number of the design matrix, so double precision's 1e-16 buys about 1e-8 of
# usable conditioning; refusing at 1e-10 stops well short of the numerical
# cliff and lands on data that genuinely cannot separate the terms rather than
# on arithmetic that has run out of bits.
SINGULAR_EPS = 1e-10


def solve(a, b):
    """Solves a x = b by Gaussian elimination with partial pivoting.

    Small and square - two or three unknowns - so this is written out rather
    than reached for. Partial pivoting is what keeps it honest on a matrix
    whose rows are wildly different sizes, and the pivot test is the guard the
    acceptance criterion asks for: a near-zero pivot means the columns are
    linearly dependent to within the precision available, which is what a
    fixed-speed archive looks like to a solver. It raises rather than
    dividing, because a coefficient divided out of a near-zero determinant is
    a confident-looking number with nothing behind it.
    """
    n = len(b)
    biggest = max((abs(x) for row in a for x in row), default=0.0)
    if biggest == 0.0:
        raise Singular("every entry of the normal matrix is zero")
    m = [list(row) + [b[i]] for i, row in enumerate(a)]

    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) <= SINGULAR_EPS * biggest:
            raise Singular(
                "the normal matrix is singular at column %d: the samples do "
                "not separate the terms being fitted" % col)
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(col + 1, n):
            factor = m[r][col] / m[col][col]
            for c in range(col, n + 1):
                m[r][c] -= factor * m[col][c]

    x = [0.0] * n
    for r in reversed(range(n)):
        acc = m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))
        x[r] = acc / m[r][r]
    return x


def least_squares(rows, y):
    """Coefficients minimising the squared residual of `rows * x - y`.

    Normal equations: (A^T A) x = A^T y, then solve().

    The columns are scaled to unit maximum first and the scaling is put back
    afterwards. That is not decoration. At 90 km/h the v^2 column is 8100
    times the constant column, so A^T A has entries spanning 10^7 before a
    single sample is looked at, and the pivot guard would then be measuring
    the units rather than the data. Scaled, a small pivot means what it should
    mean: the samples do not span enough speed.
    """
    k = len(rows[0])
    scale = []
    for c in range(k):
        biggest = max(abs(row[c]) for row in rows)
        scale.append(biggest if biggest > 0.0 else 1.0)
    scaled = [[row[c] / scale[c] for c in range(k)] for row in rows]

    ata = [[sum(r[i] * r[j] for r in scaled) for j in range(k)]
           for i in range(k)]
    aty = [sum(r[i] * yi for r, yi in zip(scaled, y)) for i in range(k)]
    return [x / scale[i] for i, x in enumerate(solve(ata, aty))]


def basis(v, linear):
    """The design row for one speed. [1, v^2], or [1, v, v^2] with --linear."""
    return [1.0, v, v * v] if linear else [1.0, v * v]


# --------------------------------------------------------------- the fit

class Fit:
    """What came out, including the case where nothing did."""

    def __init__(self):
        self.fitted = False
        self.refusal = None
        self.linear = False
        self.coeffs = []          # [a] + ([b] if linear) + [c]
        self.n = 0
        self.r2 = 0.0
        self.rms = 0.0
        self.speed_min_kmh = 0.0
        self.speed_max_kmh = 0.0
        self.captures = []
        self.contributing = []    # capture names that supplied a sample
        self.seen = 0
        self.rejected = {}        # reason -> count
        self.negative = 0         # accepted spans that recovered energy

    @property
    def a(self):
        return self.coeffs[0] if self.fitted else 0.0

    @property
    def b(self):
        return self.coeffs[1] if self.fitted and self.linear else 0.0

    @property
    def c(self):
        return self.coeffs[-1] if self.fitted else 0.0

    def predict(self, v):
        return sum(k * x for k, x in zip(self.coeffs, basis(v, self.linear)))


def fit(samples, captures, rules, linear=False):
    """The whole judgement, in one place: exclude, refuse, or solve."""
    f = Fit()
    f.linear = linear
    f.captures = captures
    f.seen = len(samples)

    accepted = []
    for s in samples:
        why = reject_reason(s, rules)
        if why is None:
            accepted.append(s)
        else:
            f.rejected[why] = f.rejected.get(why, 0) + 1

    f.n = len(accepted)
    f.contributing = sorted({s.capture for s in accepted})
    if accepted:
        f.speed_min_kmh = min(s.mean_kmh for s in accepted)
        f.speed_max_kmh = max(s.mean_kmh for s in accepted)
        f.negative = sum(1 for s in accepted if s.energy_wh < 0.0)

    want = 3 if linear else 2
    if f.n < max(want, rules.min_samples):
        f.refusal = (
            "%d span%s of road survived the exclusions, and %d is the fewest "
            "worth fitting %d coefficients to. The archive does not support a "
            "fit." % (f.n, "" if f.n == 1 else "s", rules.min_samples, want))
        return f

    span = f.speed_max_kmh - f.speed_min_kmh
    if span < rules.min_span_kmh:
        f.refusal = (
            "the accepted spans cover %.1f to %.1f km/h, a range of %.1f km/h. "
            "Below %.1f km/h of range a constant and a quadratic are not "
            "distinguishable: the fit would hand back two coefficients that "
            "trade off against each other freely and mean nothing apart."
            % (f.speed_min_kmh, f.speed_max_kmh, span, rules.min_span_kmh))
        return f

    rows = [basis(s.mean_kmh, linear) for s in accepted]
    y = [s.wh_per_km for s in accepted]
    try:
        f.coeffs = least_squares(rows, y)
    except Singular as exc:
        f.refusal = str(exc)
        return f

    # Belt and braces over the span filter above. A coefficient that is not a
    # finite number is not a coefficient, and it must never reach wf_fit.h:
    # "inf" and "nan" are not C literals, so a header carrying one would fail
    # to compile - and a Monitor that did compile it would be counting a
    # rider's remaining kilometres with it.
    if not all(math.isfinite(k) for k in f.coeffs):
        f.refusal = ("the solve produced a coefficient that is not a finite "
                     "number, so there is nothing to hand the firmware")
        return f

    mean_y = sum(y) / len(y)
    ss_tot = sum((yi - mean_y) ** 2 for yi in y)
    residuals = [yi - sum(k * x for k, x in zip(f.coeffs, row))
                 for row, yi in zip(rows, y)]
    ss_res = sum(r * r for r in residuals)
    f.rms = (ss_res / len(y)) ** 0.5
    if ss_tot == 0.0:
        f.refusal = ("every accepted span has the same Consumption, so there "
                     "is no variation for a fit to explain")
        return f
    f.r2 = 1.0 - ss_res / ss_tot
    f.fitted = True
    return f


# ------------------------------------------------------------- the report

FORM = "Wh/km(v) = a + c*v^2"
FORM_LINEAR = "Wh/km(v) = a + b*v + c*v^2"


def wrap(text, prefix):
    """A paragraph at 72 columns behind `prefix`, so a refusal reads as prose
    in the report and as a comment in the header without being written twice.
    """
    return textwrap.wrap(text, width=72, initial_indent=prefix,
                         subsequent_indent=prefix)


def report(f):
    out = []
    w = out.append
    w("Consumption against speed, over the Capture archive (#19).")
    w("")
    w("THE FORM")
    w("  %s      v in km/h" % (FORM_LINEAR if f.linear else FORM))
    w("")
    w("  Energy per unit distance is force. Rolling resistance and driveline")
    w("  loss are roughly constant with speed; aerodynamic drag is")
    w("  1/2*rho*Cd*A*v^2. So the constant term and the quadratic term are")
    w("  the two the physics predicts, and they are the two fitted.")
    if f.linear:
        w("  The linear term is on. It is off by default: a third parameter")
        w("  fits the residual better whether or not it means anything, and")
        w("  its justification has to come from the data.")
    w("")
    w("  This is Consumption against speed, NOT power against speed. Power is")
    w("  force times speed and goes as v^3; the per-kilometre figure is the")
    w("  force itself and goes as v^2. Fitting the wrong one of those two")
    w("  gives a curve that looks fine and is a different quantity.")
    w("")
    w("THE ARCHIVE")
    if not f.captures:
        w("  No Captures at all.")
    for cap in f.captures:
        w("  %-10s %6d records  %6.1f s  %8.1f m  %8.3f Wh  "
          "peak %5.1f km/h  %d span%s"
          % (cap.name, cap.records, cap.duration_ms / 1000.0, cap.distance_m,
             cap.energy_wh, cap.speed_kmh_max, cap.segments,
             "" if cap.segments == 1 else "s"))
    w("")
    w("THE SPANS")
    w("  %d closed, %d accepted." % (f.seen, f.n))
    for why in sorted(f.rejected):
        w("  %4d excluded - %s" % (f.rejected[why], why))
    if f.negative:
        w("  %d accepted span%s recovered energy rather than spending it."
          % (f.negative, "" if f.negative == 1 else "s"))
    w("")
    if not f.fitted:
        w("NO FIT")
        out.extend(wrap(f.refusal, "  "))
        w("")
        w("  This is the correct outcome for the archive as it stands, not a")
        w("  failure of the tool. The steady-speed holds that this fit wants")
        w("  are in the calibration ride, issue #6, and that ride has not")
        w("  happened. The header is written with the coefficients marked")
        w("  UNFITTED so that nothing downstream can mistake a placeholder")
        w("  for a measurement.")
        return "\n".join(out)

    w("THE FIT")
    w("  a = %.6g Wh/km" % f.a)
    if f.linear:
        w("  b = %.6g Wh/km per km/h" % f.b)
    w("  c = %.6g Wh/km per (km/h)^2" % f.c)
    w("  R^2 = %.4f over %d spans, RMS residual %.3f Wh/km" % (f.r2, f.n, f.rms))
    w("  supported from %.1f to %.1f km/h - the range the data actually covers."
      % (f.speed_min_kmh, f.speed_max_kmh))
    w("  Captures that contributed: %s" % ", ".join(f.contributing))
    w("")
    w("  Outside that range the firmware still computes the polynomial and")
    w("  flags it as extrapolated. Gradient is not in this model at all and")
    w("  is the dominant residual: without route knowledge the largest input")
    w("  is missing, which is why R^2 is a statement about these spans and")
    w("  not a claim about the next ride.")
    w("")
    w("  Predicted Consumption inside the supported range:")
    lo, hi = f.speed_min_kmh, f.speed_max_kmh
    for i in range(5):
        v = lo + (hi - lo) * i / 4.0
        w("    %6.1f km/h  %8.2f Wh/km" % (v, f.predict(v)))
    return "\n".join(out)


# ------------------------------------------------------------- the header

def c_double(x):
    """A double that round-trips through the C compiler exactly as written.

    %.17g is what makes the round trip exact. The trailing ".0" is what keeps
    the literal a double: "%.17g" % 0.0 is "0", which is an int constant, and
    an int constant in an expression the firmware means to be floating point
    is the kind of thing that works everywhere until it does not.

    A non-finite value raises rather than being formatted. "%.17g" turns an
    infinity into "inf" and a NaN into "nan", and neither is a C literal - a
    header carrying one would not compile, and a build that somehow did would
    be counting a rider's kilometres with it. fit() refuses long before this,
    so reaching here at all means a guard upstream has stopped working, and
    that is worth a traceback rather than a plausible-looking file.
    """
    if not math.isfinite(x):
        raise ValueError("%r cannot be written as a C double literal" % x)
    text = "%.17g" % x
    if "." not in text and "e" not in text:
        text += ".0"
    return text


def render_header(f):
    """The coefficients as the firmware compiles them in, with the fit that
    produced them beside them.

    Nothing here is a timestamp or a path, so the file is a pure function of
    the archive: running the tool twice over the same Captures produces the
    same bytes, and tests/test_fit_consumption.py holds the committed copy to
    exactly that.
    """
    lines = []
    w = lines.append
    w("/*")
    w(" * wf_fit.h - Consumption against speed, as constants.")
    w(" *")
    w(" * GENERATED by scripts/fit_consumption.py. Do not edit: re-run the")
    w(" * tool after adding a Capture and commit what it writes. `make test`")
    w(" * fails if this file has drifted from what the archive produces, the")
    w(" * same way docs/field-table.md is held to the Field Table.")
    w(" *")
    w(" * THE FORM. Energy per unit distance is force, so the physically")
    w(" * motivated shape is a constant rolling-and-driveline term plus a")
    w(" * quadratic drag term:")
    w(" *")
    w(" *     %s      v in km/h" % (FORM_LINEAR if f.linear else FORM))
    w(" *")
    w(" * This is Consumption - watt-hours per kilometre - and not power,")
    w(" * which is force times speed and would go as v^3.")
    w(" *")
    w(" * NOTHING FITS ON THE MONITOR. The fit happens offline, over the")
    w(" * archive; the Monitor evaluates a polynomial in two constants. See")
    w(" * wf_est_consumption_at_speed() in wfest.h, which is the only thing")
    w(" * that reads these.")
    w(" *")
    if f.fitted:
        w(" * THE FIT BEHIND THESE NUMBERS.")
        w(" *   R^2 %.4f over %d spans of road, RMS residual %.3f Wh/km."
          % (f.r2, f.n, f.rms))
        w(" *   Supported from %.1f to %.1f km/h. Outside that the evaluator"
          % (f.speed_min_kmh, f.speed_max_kmh))
        w(" *   flags the answer as extrapolated; it is not silently produced.")
        w(" *   Captures that contributed: %s." % ", ".join(f.contributing))
        w(" *")
        w(" *   Gradient is not modelled and is the dominant residual. R^2 is")
        w(" *   a statement about these spans, not a claim about a new ride.")
    else:
        w(" * THERE IS NO FIT. The coefficients below are placeholders and are")
        w(" * marked UNFITTED, in the same spirit as WF_EST_LIMP_POINT_V's")
        w(" * provisional marking and the unsettled WF_CTRL_CURRENT_LSB_PER_A.")
        w(" * WF_FIT_FITTED is 0, wf_est_consumption_at_speed() produces")
        w(" * nothing at any speed, and no figure on the Monitor rests on")
        w(" * them.")
        w(" *")
        lines.extend(wrap("Why: " + f.refusal, " *   "))
        w(" *")
        w(" *   The steady-speed holds this wants are in the calibration ride,")
        w(" *   issue #6, and that ride has not happened.")
    w(" *")
    w(" * THE ARCHIVE THIS WAS RUN OVER.")
    if not f.captures:
        w(" *   No Captures at all.")
    for cap in f.captures:
        w(" *   %s: %d records, %.1f s, %.1f m, %.3f Wh, peak %.1f km/h, "
          "%d span%s" % (cap.name, cap.records, cap.duration_ms / 1000.0,
                         cap.distance_m, cap.energy_wh, cap.speed_kmh_max,
                         cap.segments, "" if cap.segments == 1 else "s"))
    w(" *")
    w(" * %d span%s closed, %d accepted." % (f.seen, "" if f.seen == 1 else "s",
                                             f.n))
    for why in sorted(f.rejected):
        w(" *   %d excluded - %s" % (f.rejected[why], why))
    w(" */")
    w("#ifndef WF_FIT_H")
    w("#define WF_FIT_H")
    w("")
    w("/* 1 when the constants below came out of a fit over real riding, 0")
    w(" * when they are the unfitted placeholder. Everything that reads them")
    w(" * has to check this first. */")
    w("#define WF_FIT_FITTED                %d" % (1 if f.fitted else 0))
    w("")
    w("/* The rolling and driveline term: watt-hours per kilometre at rest,")
    w(" * before any drag. */")
    w("#define WF_FIT_A_WH_PER_KM           %s" % c_double(f.a))
    if f.linear:
        w("")
        w("/* The linear term, on only because the data justified it. */")
        w("#define WF_FIT_LINEAR                1")
        w("#define WF_FIT_B_WH_PER_KM_PER_KMH   %s" % c_double(f.b))
    else:
        w("")
        w("/* No linear term was fitted; see the tool's header comment. */")
        w("#define WF_FIT_LINEAR                0")
        w("#define WF_FIT_B_WH_PER_KM_PER_KMH   0.0")
    w("")
    w("/* The drag term: watt-hours per kilometre per (km/h) squared. */")
    w("#define WF_FIT_C_WH_PER_KM_PER_KMH2  %s" % c_double(f.c))
    w("")
    w("/* The speed range the data actually covers. Asking outside it is")
    w(" * extrapolation and is flagged as such - issue #20's suggested speed")
    w(" * has to be one of these. Both are zero when there is no fit, so no")
    w(" * speed at all is inside the range. */")
    w("#define WF_FIT_SPEED_MIN_KMH         %s" % c_double(f.speed_min_kmh))
    w("#define WF_FIT_SPEED_MAX_KMH         %s" % c_double(f.speed_max_kmh))
    w("")
    w("/* How well it fits, and over how much. Constants so that a Confidence")
    w(" * figure on the screen can be built out of the fit rather than out of")
    w(" * a comment nobody compiles. */")
    w("#define WF_FIT_R2                    %s" % c_double(f.r2))
    w("#define WF_FIT_SAMPLES               %d" % f.n)
    w("#define WF_FIT_RMS_WH_PER_KM         %s" % c_double(f.rms))
    w("")
    w("/* The Captures that went in, so a coefficient can be traced to the")
    w(" * riding it came from. Empty when there is no fit. */")
    w("#define WF_FIT_CAPTURES              \"%s\"" % ", ".join(f.contributing))
    w("")
    w("#endif /* WF_FIT_H */")
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------- main

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Fit Consumption against speed across the Capture archive.")
    ap.add_argument("--captures", default=CAPTURE_DIR,
                    help="directory of .wfl Captures (default: %(default)s)")
    ap.add_argument("--samples",
                    help="read samples from this file instead of replaying the "
                         "archive; - is stdin")
    ap.add_argument("--replay", default=REPLAY,
                    help="the replay harness (default: %(default)s)")
    ap.add_argument("--header", default=HEADER,
                    help="where the constants go (default: %(default)s)")
    ap.add_argument("--dry-run", action="store_true",
                    help="report only, write no header")
    ap.add_argument("--linear", action="store_true",
                    help="fit a linear term as well - only with a reason")
    args = ap.parse_args(argv)

    if args.samples == "-":
        lines = sys.stdin.read().splitlines()
    elif args.samples:
        with open(args.samples, encoding="utf-8") as f:
            lines = f.read().splitlines()
    else:
        lines = extract(args.replay, args.captures)

    rules = Rules()
    samples, captures = parse_samples(lines)
    result = fit(samples, captures, rules, linear=args.linear)

    print(report(result))
    if not args.dry_run:
        with open(args.header, "w", encoding="utf-8") as f:
            f.write(render_header(result))
        print("")
        print("Wrote %s." % os.path.relpath(args.header, ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
