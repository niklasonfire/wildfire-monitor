#!/usr/bin/env python3
"""Issue #19's fit, proven on data the archive cannot supply.

The Capture archive is nowhere near good enough to fit anything: cap0007 is a
47 s parking-lot crawl covering 16.7 m under 6 km/h, and it produces no
Consumption figure at all. So the fitter cannot be proven by running it over
the archive - that would prove only that it refuses, which is one of the two
things worth proving.

The other, and the one that matters, is that it recovers coefficients it was
not told. Every test below that fits anything fits SYNTHESISED spans built
from known `a` and `c`, so the arithmetic is held to an answer known in
advance. When the calibration ride (#6) lands, the fitter will already have
been correct for months; what changes is that it will have something to fit.

The refusals are tested just as hard, because a tool that emits confident
numbers from three points at 4 km/h is worse than one that emits nothing.
"""
import io
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import fit_consumption as fc     # noqa: E402

FIXTURE_DIR = os.path.join(ROOT, "tests", "fixtures")
REPLAY = os.path.join(ROOT, "build-host", "replay")
HEADER = os.path.join(ROOT, "main", "wfest", "wf_fit.h")

# The bike these synthesised spans describe. Nothing measured: two numbers
# picked to be of the right order for a light electric motorbike, so that a
# recovered coefficient is checked against something and not against itself.
TRUE_A = 18.0        # Wh/km of rolling and driveline
TRUE_C = 0.0045      # Wh/km per (km/h)^2 of drag - 36 Wh/km more at 90 km/h


def span(speed_kmh, wh_per_km, closed="dist", gap_ms=0, invalid=0,
         speed_samples=40, spread_kmh=0.5, capture="synth", dist_m=50.0):
    """One synthesised span, in the form `replay --samples` prints.

    Distance and duration are made to agree with the speed, so the sample is
    one the extractor could actually have produced: the fitter reads mean_kmh,
    but a span whose numbers contradicted each other would be a test of
    nothing.
    """
    dt_s = dist_m / (speed_kmh / 3.6) if speed_kmh > 0 else 30.0
    energy_wh = wh_per_km * dist_m / 1000.0
    return ("sample %s %s 0 %d %.6f %.6f %.4f %.4f %.4f %d %d %d"
            % (capture, closed, int(dt_s * 1000), dist_m, energy_wh, speed_kmh,
               speed_kmh - spread_kmh / 2.0, speed_kmh + spread_kmh / 2.0,
               speed_samples, gap_ms, invalid))


def samples_file(lines, captures=("capture synth 1000 60000 5000.0 100.0 "
                                  "90.0 40",)):
    return [fc.SAMPLES_VERSION] + list(lines) + list(captures)


def ramp(lo=15.0, hi=95.0, n=40, a=TRUE_A, c=TRUE_C, **kw):
    """n spans evenly spread over a speed range, each exactly on the curve."""
    out = []
    for i in range(n):
        v = lo + (hi - lo) * i / (n - 1)
        out.append(span(v, a + c * v * v, **kw))
    return out


def fit_lines(lines, **kw):
    samples, captures = fc.parse_samples(samples_file(lines))
    return fc.fit(samples, captures, fc.Rules(), **kw)


class RecoversKnownCoefficients(unittest.TestCase):
    """The test that matters: known a and c back out of generated data."""

    def test_exact_data_recovers_a_and_c(self):
        """Four places on `a`, and that is the samples file's limit, not the
        fitter's. `replay --samples` prints energy to a microwatt-hour, which
        over a 50 m span is 2e-5 Wh/km of quantisation in every y value; a
        tolerance tighter than that would be asserting that text carries more
        digits than it does."""
        f = fit_lines(ramp())
        self.assertTrue(f.fitted, f.refusal)
        self.assertAlmostEqual(f.a, TRUE_A, places=4)
        self.assertAlmostEqual(f.c, TRUE_C, places=8)
        self.assertAlmostEqual(f.r2, 1.0, places=9)
        self.assertLess(f.rms, 1e-3)
        self.assertEqual(f.n, 40)

    def test_a_different_bike_recovers_its_own_coefficients(self):
        """Not a lucky pair: a stiffer, draggier bike comes back too."""
        f = fit_lines(ramp(a=42.0, c=0.011))
        self.assertTrue(f.fitted, f.refusal)
        self.assertAlmostEqual(f.a, 42.0, places=4)
        self.assertAlmostEqual(f.c, 0.011, places=8)

    def test_noise_moves_the_coefficients_but_not_far(self):
        """Deterministic dither, alternating sign, so the test cannot flake.

        +/-2 Wh/km on a curve running from 19 to 59 Wh/km - the sort of
        scatter a hill puts on a span - and the coefficients still land close
        enough to be the same bike.
        """
        lines = []
        for i in range(40):
            v = 15.0 + 80.0 * i / 39.0
            wobble = 2.0 if i % 2 == 0 else -2.0
            lines.append(span(v, TRUE_A + TRUE_C * v * v + wobble))
        f = fit_lines(lines)
        self.assertTrue(f.fitted, f.refusal)
        self.assertAlmostEqual(f.a, TRUE_A, delta=1.5)
        self.assertAlmostEqual(f.c, TRUE_C, delta=0.0005)
        self.assertGreater(f.r2, 0.95)
        self.assertGreater(f.rms, 0.0)

    def test_predictions_sit_on_the_curve(self):
        f = fit_lines(ramp())
        for v in (20.0, 50.0, 90.0):
            self.assertAlmostEqual(f.predict(v), TRUE_A + TRUE_C * v * v,
                                   places=4)

    def test_the_linear_term_is_off_unless_asked_for(self):
        f = fit_lines(ramp())
        self.assertFalse(f.linear)
        self.assertEqual(f.b, 0.0)
        self.assertEqual(len(f.coeffs), 2)

    def test_the_linear_form_recovers_a_linear_bike(self):
        """Three coefficients back out too, when there are three to find."""
        lines = []
        for i in range(40):
            v = 15.0 + 80.0 * i / 39.0
            lines.append(span(v, TRUE_A + 0.08 * v + TRUE_C * v * v))
        f = fit_lines(lines, linear=True)
        self.assertTrue(f.fitted, f.refusal)
        self.assertAlmostEqual(f.a, TRUE_A, places=4)
        self.assertAlmostEqual(f.b, 0.08, places=6)
        self.assertAlmostEqual(f.c, TRUE_C, places=8)


class RefusesWhatItCannotFit(unittest.TestCase):
    """The other half. A refusal has to be a refusal, not a small number."""

    def test_one_speed_is_degenerate_and_is_refused(self):
        """40 spans, all at 30 km/h. No amount of them separates a from c."""
        f = fit_lines([span(30.0, TRUE_A + TRUE_C * 900.0) for _ in range(40)])
        self.assertFalse(f.fitted)
        self.assertIn("km/h", f.refusal)
        self.assertEqual(f.a, 0.0)
        self.assertEqual(f.c, 0.0)

    def test_a_narrow_speed_range_is_refused_before_it_is_solved(self):
        f = fit_lines(ramp(lo=30.0, hi=34.0))
        self.assertFalse(f.fitted)
        self.assertIn("not distinguishable", f.refusal)

    def test_too_few_spans_is_refused(self):
        f = fit_lines(ramp(n=8))
        self.assertFalse(f.fitted)
        self.assertIn("survived the exclusions", f.refusal)

    def test_no_samples_at_all_is_refused(self):
        f = fit_lines([])
        self.assertFalse(f.fitted)
        self.assertEqual(f.n, 0)

    def test_the_solver_itself_raises_on_a_singular_matrix(self):
        """The guard, reached directly: two identical columns.

        Straight at solve(), because the checks above stop the fit before it
        ever gets here and a guard nothing reaches is a guard nobody knows
        works.
        """
        with self.assertRaises(fc.Singular):
            fc.solve([[1.0, 2.0], [2.0, 4.0]], [3.0, 6.0])

    def test_the_solver_raises_on_an_all_zero_matrix(self):
        with self.assertRaises(fc.Singular):
            fc.solve([[0.0, 0.0], [0.0, 0.0]], [0.0, 0.0])

    def test_the_solver_solves_a_system_it_should(self):
        """The guard is not simply refusing everything."""
        x = fc.solve([[2.0, 1.0], [1.0, 3.0]], [5.0, 10.0])
        self.assertAlmostEqual(x[0], 1.0, places=12)
        self.assertAlmostEqual(x[1], 3.0, places=12)

    def test_wildly_scaled_columns_still_solve(self):
        """Column scaling earning its place: v^2 at 90 km/h is 8100x the
        constant column, and a fit over it must not trip the guard."""
        f = fit_lines(ramp(lo=1.0, hi=200.0, n=60))
        self.assertTrue(f.fitted, f.refusal)
        self.assertAlmostEqual(f.a, TRUE_A, places=4)
        self.assertAlmostEqual(f.c, TRUE_C, places=8)


class ExcludesWhatWouldDragTheFit(unittest.TestCase):
    """Stationary spans, spans over a downed link, and spans the bike
    accelerated across are dropped and counted - never averaged in."""

    def bad(self, extra):
        """A good ramp with one bad span in it, fitted."""
        return fit_lines(ramp() + [extra])

    def poisoned(self, field, text):
        """An otherwise-good span with one numeric field replaced.

        By index rather than by search-and-replace: a replacement that
        silently matched nothing would leave a perfectly good span in the
        ramp, and the test would pass while asserting nothing at all."""
        parts = span(50.0, 500.0).split()
        self.assertNotEqual(parts[field], text, "field %d already %r" %
                            (field, text))
        parts[field] = text
        return " ".join(parts)

    def test_a_stationary_span_is_excluded(self):
        f = self.bad(span(0.5, 900.0, closed="time"))
        self.assertEqual(f.n, 40)
        self.assertTrue(any("stationary" in why for why in f.rejected))
        self.assertAlmostEqual(f.a, TRUE_A, places=6)

    def test_a_crawling_span_is_excluded_even_if_it_closed_on_distance(self):
        f = self.bad(span(2.0, 400.0))
        self.assertEqual(f.n, 40)
        self.assertTrue(any("below 5.0 km/h" in why for why in f.rejected))

    def test_a_span_with_a_link_gap_is_excluded(self):
        f = self.bad(span(50.0, 500.0, gap_ms=4000))
        self.assertEqual(f.n, 40)
        self.assertTrue(any("link down" in why for why in f.rejected))
        self.assertAlmostEqual(f.c, TRUE_C, places=9)

    def test_a_span_before_the_controller_had_spoken_is_excluded(self):
        """The blocks latch once seen, so this is the start of a ride: frames
        arriving before the Controller had reported both of them. Its energy
        or its speed is missing, which is one half of Wh/km."""
        f = self.bad(span(50.0, 500.0, invalid=7))
        self.assertEqual(f.n, 40)
        self.assertTrue(any("link incomplete" in why for why in f.rejected))

    def test_a_span_the_bike_accelerated_across_is_excluded(self):
        """40 km/h of spread on a 50 km/h span: a launch, not a hold. Its
        energy went into kinetic energy, which Wh/km = a + c*v^2 does not
        model, so it must not be averaged in."""
        f = self.bad(span(50.0, 500.0, spread_kmh=40.0))
        self.assertEqual(f.n, 40)
        self.assertTrue(any("accelerating" in why for why in f.rejected))

    def test_a_span_carrying_an_infinity_is_excluded(self):
        """One of these left in poisons every coefficient in the solve, not
        just its own residual - the normal equations sum over all of them."""
        f = self.bad(self.poisoned(6, "inf"))       # energy_wh
        self.assertEqual(f.n, 40)
        self.assertTrue(any("not a number" in why for why in f.rejected))
        self.assertAlmostEqual(f.a, TRUE_A, places=4)

    def test_a_span_carrying_a_nan_is_excluded(self):
        f = self.bad(self.poisoned(7, "nan"))       # mean_kmh
        self.assertEqual(f.n, 40)
        self.assertTrue(any("not a number" in why for why in f.rejected))

    def test_a_span_the_capture_ended_inside_is_excluded(self):
        f = self.bad(span(50.0, 500.0, closed="eof"))
        self.assertEqual(f.n, 40)
        self.assertTrue(any("incomplete" in why for why in f.rejected))

    def test_a_span_with_almost_no_speed_readings_is_excluded(self):
        f = self.bad(span(50.0, 500.0, speed_samples=2))
        self.assertEqual(f.n, 40)
        self.assertTrue(any("too few speed readings" in why
                            for why in f.rejected))

    def test_every_exclusion_is_counted(self):
        """The tally accounts for every span, so nothing is dropped quietly."""
        f = fit_lines(ramp() + [span(0.5, 900.0, closed="time"),
                                span(50.0, 500.0, gap_ms=4000),
                                span(50.0, 500.0, spread_kmh=40.0)])
        self.assertEqual(f.seen, 43)
        self.assertEqual(f.n + sum(f.rejected.values()), f.seen)

    def test_a_steady_span_at_speed_is_kept(self):
        """The rules reject the right things and not everything."""
        f = fit_lines(ramp(n=39) + [span(60.0, TRUE_A + TRUE_C * 3600.0)])
        self.assertEqual(f.n, 40)
        self.assertEqual(f.rejected, {})


class ReportsWhatTheDataSupports(unittest.TestCase):
    """Extrapolation is flagged by the fit reporting its own range, and #20
    consumes that range to pick a speed the fit actually covers."""

    def test_the_speed_range_is_the_accepted_spans_range(self):
        f = fit_lines(ramp(lo=22.0, hi=88.0))
        self.assertTrue(f.fitted, f.refusal)
        self.assertAlmostEqual(f.speed_min_kmh, 22.0, places=4)
        self.assertAlmostEqual(f.speed_max_kmh, 88.0, places=4)

    def test_an_excluded_fast_span_does_not_widen_the_range(self):
        """A range that counted excluded spans would licence extrapolation
        into speeds nothing was fitted over."""
        f = fit_lines(ramp(lo=22.0, hi=60.0) + [span(140.0, 200.0, gap_ms=9000)])
        self.assertTrue(f.fitted, f.refusal)
        self.assertAlmostEqual(f.speed_max_kmh, 60.0, places=4)

    def test_the_contributing_captures_are_named(self):
        f = fit_lines(ramp(n=20, capture="capA")
                      + ramp(n=20, capture="capB"))
        self.assertEqual(f.contributing, ["capA", "capB"])

    def test_a_capture_that_contributed_nothing_is_not_named(self):
        f = fit_lines(ramp(n=40, capture="capA")
                      + [span(0.5, 900.0, closed="time", capture="capB")])
        self.assertEqual(f.contributing, ["capA"])


class TheHeaderCarriesItsProvenance(unittest.TestCase):
    """A coefficient with no provenance is what the Confidence discipline
    exists to prevent, so the header is checked for it."""

    def test_a_fitted_header_carries_the_fit(self):
        f = fit_lines(ramp())
        text = fc.render_header(f)
        self.assertIn("#define WF_FIT_FITTED                1", text)
        self.assertIn("WF_FIT_SAMPLES               40", text)
        self.assertIn("R^2 1.0000", text)
        self.assertIn("15.0 to 95.0 km/h", text)
        self.assertIn("Captures that contributed: synth", text)
        self.assertIn("Wh/km(v) = a + c*v^2", text)

    def test_an_unfitted_header_says_so_loudly(self):
        f = fit_lines([span(2.0, 400.0, closed="time")])
        text = fc.render_header(f)
        self.assertIn("#define WF_FIT_FITTED                0", text)
        self.assertIn("UNFITTED", text)
        self.assertIn("WF_FIT_SPEED_MIN_KMH         0", text)
        self.assertIn("WF_FIT_SPEED_MAX_KMH         0", text)
        self.assertIn("issue #6", text)

    def test_the_coefficients_round_trip_through_the_text(self):
        """%.17g, so what the compiler reads is the double that was fitted."""
        f = fit_lines(ramp())
        text = fc.render_header(f)
        for line in text.splitlines():
            if line.startswith("#define WF_FIT_C_WH_PER_KM_PER_KMH2"):
                self.assertEqual(float(line.split()[-1]), f.c)
                return
        self.fail("no drag coefficient in the header")

    def test_a_non_finite_coefficient_never_reaches_the_header(self):
        """"inf" and "nan" are not C literals. fit() refuses before this can
        happen, so c_double() raising is the guard behind the guard: if one
        ever does get through, the tool stops rather than writing a header
        that will not compile - or, worse, one that quietly does."""
        for bad in (float("inf"), float("-inf"), float("nan")):
            with self.assertRaises(ValueError):
                fc.c_double(bad)

    def test_finite_coefficients_stay_double_literals(self):
        """The other half: an ordinary value still comes out as a double and
        not as an int constant."""
        self.assertEqual(fc.c_double(0.0), "0.0")
        self.assertEqual(fc.c_double(-7.0), "-7.0")
        self.assertIn(".", fc.c_double(18.0))
        self.assertEqual(float(fc.c_double(0.0045)), 0.0045)

    def test_rendering_is_deterministic(self):
        """No clock and no path in it, so the committed copy can be compared
        against a fresh run byte for byte."""
        f = fit_lines(ramp())
        self.assertEqual(fc.render_header(f),
                         fc.render_header(f))


class OverTheArchiveAsItStands(unittest.TestCase):
    """What the tool says about today's archive, asserted rather than hoped.

    These are the tests that will change when the calibration ride lands, and
    changing them is the point: right now the honest answer is that there is
    nothing to fit, and the suite says so out loud.
    """

    def samples(self):
        self.assertTrue(os.path.exists(REPLAY),
                        "%s is not built - run make" % REPLAY)
        done = subprocess.run([REPLAY, "--samples", FIXTURE_DIR],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True)
        self.assertEqual(done.returncode, 0, done.stderr)
        return done.stdout.splitlines()

    def test_the_extractor_produces_a_readable_samples_file(self):
        samples, captures = fc.parse_samples(self.samples())
        self.assertTrue(captures, "no Capture summaries at all")
        for cap in captures:
            self.assertGreater(cap.records, 0)
        for s in samples:
            self.assertIn(s.closed, ("dist", "time", "eof"))

    def test_the_archive_supports_no_fit_and_the_tool_says_so(self):
        samples, captures = fc.parse_samples(self.samples())
        f = fc.fit(samples, captures, fc.Rules())
        self.assertFalse(
            f.fitted,
            "the archive now supports a fit - the calibration ride has "
            "landed, so update this test and commit the fitted wf_fit.h")
        self.assertIsNotNone(f.refusal)
        text = fc.report(f)
        self.assertIn("NO FIT", text)
        self.assertIn("issue #6", text)

    def test_the_committed_header_is_what_the_archive_produces(self):
        """The other half of the provenance promise: the constants the
        firmware compiles in are the ones today's archive gives, not ones
        that were true once."""
        samples, captures = fc.parse_samples(self.samples())
        fresh = fc.render_header(fc.fit(samples, captures, fc.Rules()))
        with open(HEADER, encoding="utf-8") as f:
            committed = f.read()
        self.assertEqual(committed, fresh,
                         "main/wfest/wf_fit.h has drifted from the archive - "
                         "run scripts/fit_consumption.py and commit it")

    def test_the_tool_runs_end_to_end(self):
        """argument parsing, the subprocess, the report and the write, once.

        Into a temporary file, so a passing test cannot be the thing that
        rewrote the committed header the test above compares against."""
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "wf_fit.h")
            quiet = io.StringIO()
            stdout, sys.stdout = sys.stdout, quiet
            try:
                rc = fc.main(["--captures", FIXTURE_DIR, "--header", out])
            finally:
                sys.stdout = stdout
            self.assertEqual(rc, 0)
            self.assertIn("NO FIT", quiet.getvalue())
            with open(out, encoding="utf-8") as f:
                self.assertIn("WF_FIT_FITTED", f.read())


if __name__ == "__main__":
    unittest.main()
