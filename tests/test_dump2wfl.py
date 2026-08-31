"""The dump-to-archive conversion, checked against the committed fixture.

The .wfl fixtures are generated, not hand-made, so the thing worth testing is
that regenerating one produces exactly the bytes that are checked in - which
means the fixture stays reproducible from the dump it came from, and a change
to the converter cannot silently rewrite history.
"""
import io
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import dump2wfl  # noqa: E402
import wfl  # noqa: E402

DUMP = os.path.join(ROOT, "captures", "cap0007_dump.log")
FIXTURE = os.path.join(ROOT, "tests", "fixtures", "cap0007.wfl")


def convert(path):
    with open(path, "r", errors="replace") as f:
        header, records = dump2wfl.parse(f)
    return header, records, dump2wfl.build(header, records)


class Cap0007(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.header, cls.records, cls.blob = convert(DUMP)

    def test_fixture_is_reproducible(self):
        with open(FIXTURE, "rb") as f:
            committed = f.read()
        self.assertEqual(self.blob, committed,
                         "tests/fixtures/cap0007.wfl no longer matches what "
                         "dump2wfl.py produces from captures/cap0007_dump.log; "
                         "run `make fixtures`")

    def test_header_round_trips(self):
        h = wfl.read_header(io.BytesIO(self.blob))
        self.assertEqual(h["version"], 1)
        self.assertEqual(h["seq"], 7)
        self.assertEqual(h["mcu"], "c0:0e:11:de:00:d8")
        self.assertEqual(h["bms"], "40:18:03:01:20:e9")
        self.assertEqual(h["note"], "wildfire idf v6.1")
        # capdump does not print the duration, so a rebuilt Capture carries the
        # archive's own "unknown" rather than a guess.
        self.assertEqual(h["duration_ms"], 0)

    def test_records_round_trip(self):
        f = io.BytesIO(self.blob)
        wfl.read_header(f)
        back = [(t, ms, payload) for t, ms, payload in wfl.records(f)]
        self.assertEqual(len(back), len(self.records))
        self.assertEqual(back, self.records)

    def test_counts_match_the_dump(self):
        counts = {}
        for rtype, _, _ in self.records:
            counts[rtype] = counts.get(rtype, 0) + 1
        self.assertEqual(counts[wfl.WFREC_MCU], 1674)
        self.assertEqual(counts[wfl.WFREC_BMS], 34)
        self.assertEqual(counts[wfl.WFREC_EVENT], 8)
        self.assertEqual(counts[wfl.WFREC_TELEM], 10)
        self.assertEqual(counts[wfl.WFREC_IMU], 942)

    def test_controller_checksums_verify(self):
        bad = [t for rtype, t, p in self.records
               if rtype == wfl.WFREC_MCU and (len(p) != 16 or wfl.crc16(p) != 0)]
        self.assertEqual(bad, [])

    def test_bms_responses_decode(self):
        for rtype, t_ms, payload in self.records:
            if rtype != wfl.WFREC_BMS:
                continue
            d = wfl.decode_daly(payload)
            self.assertIsNotNone(d, f"t={t_ms} did not decode")
            self.assertEqual(d["cell_count"], 28)
            self.assertEqual(d["soc_pct"], 66.7)
            self.assertGreaterEqual(d["pack_v"], 105.1)
            self.assertLessEqual(d["pack_v"], 105.5)


class Grammar(unittest.TestCase):
    """The two spellings of a frame line, and the noise around them."""

    def test_skips_console_noise_before_the_header(self):
        header, records = dump2wfl.parse([
            "wf>  \n",
            "=== $ capdump 7\n",
            "capdump 7\n",
            "CAPHDR seq=7 version=1 unix_start=1 boot_ms=2 mcu=a bms=b note=n\n",
            "WFR t=1 src=mcu type=0x01 len=2 data=aabb\n",
            "CAPDUMP_END records=0\n",
        ])
        self.assertEqual(header["seq"], "7")
        self.assertEqual(records, [(wfl.WFREC_MCU, 1, b"\xaa\xbb")])

    def test_frame_line_without_a_type_field(self):
        """cap0006's capdump does not print type=; the src decides."""
        _, records = dump2wfl.parse([
            "CAPHDR seq=6 version=1 unix_start=0 boot_ms=0 mcu= bms= note=\n",
            "WFR t=5 src=bms len=3 data=d20301\n",
        ])
        self.assertEqual(records, [(wfl.WFREC_BMS, 5, b"\xd2\x03\x01")])

    def test_marker_text_is_carried_through_verbatim(self):
        """A Marker is a WFREC_EVENT whose text starts with "marker"."""
        _, records = dump2wfl.parse([
            "CAPHDR seq=1 version=1 unix_start=0 boot_ms=0 mcu= bms= note=\n",
            "WFE t=9 marker: marker 1\n",
        ])
        self.assertEqual(records, [(wfl.WFREC_EVENT, 9, b"marker: marker 1")])

    def test_a_line_nobody_recognises_is_an_error(self):
        with self.assertRaises(dump2wfl.ParseError):
            dump2wfl.parse([
                "CAPHDR seq=1 version=1 unix_start=0 boot_ms=0 mcu= bms= note=\n",
                "WFR t=1 src=mcu len=2 data=aa\n",   # len disagrees with the hex
            ])


if __name__ == "__main__":
    unittest.main()
