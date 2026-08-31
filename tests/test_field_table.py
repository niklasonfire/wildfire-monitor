#!/usr/bin/env python3
"""ADR-0002, asserted rather than asserted-to.

The Field Table's whole claim is that one description of the bytes produces
three artefacts that cannot disagree. Three tests hold it to that:

* the generated C decoder and the generated Python decoder are handed the same
  recorded Capture and have to produce identical numbers for every field of
  every record,
* both are handed values no Capture we hold contains, because the comparison
  above can only compare what the archive happens to carry, and
* the committed field documentation has to be what the generator produces from
  the table as it stands today.

All fail loudly. The first names the record and the field that diverged; the
last says which command puts it right.
"""
import glob
import json
import os
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "scripts"))
from field_table import load as load_fields          # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURE_DIR = os.path.join(ROOT, "tests", "fixtures")
REPLAY = os.path.join(ROOT, "build-host", "replay")
WFL = os.path.join(ROOT, "scripts", "wfl.py")
GENERATOR = os.path.join(ROOT, "scripts", "gen_fields.py")
TABLE = os.path.join(ROOT, "field-table.json")

# Read out of the table rather than repeated here. Repeating it would mean a
# renamed document could leave this test comparing a file nobody publishes
# against a file nobody reads, and passing - a check that has quietly stopped
# checking is worse than no check.
with open(TABLE, encoding="utf-8") as _f:
    DOC_REL = json.load(_f)["doc_file"]
DOC = os.path.join(ROOT, *DOC_REL.split("/"))


def captures():
    return sorted(glob.glob(os.path.join(FIXTURE_DIR, "*.wfl")))


def run(argv):
    done = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          text=True)
    if done.returncode != 0:
        raise AssertionError("%s failed (%d):\n%s"
                             % (" ".join(argv), done.returncode, done.stderr))
    return done.stdout.splitlines()


def divergence(c_lines, py_lines):
    """The first place the two decoders disagree, named down to the field, or
    None when they do not."""
    for lineno, (a, b) in enumerate(zip(c_lines, py_lines), start=1):
        if a == b:
            continue
        ta, tb = a.split(), b.split()
        for x, y in zip(ta, tb):
            if x != y:
                return ("line %d, record %s: C decodes `%s`, Python decodes "
                        "`%s`\n  C:      %s\n  Python: %s"
                        % (lineno, ta[1] if len(ta) > 1 else "?", x, y, a, b))
        return ("line %d: the two decoders assign a different number of "
                "fields\n  C:      %s\n  Python: %s" % (lineno, a, b))
    if len(c_lines) != len(py_lines):
        return ("the two decoders decoded a different number of records: "
                "C %d, Python %d" % (len(c_lines), len(py_lines)))
    return None


class CrossLanguageAgreement(unittest.TestCase):
    """ADR-0001 forces the decoding to exist twice. This is what stops the two
    copies drifting."""

    def test_the_replay_harness_is_built(self):
        self.assertTrue(os.path.exists(REPLAY),
                        "%s is missing - this test compares the generated C "
                        "decoder against the generated Python one, so run "
                        "`make test` rather than unittest on its own" % REPLAY)

    def test_both_decoders_agree_on_every_capture(self):
        found = captures()
        self.assertTrue(found, "no .wfl fixtures in %s" % FIXTURE_DIR)
        for path in found:
            with self.subTest(capture=os.path.basename(path)):
                c_lines = run([REPLAY, "--fields", path])
                py_lines = run([sys.executable, WFL, path, "--fields"])
                # A Capture that decodes into nothing would let both sides
                # agree on silence, which asserts nothing at all.
                self.assertTrue(any("=" in line for line in c_lines),
                                "%s decoded no fields at all" % path)
                problem = divergence(c_lines, py_lines)
                if problem is not None:
                    self.fail("%s: the generated decoders disagree.\n%s"
                              % (os.path.basename(path), problem))


class BothDecodersNarrowTheSameWay(unittest.TestCase):
    """The cross-language test above can only compare the values the archive
    happens to contain, and the archive is one 47-second parking-lot ride. It
    would agree just as loudly on two decoders that diverge on every value that
    ride did not produce.

    This drives the values instead of waiting for them. The C decoder assigns
    into a struct member of a declared width, so every integer it produces is
    inside that member's range by construction; Python has no such member and
    has to be made to do the same thing deliberately. The obvious case is a
    sensor that is absent: a BMS answering 0xffff for a temperature it does not
    have reads -41 C in C and 65495 in Python if nobody narrows.
    """

    @classmethod
    def setUpClass(cls):
        cls.fields = load_fields()
        with open(TABLE, encoding="utf-8") as f:
            cls.table = json.load(f)

    # What a C assignment to each member can hold. A ctype that is not in here
    # is one nobody has thought about, which is a failure and not a skip.
    RANGE = {
        "uint8_t":  (0, 0xff),
        "int8_t":   (-0x80, 0x7f),
        "uint16_t": (0, 0xffff),
        "int16_t":  (-0x8000, 0x7fff),
        "uint32_t": (0, 0xffffffff),
        "int32_t":  (-0x80000000, 0x7fffffff),
    }

    def device(self, dev_id):
        for d in self.table["devices"]:
            if d["id"] == dev_id:
                return d
        self.fail("no device %s in the table" % dev_id)

    def check(self, dev_id, decoded, what):
        """Every integer field of one device, against the range the member it
        is assigned to in C can actually hold."""
        checked = 0
        for f in self.device(dev_id)["fields"]:
            ctype = f["ctype"]
            if ctype in ("bool", "float"):
                continue
            self.assertIn(ctype, self.RANGE,
                          "%s is declared %s, which this test has no C range "
                          "for - add one rather than letting the field go "
                          "unchecked" % (f["name"], ctype))
            lo, hi = self.RANGE[ctype]
            if f["name"] not in decoded:
                continue
            got = decoded[f["name"]]
            values = got if isinstance(got, list) else [got]
            for i, v in enumerate(values):
                where = ("%s[%d]" % (f["name"], i)) if isinstance(got, list) \
                    else f["name"]
                self.assertTrue(
                    lo <= v <= hi,
                    "%s: the Python decoder produced %s = %r, outside the "
                    "%s the C decoder assigns it to (%d..%d). The two "
                    "decoders disagree on this value and no fixture contains "
                    "it." % (what, where, v, ctype, lo, hi))
                checked += 1
        self.assertTrue(checked, "%s: no integer field was checked at all"
                        % what)

    def test_the_bms_decoder_narrows_every_integer_field(self):
        for fill, what in ((0xffff, "every register 0xffff"),
                           (0x0000, "every register zero"),
                           (0x8000, "every register 0x8000")):
            with self.subTest(registers=what):
                regs = [fill] * self.fields.WF_BMS_MAX_REGS
                out = {}
                self.assertTrue(self.fields.bms_apply(out, regs),
                                "a full-width response was rejected")
                self.check("bms", out, what)

    def test_the_controller_decoder_narrows_every_integer_field(self):
        # Every frame type, so no group is missed for want of knowing which
        # types carry it; the ones the table does not cover assign nothing.
        for fill, what in ((0xff, "every payload byte 0xff"),
                           (0x00, "every payload byte zero"),
                           (0x80, "every payload byte 0x80")):
            with self.subTest(payload=what):
                live = {}
                payload = bytes([fill]) * 12
                for ftype in range(256):
                    self.fields.ctrl_apply(live, ftype, payload)
                self.check("controller", live, what)


class DocumentationIsGenerated(unittest.TestCase):
    """The third artefact. It is committed so that it can be read on the way
    past, which means it is the one that can go stale."""

    def test_the_document_the_table_names_exists(self):
        self.assertTrue(os.path.exists(DOC),
                        "field-table.json says the document is %s, and there "
                        "is no such file - so the test below would be "
                        "comparing against nothing" % DOC_REL)

    def test_committed_document_matches_the_table(self):
        with tempfile.TemporaryDirectory() as tmp:
            fresh = os.path.join(tmp, os.path.basename(DOC))
            subprocess.run([sys.executable, GENERATOR, TABLE, "--doc", fresh],
                           check=True)
            with open(fresh, encoding="utf-8") as f:
                want = f.read()
        with open(DOC, encoding="utf-8") as f:
            got = f.read()
        if got == want:
            return
        # Not assertEqual: the whole document as a diff buries the one line
        # that matters and the one command that fixes it.
        here = got.splitlines()
        fresh = want.splitlines()
        where = "the end of the file"
        for lineno, (a, b) in enumerate(zip(here, fresh), start=1):
            if a != b:
                where = ("line %d\n  committed: %s\n  generated: %s"
                         % (lineno, a, b))
                break
        self.fail("%s is not what field-table.json produces any more. It is "
                  "generated, not written: run `make docs`.\nFirst difference "
                  "at %s" % (os.path.relpath(DOC, ROOT), where))


if __name__ == "__main__":
    unittest.main()
