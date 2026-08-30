#!/usr/bin/env python3
"""ADR-0002, asserted rather than asserted-to.

The Field Table's whole claim is that one description of the bytes produces
three artefacts that cannot disagree. Two tests hold it to that:

* the generated C decoder and the generated Python decoder are handed the same
  recorded Capture and have to produce identical numbers for every field of
  every record, and
* the committed field documentation has to be what the generator produces from
  the table as it stands today.

Both fail loudly. The first names the record and the field that diverged; the
second says which command puts it right.
"""
import glob
import json
import os
import subprocess
import sys
import tempfile
import unittest

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
