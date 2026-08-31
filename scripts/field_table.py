#!/usr/bin/env python3
"""Hands the offline tools the generated Python decoder.

ADR-0002 generates the decoding from `field-table.json` at build time, into a
build directory that is gitignored. That works for the firmware and for the
host build, which both have a build step; it does not work for a script
somebody runs straight out of the repository. So this bootstrap does what the
build would have done: if the generated module is missing or older than the
Field Table it was generated from, it runs the generator first.

That is the only hand-written thing here. Everything about what the bytes mean
comes back out of the generated module.

    from field_table import load
    fields = load()
    fields.bms_apply(out, registers)
"""
import os
import subprocess
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TABLE = os.path.join(ROOT, "field-table.json")
GENERATOR = os.path.join(ROOT, "scripts", "gen_fields.py")

# The Makefile generates into build-host/gen; the ESP-IDF build generates into
# its own build directory and sets this, so nothing has to guess.
OUT_DIR = os.environ.get("WF_GEN_DIR", os.path.join(ROOT, "build-host", "gen"))
MODULE = os.path.join(OUT_DIR, "wf_fields.py")

_cached = None


def _stale():
    if not os.path.exists(MODULE):
        return True
    made = os.path.getmtime(MODULE)
    return any(os.path.getmtime(src) > made for src in (TABLE, GENERATOR))


def load():
    """The generated decoder, regenerating it first if the table has moved."""
    global _cached
    if _cached is not None:
        return _cached
    if _stale():
        subprocess.run([sys.executable, GENERATOR, TABLE, "--py-dir", OUT_DIR],
                       check=True)
    if not os.path.exists(MODULE):
        raise RuntimeError(f"{MODULE} was not generated; run `make gen`")
    # Compiled and exec'd rather than imported, so that no __pycache__ sits
    # between the Field Table and what actually runs. Python's bytecode cache
    # keys on the source's size and mtime to the second, and a regenerated
    # decoder can match an older one on both - which would leave a test that
    # exists to catch drift quietly running the drifted-from version.
    with open(MODULE, "r", encoding="utf-8") as f:
        source = f.read()
    module = types.ModuleType("wf_fields")
    module.__file__ = MODULE
    exec(compile(source, MODULE, "exec"), module.__dict__)
    _cached = module
    return module
