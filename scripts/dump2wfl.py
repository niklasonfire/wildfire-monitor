#!/usr/bin/env python3
"""Rebuild a .wfl Capture from the text form `capdump` prints over the console.

The Monitor writes Captures as .wfl - one header, then a stream of records -
but the only copies of the first rides that left the bike left it as console
text, because a serial line was all that was attached at the time. Those dumps
carry the raw hex of every record, so the archive file can be rebuilt from
them, and the result is a Capture like any other: the same bytes the
Controller and the BMS sent, and nothing the Monitor worked out (ADR-0001).

    ./scripts/dump2wfl.py captures/cap0007_dump.log tests/fixtures/cap0007.wfl

The format constants come from scripts/wfl.py, which reads the files this
writes; keeping one description of the layout is the point.

Two header fields cannot be recovered, because `capdump` does not print them:
`hdr_len`, which is written back as sizeof(wflog_hdr_t) - the only value the
firmware has ever used - and `duration_ms`, which store_end() back-patches on
the board and which is left at 0 here rather than guessed. 0 is the archive's
own "unknown", so nothing downstream has to learn a new case.

The record grammar is what `capdump` emits; both spellings seen in the
checked-in dumps are accepted (cap0007 writes `type=0x01` on frame lines,
cap0006 does not). Anything before the CAPHDR line is console noise and is
skipped. Marker text is carried through verbatim: a Marker is a WFREC_EVENT
whose text starts with "marker", not a record type of its own.
"""
import argparse
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wfl  # noqa: E402  - the format lives there, see the docstring

WFLOG_VERSION = 1
SRC_TYPE = {"mcu": wfl.WFREC_MCU, "bms": wfl.WFREC_BMS}

HDR_RE = re.compile(
    r"^CAPHDR\s+seq=(?P<seq>\d+)\s+version=(?P<version>\d+)\s+"
    r"unix_start=(?P<unix_start>-?\d+)\s+boot_ms=(?P<boot_ms>\d+)\s+"
    r"mcu=(?P<mcu>\S*)\s+bms=(?P<bms>\S*)\s*(?:note=(?P<note>.*))?$")
FRAME_RE = re.compile(
    r"^WFR\s+t=(?P<t>\d+)\s+src=(?P<src>mcu|bms)\s+(?:type=(?P<type>\S+)\s+)?"
    r"len=(?P<len>\d+)\s+data=(?P<data>[0-9a-fA-F]*)$")
EVENT_RE = re.compile(r"^WFE\s+t=(?P<t>\d+)\s(?P<text>.*)$")
TELEM_RE = re.compile(
    r"^WFT\s+t=(?P<t>\d+)\s+batt_mv=(?P<batt>\d+)\s+rssi_mcu=(?P<rmcu>-?\d+)\s+"
    r"rssi_bms=(?P<rbms>-?\d+)\s+mcu=(?P<nmcu>\d+)\s+bms=(?P<nbms>\d+)\s+"
    r"dropped=(?P<drop>\d+)$")
IMU_RE = re.compile(
    r"^WFI\s+t=(?P<t>\d+)\s+a=(?P<ax>-?\d+),(?P<ay>-?\d+),(?P<az>-?\d+)\s+"
    r"g=(?P<gx>-?\d+),(?P<gy>-?\d+),(?P<gz>-?\d+)$")


class ParseError(Exception):
    pass


def fixed(text, size):
    """A NUL-padded fixed-width char array, the way the firmware writes one."""
    raw = text.encode("ascii", "replace")[:size - 1]
    return raw + b"\0" * (size - len(raw))


def parse(lines):
    """Returns (header dict, [(type, t_ms, payload), ...]) in file order."""
    header = None
    records = []

    for lineno, raw in enumerate(lines, 1):
        line = raw.rstrip("\r\n")
        if header is None:
            # Console noise: the prompt, the echoed command, the banner.
            if not line.startswith("CAPHDR"):
                continue
            m = HDR_RE.match(line)
            if m is None:
                raise ParseError(f"line {lineno}: unreadable CAPHDR: {line}")
            header = m.groupdict()
            continue

        if not line or line.startswith(("CAPDUMP_END", "wf>")):
            continue

        m = FRAME_RE.match(line)
        if m:
            payload = bytes.fromhex(m["data"])
            if len(payload) != int(m["len"]):
                raise ParseError(f"line {lineno}: len={m['len']} but "
                                 f"{len(payload)} bytes of hex")
            rtype = int(m["type"], 0) if m["type"] else SRC_TYPE[m["src"]]
            records.append((rtype, int(m["t"]), payload))
            continue

        m = EVENT_RE.match(line)
        if m:
            # The firmware writes strlen() bytes, with no terminator.
            records.append((wfl.WFREC_EVENT, int(m["t"]),
                            m["text"].encode("ascii", "replace")))
            continue

        m = TELEM_RE.match(line)
        if m:
            payload = struct.pack(wfl.TELEM_FMT, int(m["batt"]), int(m["rmcu"]),
                                  int(m["rbms"]), int(m["nmcu"]),
                                  int(m["nbms"]), int(m["drop"]))
            records.append((wfl.WFREC_TELEM, int(m["t"]), payload))
            continue

        m = IMU_RE.match(line)
        if m:
            payload = struct.pack(wfl.IMU_FMT, *(int(m[k]) for k in
                                                 ("ax", "ay", "az",
                                                  "gx", "gy", "gz")))
            records.append((wfl.WFREC_IMU, int(m["t"]), payload))
            continue

        raise ParseError(f"line {lineno}: unrecognised record: {line}")

    if header is None:
        raise ParseError("no CAPHDR line in the dump")
    if int(header["version"]) != WFLOG_VERSION:
        raise ParseError(f"dump says version={header['version']}, "
                         f"this writer only knows {WFLOG_VERSION}")
    for rtype, t_ms, payload in records:
        if len(payload) > 255:
            raise ParseError(f"t={t_ms} type=0x{rtype:02x}: payload of "
                             f"{len(payload)} bytes does not fit a uint8_t")
    return header, records


def build(header, records):
    hdr_len = struct.calcsize(wfl.HDR_FMT)
    out = bytearray(struct.pack(
        wfl.HDR_FMT, wfl.MAGIC, int(header["version"]), hdr_len,
        int(header["seq"]), int(header["unix_start"]), int(header["boot_ms"]),
        0,  # duration_ms: not in the dump, see the module docstring
        fixed(header["mcu"], 18), fixed(header["bms"], 18),
        fixed(header["note"] or "", 32)))
    for rtype, t_ms, payload in records:
        out += struct.pack(wfl.REC_FMT, rtype, len(payload), t_ms)
        out += payload
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump", help="capdump text log")
    ap.add_argument("out", help="where to write the .wfl")
    args = ap.parse_args()

    with open(args.dump, "r", errors="replace") as f:
        try:
            header, records = parse(f)
        except ParseError as e:
            sys.exit(f"{args.dump}: {e}")

    blob = build(header, records)
    with open(args.out, "wb") as f:
        f.write(blob)

    counts = {}
    for rtype, _, _ in records:
        counts[rtype] = counts.get(rtype, 0) + 1
    print(f"{args.out}: seq={header['seq']} {len(blob)} bytes, "
          f"{len(records)} records "
          f"(mcu={counts.get(wfl.WFREC_MCU, 0)} bms={counts.get(wfl.WFREC_BMS, 0)} "
          f"event={counts.get(wfl.WFREC_EVENT, 0)} "
          f"telem={counts.get(wfl.WFREC_TELEM, 0)} "
          f"imu={counts.get(wfl.WFREC_IMU, 0)})")


if __name__ == "__main__":
    main()
