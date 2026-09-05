#!/usr/bin/env python3
"""Measure the BMS link's stall per connection, so a matrix of runs can name it.

The link dies on a ~40.7 s cycle (issue #34), and the shape underneath it is
sharper than the cycle: the BMS answers a fixed number of polls on a
connection and then goes silent, our stale watchdog drops the link, and a
rediscovery rebuilds it. Whether the thing that runs out is counted in
exchanges, in seconds or in bytes cannot be read off any single Capture,
because at one fixed poll period of one fixed width all three predict the same
file. It can be read off a matrix of parked Captures taken at different poll
periods and widths, which is what this reads:

    ./scripts/bms_stall.py captures/cap00*_dump.log      # a row per Capture
    ./scripts/bms_stall.py -v captures/cap0007_dump.log  # ... and per connection

    answers/conn constant while the clock moves -> the limit is exchanges
    seconds/conn constant                       -> a timer in the peer
    bytes/conn   constant                       -> a buffer

It reads the text `capdump` prints over the console, and the .wfl files the
Readout mode serves. The record grammar comes from scripts/dump2wfl.py and the
envelopes and the CRC from scripts/wfl.py; nothing about either format is
described twice.

**An answer is a frame that decodes as one, never a frame the dump labelled
`src=bms`.** That is load bearing rather than fastidious: cap0006 was written
by a firmware whose `capdump` printed the 20 Hz IMU records as `src=bms`, so
counting labels there gives 826 answers and "100 % anchored", the exact
opposite of what that file shows. Every frame record in the file, whatever its
label, goes through the 0xd2 envelope, the byte count and wfl's Modbus CRC, and
what survives that is an answer; answer_regs() says why that stops one step
short of wfl.decode_daly().

Older Captures predate most of the events this reads. Every one of them is
optional, and a Capture that carries none of them still produces every column
except the probe verdict and the firmware's own ledger.
"""
import argparse
import os
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dump2wfl  # noqa: E402  - the console dump grammar lives there
import wfl       # noqa: E402  - the archive format, the envelopes and the CRC

# A silence longer than this is a stall episode rather than a late answer. It
# is also roughly when the firmware's probe fires, so an episode is the window
# the probe reports inside.
STALL_S = 3.0

# The Anchor is present for as long as an answer is less than this old. Same
# threshold, used as a duty cycle rather than as an event.
ANCHOR_S = 3.0

EV_TUNE = re.compile(r"^tune\s+(?P<kv>.*\S)\s*$")
EV_CONNECTED = re.compile(r"^bms (?:connected|reconnected)\b")
EV_SUBSCRIBED = re.compile(r"^bms subscribed\b.*?(?:\bregs=(?P<regs>\d+))?\s*$")
EV_STALE = re.compile(r"^bms stale, no frame for (?P<ms>\d+) ms")
EV_DISCONNECTED = re.compile(r"^bms disconnected reason=(?P<reason>\d+)")
EV_LEDGER = re.compile(
    r"^bms link down \((?P<how>[^)]*)\) after (?P<answers>\d+) answers "
    r"in (?P<ms>\d+) ms(?:,\s*(?P<kv>.*\S))?\s*$")
EV_WRITE_RC = re.compile(
    r"^bms poll write rc=(?P<rc>-?\d+) after (?P<writes>\d+) writes")
EV_PROBE = re.compile(r"^bms probe (?P<what>\S+)\s*(?P<rest>.*)$")
EV_REVIVED = re.compile(r"^bms answered (?P<ms>\d+) ms after probe")


def answer_regs(payload):
    """How many registers this record answers with, or 0 if it is not an answer.

    wfl.decode_daly() is the authority on what a BMS answer is and stays it -
    the 0xd2 envelope, the byte count, the length, the Modbus CRC, and then
    the Field Table's register map. This repeats only the four envelope checks
    it makes before that map, for one reason: decode_daly() also refuses an
    answer narrower than WF_BMS_REG_NEEDED registers, because below that width
    the fields a ride needs are absent. Run 4 of docs/bms-stall.md polls 8
    registers on purpose, and those answers carry no fields and are still
    exchanges. This instrument counts exchanges, so it stops where the field
    map begins. The CRC is wfl's, and so is the ceiling.
    """
    if len(payload) < 7 or payload[0] != 0xd2 or payload[1] != 0x03:
        return 0
    n = payload[2]
    if n == 0 or n % 2 or len(payload) != 3 + n + 2:
        return 0
    if n // 2 > wfl.fields.WF_BMS_MAX_REGS:
        return 0
    if wfl.crc16(payload, init=0xFFFF) != 0:
        return 0
    return n // 2


class Capture:
    """One file, reduced to what this instrument looks at."""

    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        self.header = {}
        self.answers = []      # (t_ms, payload bytes)
        self.events = []       # (t_ms, text)
        self.span_ms = 0
        self.frames = 0
        self.labelled_bms = 0  # what trusting src= would have counted


def read_capture(path):
    """A Capture from either shape of file: the console text or the .wfl."""
    cap = Capture(path)
    with open(path, "rb") as f:
        head = f.read(len(wfl.MAGIC))
    if head.startswith(wfl.MAGIC[:6]):
        with open(path, "rb") as f:
            cap.header = wfl.read_header(f)
            stream = list(wfl.records(f))
    else:
        with open(path, "r", errors="replace") as f:
            cap.header, stream = read_dump(f)

    for rtype, t_ms, payload in stream:
        cap.span_ms = max(cap.span_ms, t_ms)
        if rtype == wfl.WFREC_EVENT:
            cap.events.append((t_ms, payload.split(b"\0")[0]
                               .decode(errors="replace").strip()))
            continue
        if rtype not in (wfl.WFREC_MCU, wfl.WFREC_BMS):
            continue
        cap.frames += 1
        if rtype == wfl.WFREC_BMS:
            cap.labelled_bms += 1
        # The label is a hint at most; the envelope and the CRC decide.
        n_reg = answer_regs(payload)
        if n_reg:
            cap.answers.append((t_ms, len(payload), n_reg))
    return cap


def read_dump(fh):
    """The console text, tolerantly: a log is a session, not only a Capture.

    dump2wfl.parse() refuses anything it does not recognise, which is right
    for a writer that has to rebuild a file byte for byte and wrong here - a
    log that also holds the prompt, an echoed `captune` or a second `capdump`
    still holds one readable Capture. The line grammar is dump2wfl's, so the
    two agree on what a record is; only the strictness differs.
    """
    header, records = {}, []
    for raw in fh:
        line = raw.strip()
        if not line:
            continue
        if not header:
            m = dump2wfl.HDR_RE.match(line)
            if m:
                header = m.groupdict()
            continue
        m = dump2wfl.FRAME_RE.match(line)
        if m:
            rtype = (int(m["type"], 0) if m["type"]
                     else dump2wfl.SRC_TYPE[m["src"]])
            records.append((rtype, int(m["t"]), bytes.fromhex(m["data"])))
            continue
        m = dump2wfl.EVENT_RE.match(line)
        if m:
            records.append((wfl.WFREC_EVENT, int(m["t"]),
                            m["text"].encode("ascii", "replace")))
            continue
        # Telemetry, IMU, console noise: nothing this reads.
    if not header:
        raise ValueError("no CAPHDR line - is this a capdump log?")
    return header, records


class Connection:
    """One BMS connection: opened, served some answers, ended somehow."""

    def __init__(self, index, start_ms, opened):
        self.index = index
        self.start_ms = start_ms
        self.opened = opened     # False for a link already up at capture start
        self.regs = None         # the width the subscribe event recorded
        self.answer_regs = set()  # the widths the answers actually carry
        self.answers = []        # (t_ms, nbytes)
        self.bytes = 0
        self.end_ms = None
        self.ended = None        # human-readable
        self.closed = False      # the peer stopped and we saw the drop
        self.stale_ms = None
        self.ledger = None
        self.write_rc = []       # (t_ms, rc, writes) for non-zero rc only

    def close(self, t_ms, why):
        """The first ending wins: a stale drop is followed by its disconnect."""
        if not self.closed:
            self.end_ms = t_ms
            self.ended = why
            self.closed = True

    def width(self):
        """The poll width, from the subscribe event or from the answers."""
        if self.regs is not None:
            return str(self.regs)
        if not self.answer_regs:
            return "-"
        return "/".join(str(r) for r in sorted(self.answer_regs))

    @property
    def n(self):
        return len(self.answers)

    @property
    def served_ms(self):
        if self.n < 2:
            return 0
        return self.answers[-1][0] - self.answers[0][0]

    def gaps_s(self):
        t = [a[0] for a in self.answers]
        return [(b - a) / 1000.0 for a, b in zip(t, t[1:])]


class Episode:
    """One stall: a silence longer than STALL_S, however it ended."""

    def __init__(self, start_ms, end_ms, ended, probe, revived_ms):
        self.start_ms = start_ms
        self.end_ms = end_ms
        self.ended = ended
        self.probe = probe           # list of "what rc=.." strings
        self.revived_ms = revived_ms  # ms from probe to the answer, or None

    @property
    def silent_ms(self):
        return self.end_ms - self.start_ms


def tune_of(cap):
    """The `captune` settings the firmware wrote in, or None."""
    for _t, text in cap.events:
        m = EV_TUNE.match(text)
        if m:
            return m["kv"]
    return None


def connections(cap):
    """Split the answers into connections on the events the firmware writes."""
    marks = [(t, text) for t, text in cap.events
             if EV_CONNECTED.match(text) or EV_SUBSCRIBED.match(text)
             or EV_STALE.match(text) or EV_DISCONNECTED.match(text)
             or EV_LEDGER.match(text) or EV_WRITE_RC.match(text)]

    opens = [t for t, text in marks if EV_CONNECTED.match(text)]
    starts = list(opens)
    # A Capture can start with the link already up, or with answers arriving
    # before the first event; that connection is partial by construction.
    if not starts or (cap.answers and cap.answers[0][0] < starts[0]):
        starts.insert(0, 0)
        opened = [False] + [True] * len(opens)
    else:
        opened = [True] * len(opens)

    conns = [Connection(i + 1, t, o)
             for i, (t, o) in enumerate(zip(starts, opened))]

    def current(t_ms):
        pick = None
        for c in conns:
            if c.start_ms <= t_ms:
                pick = c
            else:
                break
        return pick

    for t_ms, nbytes, n_reg in cap.answers:
        c = current(t_ms)
        if c is not None:
            c.answers.append((t_ms, nbytes, n_reg))
            c.bytes += nbytes
            c.answer_regs.add(n_reg)

    for t_ms, text in marks:
        c = current(t_ms)
        if c is None:
            continue
        m = EV_SUBSCRIBED.match(text)
        if m and m["regs"]:
            c.regs = int(m["regs"])
        m = EV_STALE.match(text)
        if m:
            c.stale_ms = int(m["ms"])
            c.close(t_ms, f"stale after {int(m['ms']) / 1000.0:.1f} s")
        m = EV_DISCONNECTED.match(text)
        if m:
            c.close(t_ms, f"disconnected reason={m['reason']}")
        m = EV_LEDGER.match(text)
        if m:
            c.ledger = m.groupdict()
            c.close(t_ms, f"link down ({m['how']})")
        m = EV_WRITE_RC.match(text)
        if m and int(m["rc"]) != 0:
            c.write_rc.append((t_ms, int(m["rc"]), int(m["writes"])))

    for i, c in enumerate(conns):
        horizon = (conns[i + 1].start_ms if i + 1 < len(conns) else cap.span_ms)
        if not c.closed:
            c.end_ms = horizon
            c.ended = c.ended or "capture end"
    return [c for c in conns if c.n]


def episodes(cap):
    """Every silence longer than STALL_S, with how it ended."""
    out = []
    if not cap.answers:
        return out
    edges = [(a[0], b[0]) for a, b in zip(cap.answers, cap.answers[1:])
             if b[0] - a[0] > STALL_S * 1000]
    tail_start = cap.answers[-1][0]
    if cap.span_ms - tail_start > STALL_S * 1000:
        edges.append((tail_start, cap.span_ms))

    for start_ms, end_ms in edges:
        inside = [(t, text) for t, text in cap.events
                  if start_ms < t <= end_ms]
        probe, revived, rebuilt = [], None, False
        for t, text in inside:
            m = EV_PROBE.match(text)
            if m:
                probe.append(" ".join(x for x in (m["what"], m["rest"]) if x))
            m = EV_REVIVED.match(text)
            if m:
                revived = int(m["ms"])
            if EV_CONNECTED.match(text) or EV_DISCONNECTED.match(text):
                rebuilt = True
        if end_ms >= cap.span_ms:
            ended = "capture end"
        elif revived is not None:
            ended = "probe"
        elif rebuilt:
            ended = "reconnect"
        else:
            ended = "peer resumed"
        out.append(Episode(start_ms, end_ms, ended, probe, revived))
    return out


def bursts(answers):
    """One connection's answers split on its own stalls.

    With the probe off a connection is one burst and the two are the same
    number. With the probe on a stall can end without a disconnect, so a
    connection carries several bursts and it is the burst, not the connection,
    that the limit is spent on - which is worth saying out loud rather than
    leaving in a column that quietly doubled."""
    out, run = [], []
    for a in answers:
        if run and a[0] - run[-1][0] > STALL_S * 1000:
            out.append(run)
            run = []
        run.append(a)
    if run:
        out.append(run)
    return out


def anchored_ms(cap):
    """How long the Anchor was present: an answer is good for ANCHOR_S."""
    total, prev_end = 0, 0
    for t_ms, _nbytes, _n_reg in cap.answers:
        start = max(prev_end, t_ms)
        end = min(t_ms + ANCHOR_S * 1000, cap.span_ms)
        if end > start:
            total += end - start
        prev_end = max(prev_end, end)
    return total


def p95(values):
    if not values:
        return 0.0
    s = sorted(values)
    return s[min(len(s) - 1, int(round(0.95 * (len(s) - 1))))]


def span(values, decimals=0):
    """`mean [min-max]`, so a constant is visibly constant."""
    if not values:
        return "-"
    lo, hi = min(values), max(values)
    return (f"{statistics.mean(values):.{decimals}f} "
            f"[{lo:.{decimals}f}-{hi:.{decimals}f}]")


def check_ledger(conn, out):
    """Cross-check the firmware's own count against ours; disagree loudly."""
    led = conn.ledger
    if not led:
        return
    kv = dict(re.findall(r"(\w+)=(-?\d+)", led.get("kv") or ""))
    said = int(led["answers"])
    if said != conn.n:
        out.append(f"!! conn {conn.index}: the firmware counted {said} answers, "
                   f"this file holds {conn.n}. One of the two instruments is "
                   f"wrong - do not trust either number until that is settled.")
    if "bytes" in kv and int(kv["bytes"]) != conn.bytes:
        out.append(f"!! conn {conn.index}: the firmware counted {kv['bytes']} "
                   f"bytes of answers, this file holds {conn.bytes}.")
    said_ms = int(led["ms"])
    if abs(said_ms - conn.served_ms) > 2000:
        out.append(f"!! conn {conn.index}: the firmware served {said_ms} ms, "
                   f"first answer to last is {conn.served_ms} ms.")
    if int(kv.get("err", 0)):
        out.append(f"!! conn {conn.index}: {kv['err']} poll writes failed - "
                   f"we may have stopped asking, see candidate 3 in #34.")


def report(cap, verbose, out):
    conns = connections(cap)
    eps = episodes(cap)
    # Only a connection this file saw both ends of measures a limit.
    closed = [c for c in conns if c.closed and c.opened]
    stats = closed or conns

    seq = cap.header.get("seq", "?")
    note = cap.header.get("note") or ""
    out.append(f"{cap.name}  seq={seq}  {cap.span_ms / 1000.0:.1f} s"
               + (f"  note={note}" if note else ""))

    tune = tune_of(cap)
    out.append(f"  tune  {tune}" if tune else
               "  tune  none recorded - this Capture predates `captune`, so "
               "the poll ran at its boot defaults")

    if not closed and conns:
        out.append("  note  no connection in this file ended on its own, so "
                   "the row below counts connections the Capture cut short - "
                   "a longer run is needed before it means anything")

    if cap.labelled_bms != len(cap.answers):
        out.append(f"  note  {cap.labelled_bms} records are labelled src=bms, "
                   f"{len(cap.answers)} decode as answers; the labels are "
                   f"ignored (see the module docstring)")

    if verbose and conns:
        out.append("")
        out.append("  conn  start   regs  answers  served_s  bytes  gap_med  "
                   "gap_p95  ended")
        for c in conns:
            gaps = c.gaps_s()
            out.append(
                f"  {c.index:<4}  {c.start_ms / 1000.0:>6.1f}  "
                f"{c.width():>4}  {c.n:>7}  "
                f"{c.served_ms / 1000.0:>8.1f}  {c.bytes:>5}  "
                f"{statistics.median(gaps) if gaps else 0:>7.2f}  "
                f"{p95(gaps):>7.2f}  {c.ended}"
                + ("" if c.closed else "  (partial)"))
        if len(conns) != len(closed):
            out.append(f"  {len(conns) - len(closed)} connection(s) were still "
                       f"open when the Capture ended and are left out of the "
                       f"row below, because a truncated one is not a limit.")

    if verbose and eps:
        out.append("")
        out.append("  stall  start   silent_s  ended by      probe")
        for i, e in enumerate(eps, 1):
            probe = ", ".join(e.probe) if e.probe else "-"
            if e.revived_ms is not None:
                probe += f" -> answered {e.revived_ms} ms later"
            out.append(f"  {i:<5}  {e.start_ms / 1000.0:>6.1f}  "
                       f"{e.silent_ms / 1000.0:>8.1f}  {e.ended:<12}  {probe}")

    for c in conns:
        parts = bursts(c.answers)
        if len(parts) > 1:
            out.append(f"  note  conn {c.index} served {len(parts)} bursts of "
                       + "/".join(str(len(b)) for b in parts)
                       + " answers - a stall ended without a disconnect, so "
                         "the limit is spent per burst here and the row below "
                         "counts the whole connection")
        check_ledger(c, out)
        for t_ms, rc, writes in c.write_rc:
            out.append(f"!! {t_ms / 1000.0:.1f} s: poll write rc={rc} after "
                       f"{writes} writes - the request did not leave the "
                       f"Monitor, so this silence is ours, not the peer's.")

    probed = [e for e in eps if e.probe]
    if probed:
        revived = [e for e in probed if e.revived_ms is not None]
        att = [e for e in probed if any("rc=0" in p for p in e.probe)]
        out.append(f"  probe  {len(probed)} stall(s) probed, "
                   f"{len(att)} got an ATT reply, {len(revived)} answered "
                   f"again without a reconnect"
                   + (" (median %.0f ms)" % statistics.median(
                       [e.revived_ms for e in revived]) if revived else ""))
    out.append("")

    anchored = anchored_ms(cap)
    return {
        "name": cap.name,
        "conns": len(conns),
        "closed": len(closed),
        "answers": [c.n for c in stats],
        "seconds": [c.served_ms / 1000.0 for c in stats],
        "bytes": [c.bytes for c in stats],
        "silent_s": (cap.span_ms - anchored) / 1000.0,
        "anchored": 100.0 * anchored / cap.span_ms if cap.span_ms else 0.0,
    }


ROW = ("{name:<20}  {conns:>7}  {answers:<16}  {seconds:<16}  {bytes:<17}  "
       "{silent:>8}  {anchored:>8}")
HEAD = ROW.format(name="Capture", conns="conns", answers="answers/conn",
                  seconds="seconds/conn", bytes="bytes/conn",
                  silent="silent_s", anchored="anchored")
LEGEND = """
answers/conn constant while the clock moves  ->  the limit is counted in exchanges
seconds/conn constant while the count moves  ->  a timer in the peer
bytes/conn   constant while both move        ->  a buffer, ours or the bridge's

conns is closed/seen: every cell is mean [min-max] over the connections that
ended on their own, because one the operator cut short is not a limit.
seconds/conn is first answer to last. anchored is the share of the Capture with
an answer less than 3 s old - the Anchor's duty cycle, which is what a ride
actually loses."""


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help="capdump logs or .wfl Captures")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="one row per connection and per stall episode too")
    args = ap.parse_args()

    rows, lines = [], []
    for path in args.files:
        try:
            cap = read_capture(path)
        except (OSError, ValueError) as e:
            lines.append(f"!! {path}: {e}")
            lines.append("")
            continue
        if not cap.answers:
            lines.append(f"{os.path.basename(path)}: no answer decodes in this "
                         f"file ({cap.frames} frames read); nothing to measure.")
            lines.append("")
            continue
        rows.append(report(cap, args.verbose, lines))

    print("\n".join(lines))
    if not rows:
        return 1

    print(HEAD)
    for r in rows:
        print(ROW.format(name=r["name"][:20],
                         conns=f"{r['closed']}/{r['conns']}",
                         answers=span(r["answers"]),
                         seconds=span(r["seconds"], 1),
                         bytes=span(r["bytes"]),
                         silent=f"{r['silent_s']:.1f}",
                         anchored=f"{r['anchored']:.1f} %"))
    print(LEGEND)
    return 0


if __name__ == "__main__":
    sys.exit(main())
