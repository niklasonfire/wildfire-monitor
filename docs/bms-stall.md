# Naming the BMS stall

The BMS link dies on a ~40.7 s cycle (issue #34), and two thirds of that cycle
is our own: 27 answers, 10 s of stale watchdog, 3 s of GATT rediscovery. The
27 is the part we do not understand. Something is spent once per exchange,
runs out at about 27, and is given back by the disconnect, and four candidates
survive every Capture in this repository:

1. the peer's answer path wedges - a bridge leaking one buffer per answer;
2. our RX chokes on the *large* answers - 129 bytes fragment where the
   Controller's 16 do not, and `ACL_FROM_LL_COUNT=24` sits uncomfortably close
   to 27;
3. our requests stop leaving the Monitor at all - the poll's write rc was
   never recorded, so no Capture we hold can say whether we stopped asking or
   the peer stopped answering;
4. a timer in the peer, ~27 s from the first request.

At one fixed poll period of one fixed width all four predict the file we
already have. Five parked runs at different periods and widths separate them,
in about twenty minutes. Nothing here is a ride.

## Before you start

* **Ignition on, bike parked.** With the ignition off the Controller does not
  advertise at all, and half of these runs are about it being there.
* The branch's firmware on the Monitor (`fix/34-bms-link-stall`): `captune`,
  the probe and the per-connection ledger only exist there.
* The Monitor on USB with `./wf.sh` on the other end, and room on the flash:
  `./wf.sh caps`, and `./wf.sh 'caprm all'` if the last session is still on it.
* `./wf.sh 'time set 2026-09-05 11:20:00'` (UTC) if the RTC is cold, so the
  files are stamped rather than "unknown".
* A notebook. **Write down which seq is which run.** The `tune` event says what
  the poll was doing, but only you know that this was run 3.

## The shape of every run

```bash
./wf.sh 'captune poll_ms 3000' 'captune probe 0' 'captune' 'cap rec'
#   ... wait the run's length by the clock, hands off the bike ...
./wf.sh 'cap stop' 'caps'
```

* `captune` is RAM only and is refused while a Capture is running, so every
  setting goes in **before** `cap rec`. A bare `captune` prints what is set;
  read it back before recording, because the Capture inherits it and the whole
  file means something else if it is wrong.
* Settings persist until reboot, so each run starts by setting everything it
  cares about, including putting back what the run before it changed.
* Every `./wf.sh` invocation opens the serial port, which **can reset the
  board** and end the run early (`docs/capture.md`). So: two invocations per
  run, nothing in between, and check `dur_ms` in the `caps` listing before
  believing a file. A short one is a reset, not a result.

## The runs

| # | tuning | length | what it decides |
| --- | --- | --- | --- |
| 1 | boot default, `probe 1` | 3 min | Is the peer still answering anything while stalled? Does a re-subscribe revive it? |
| 2 | `poll_ms 3000`, `probe 0` | 5 min | Count or clock: 27 answers in 81 s, or 9 in 27 s? |
| 3 | `poll_ms 400`, `probe 0` | 2 min | The same question from the other side. |
| 4 | `poll_regs 8`, `probe 0` | 2 min | Exchanges or bytes: 27 answers of 21 bytes, or ~165 of them? |
| 5 | `mcu_off 1`, `probe 0` | 2 min | Is the Controller's stream involved at all? |

**Run 1 - what the peer can still do.** Three minutes, boot defaults, probe on.

```bash
./wf.sh --reset 'captune' 'cap rec'  # boot defaults, read back, then record
./wf.sh 'cap stop' 'caps'            # after 3 minutes
```

`--reset` is what makes "boot default" true if anything was tuned earlier in
the session; it is the only run that wants it, because every other run sets
what it cares about by hand.

**Run 2 - count or clock, slowly.** A 3 s poll. If the limit is exchanges the
connection serves 27 answers over 81 s; if it is a timer it serves about 9 over
27 s.

```bash
./wf.sh 'captune poll_ms 3000' 'captune probe 0' 'captune' 'cap rec'
./wf.sh 'cap stop' 'caps'            # after 5 minutes
```

**Run 3 - the same question from the other side.** A 0.4 s poll: 27 answers in
10.8 s, or about 67 in 27 s.

```bash
./wf.sh 'captune poll_ms 400' 'captune probe 0' 'captune' 'cap rec'
./wf.sh 'cap stop' 'caps'            # after 2 minutes
```

**Run 4 - exchanges or bytes.** Eight registers is a 21-byte answer instead of
129. If the limit is exchanges the connection serves 27 answers and 567 bytes;
if it is a buffer it serves about 165 answers and the same 3483 bytes.

```bash
./wf.sh 'captune poll_ms 1000' 'captune poll_regs 8' 'captune probe 0' \
        'captune' 'cap rec'
./wf.sh 'cap stop' 'caps'            # after 2 minutes
```

Two minutes is enough only for the exchange answer, which drops every ~39 s.
If it is a buffer the first connection runs 165 s and is still up when the
Capture ends - which the script says in as many words, `no connection in this
file ended on its own`. That is already most of the answer; give it four
minutes and let one connection close so the row has a number in it.

**Run 5 - the control run.** The Controller's link off, so the BMS has the
radio to itself. Everything else back at the boot default.

```bash
./wf.sh 'captune poll_regs 0' 'captune mcu_off 1' 'captune probe 0' \
        'captune' 'cap rec'
./wf.sh 'cap stop' 'caps'            # after 2 minutes
```

Afterwards, `./wf.sh 'captune mcu_off 0'` or a reboot, so the next capture is a
capture and not another control run.

## Getting the files off

Over the cable, one file at a time - 115200 baud and a few thousand records,
so the timeout is not optional:

```bash
./wf.sh caps
./wf.sh --capture captures/cap0009_dump.log 'capdump 9@300'
```

Or over the air, once **all** the runs are done: `./wf.sh 'wifi on'` puts the
Monitor into Readout mode, the LCD shows the SSID and the password, and the
`.wfl` files are at `http://192.168.4.1/`. Readout mode takes BLE down for
good; the Monitor has to be rebooted before it can capture again, which is why
it goes last. `scripts/bms_stall.py` reads both shapes of file.

## Reading the answer

```bash
./scripts/bms_stall.py captures/cap00*_dump.log     # one row per Capture
./scripts/bms_stall.py -v captures/cap0010_dump.log # per connection and stall
```

The table's last three columns are the whole point:

| what stays constant across runs 2-4 | what is running out |
| --- | --- |
| answers/conn | exchanges - something spent once per request |
| seconds/conn | a timer in the peer |
| bytes/conn | a buffer, ours or the bridge's |

What each run is expected to print, per candidate:

| run | if exchanges | if a timer | if a buffer |
| --- | --- | --- | --- |
| 2 (3 s poll) | 27 answers, 81 s, 3483 B | ~9 answers, 27 s, 1161 B | 27 answers, 81 s, 3483 B |
| 3 (0.4 s poll) | 27 answers, 10.8 s, 3483 B | ~67 answers, 27 s, 8643 B | 27 answers, 10.8 s, 3483 B |
| 4 (8 registers) | 27 answers, 26.3 s, 567 B | ~27 answers, 27 s, 567 B | ~165 answers, 165 s, 3483 B |

Runs 2 and 3 separate the timer from the other two; run 4 separates the buffer
from the exchange count. Two runs agreeing is the result; one run is an
anecdote, which is what the existing Captures already are.

Read the lines above the table too. Anything starting `!!` is the instrument
saying it does not believe the file: a poll write that returned non-zero (the
request never left us, candidate 3), or the firmware's own per-connection
ledger disagreeing with the answers counted out of the records, which means one
of the two is wrong and neither number is usable until that is settled. `-v`
adds a row per connection - width, answers, seconds, bytes, median and p95 gap,
and how it ended - and a row per stall, saying whether the probe, a reconnect
or the peer itself ended the silence. Run 1 is read almost entirely there.

## Which run kills which candidate

| candidate | killed by |
| --- | --- |
| 3 - our requests stop leaving | Run 1. A `bms poll write rc=` line with a non-zero rc, which the script prints loudly, says the silence is ours. Silence with `rc=0` throughout, and a probe write that gets an ATT reply, says the writes leave and the peer's ATT layer is alive. |
| 2 - our RX chokes on large answers | Run 4. An 8-register answer is 21 bytes and needs no fragmentation, so if the link still stops at 27 answers, fragmentation is not what runs out. |
| 4 - a timer in the peer | Runs 2 and 3. A timer cannot serve 81 s in one and 10.8 s in the other; a count cannot serve 27 s in both. |
| 1 - the peer's answer path wedges | What is left standing if the three above fall, and run 1's probe describes it: writes and reads answered while notifications have stopped is a peer whose answer path is wedged and whose ATT layer is not. |
| the shared radio, or the Controller's load | Run 5. The same 27 with the Controller's link off is not a radio effect. |

If it is exchanges, the fix is to spend the budget more slowly rather than to
poll harder: ADR-0003 has the BMS as the Anchor and not the rate, so a 5 s poll
turns 27 answers into 135 s of Anchor.

## What the runs answered, 2026-09-05

Eight Captures, all parked with the ignition on, all in `captures/`. Run 4 was
never taken: runs 2 and 3 had already moved bytes per connection sevenfold with
the clock flat, which settles what run 4 was for.

| run | what it varied | answers/conn | seconds/conn | bytes/conn |
| --- | --- | --- | --- | --- |
| 1 | boot defaults, `probe 1` | 26 | 25.5 | 3354 |
| 2 | `poll_ms 3000` | 9 | 24.3 | 1161 |
| 3 | `poll_ms 400` | 66 | 26.8 | 8514 |
| 5 | `mcu_off 1` | 28 | 27.1 | 3580 |
| 6b | `itvl 24` - 30 ms, forced and verified | 27 | 26.3 | 3462 |
| 7 | `itvl 80` - 100 ms, forced and verified | 27 | 26.5 | 3483 |
| 8 | `poll_regs 62`, so no wide reads are attempted | 28 | 27.4 | 3590 |

**The BMS stops answering 26 to 27 seconds after the first answer, and nothing
the Monitor does moves that number.** The count of answers spans 9 to 66 across
these runs and the bytes span 1161 to 8514, both by a factor of seven; the
clock never leaves a one-second band.

Run 6 is not in the table. It was recorded with `itvl 24` set and is worthless
as an interval run: the knob wrote through `self_params` under
`BLE_GAP_EVENT_CONN_UPDATE_REQ`, and the Pack never uses that procedure. Its
numbers are a second control at the peer's own interval and agree with run 5.

### Do not re-run these

Every one of these is answered, with two runs agreeing rather than one:

* **Poll period.** Runs 2 and 3, 400 ms against 3000 ms.
* **Payload size and byte count.** The same two runs, sevenfold.
* **An exchange budget.** Same.
* **The Controller's link, and radio contention.** Run 5.
* **Our requests not leaving the Monitor.** Run 1's probe: `write_rsp rc=0`,
  ledger `writes=36 err=0`. Every request left.
* **Fragmentation of the 129-byte answers.** Run 3 served 66 answers on one
  connection against `CONFIG_BT_NIMBLE_TRANSPORT_ACL_FROM_LL_COUNT=24`.
* **The connection interval, and a budget counted in connection events.** Runs
  6b and 7 hold 30 ms and 100 ms against the Pack's own 15 ms - 1800, 900 and
  270 connection events inside the same 27 seconds. Flat.
* **The five 125-register reads each connection opens with.** Run 8 pins the
  width so they never happen. Flat, and the Pack answered none of the five in
  any run that attempted them.

### What the parameter lines say

`itvl` also bought a fact worth keeping. About 60 ms after every connection the
Pack asks, over L2CAP and never over the link layer, for `itvl=12` (15 ms) at
`latency=0`; rejected, it asks again for `itvl=15`. It wants the Monitor's
radio awake 66 times a second to answer once a second. Nothing in issue #34
depends on that, but a battery-powered Monitor might.

`BLE_GAP_EVENT_CONN_UPDATE_REQ` never fires on this peer. Only
`BLE_GAP_EVENT_L2CAP_UPDATE_REQ` does, and on that path NimBLE leaves
`self_params` NULL - writing through it panics, which is what
`ble_gap_rx_l2cap_update_req()` in the IDF makes plain and what the header's
own doc comment denies. Accept or reject is the whole vocabulary there; to hold
an interval the Monitor has to reject and then call `ble_gap_update_params()`
itself, which is what `link_setup()` now does.

### Still open

The Pack's own app polls at about 4 to 5 seconds and does not stall. A 27 s
wall clock should end its session too, so something separates the two and it is
none of the eight things above. Three candidates, none tested:

1. the app reconnects and does not say so;
2. the clock starts at the connection rather than at the first request, which
   no Capture here can tell apart because every run polls immediately after
   subscribing, and no `captune` knob delays the first poll;
3. the wedge clears on its own if nobody asks for a while - our poll has never
   paused.

2 and 3 are a phone and twenty minutes, and `docs/bms-stall.md` in the handoff
of 2026-09-05 has the frames and the procedure.
