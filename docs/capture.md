# Running a capture

The procedure below is the one that produced `captures/mcu_frames.log`, with
the failure modes that were actually hit along the way. It assumes the bike is
in front of you and the M5StickC is on USB.

## Before anything

The ignition has to be **on**. With the ignition off the Fardriver does not
advertise at all, so a scan finds the Daly BMS and nothing else.

## The short version

```bash
./wf.sh --reset --capture captures/session.log \
        'scan 8@15' 'connect name Yuan@30' 'mtu@10' 'discover@40' \
        'sub all@20' 'rec 20@40' 'rec dump@200'
```

That is a complete, self-contained capture: scan, connect, negotiate a 247
byte MTU, walk the GATT database, subscribe every CCCD, record for 20 s and
print the frames.

## Step by step, and why each step is there

1. **`scan 8@15`** - the Fardriver answers as `YuanQuFOC274` and advertises
   service `0xffe0`. Its address rotates between sessions, so note the name,
   never the MAC.
2. **`connect name Yuan@30`** - matching on the name substring is what makes
   the rotation irrelevant. Connecting by MAC needs the address type spelled
   out, and both bike devices are `random`.
3. **`mtu@10`** - the controller accepts 247. Not required for 16-byte frames,
   but it removes MTU as a variable.
4. **`discover@40`** - required before `sub`, `readall` or `daly`; those
   commands refuse to run without a discovered database.
5. **`sub all@20`** - this is the entire Fardriver protocol. Once the CCCD at
   `0x0011` is written, frames arrive unsolicited at ~35.5 Hz. Nothing is ever
   sent to the controller.
6. **`rec 20@40`** - records into RAM. Frames are *not* printed live, because
   35.5 frames/s of hex overruns the 115200 baud console.
7. **`rec dump@200`** - prints the recording afterwards, timestamped in
   microseconds. Give it a generous timeout; the dump itself is slow.

`probe <secs>` collapses steps 3-6 into one command and adds `readall` and
notification statistics, which is the better first contact with an unknown
device. It is worse for a clean data capture, because the reads and the
statistics run inside the same window.

## Things that will bite you

* **`--capture` takes a path inside the container.** The project is bind
  mounted at `/project` and that is the working directory, so a relative path
  works and a host absolute path such as `/home/you/...` fails - and it fails
  by producing no output at all rather than an error you will notice.
* **The record buffer is 20 KB**, which is 24 bytes per frame (16 payload plus
  an 8 byte header) and therefore about 853 frames, or roughly **24 seconds**
  at the observed rate. `rec 30` silently drops the overflow; check the
  `dropped=` field in `REC_STOP`. For anything longer, stream with `--listen`
  instead of recording.
* **Every `./wf.sh` invocation opens the serial port**, which can reset the
  board and lose an open connection and the recording with it. Put the whole
  session into one invocation rather than chaining separate calls.
* **`--reset` gives you a deterministic starting point** but throws away any
  previous scan table and connection.

## Checking a capture

Every frame carries a Modbus CRC-16 with the unusual initial value `0x7f3c`.
Running it across all 16 bytes leaves a residue of 0:

```python
def crc16(d, init=0x7f3c):
    c = init
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

# frame is the 16 raw bytes
assert crc16(frame) == 0
```

All 709 frames of `captures/mcu_frames.log` pass this.

## What is still missing

Parked, only payload bytes 8..9 and 10..11 of the eight live frame types move,
and they sit at -1/-2 and 0/0xffff - idle noise. That is still true as a
description of a parked capture, but it does not mean these are the only
telemetry fields: `cur_rpm` lives at payload bytes 6..7 of all eight of those
types - they are the `motion` block - and simply reads 0 while parked,
indistinguishable from a static field. See `docs/field-table.md` for the Field
Table, the Confidence of each entry and what it's sourced from.

Bytes 8..9/10..11 of the motion block are the torque current and its companion,
assigned by cap0002 and confirmed by ride0 - which opened the throttle to its
stop with the drive disengaged and watched them stay at zero, because there was
no torque to report. The throttle itself is at type `0x99`, payload bytes 0..1,
and a parked capture is exactly what found it: it is the only field on the link
a stationary rider can move, so it is the one thing the flow above *can*
assign. Everything else still needs a capture taken **while riding**, which the
host-driven flow cannot deliver because it needs a PC on the serial link. That
is what the standalone capture below is for.

## Standalone capture

The flow above needs `./wf.sh` on the other end of the USB cable, so it cannot
follow the bike. The standalone capture runs on the Monitor alone: it connects
to both devices by itself, writes a Capture to flash as a `.wfl` file, and is
driven from the console only before and after the ride.

```
cap status | scan | idle | rec | stop | mark [text]
caps                 # list the Captures on flash
capdump <seq>        # print one over the console
caprm <seq>|all      # delete
wifi on              # Service Mode, for pulling files off without a cable
```

`cap rec` starts recording and `cap stop` ends it; `cap mark` drops a Marker at
the moment a deliberate manoeuvre starts or ends, which is what separates an
experiment from a blob of frames. A Capture holds only what the two devices
said - never anything the Monitor worked out from it, per ADR-0001 - so it can
be re-decoded whenever the Field Table improves.

Afterwards, off the bike:

```bash
./scripts/wfl.py cap0007.wfl              # tagged one-line records
./scripts/wfl.py cap0007.wfl --fields     # every decoded field, per record
```

## The BMS poll is wider than anything we have recorded

Every Capture in this repository was taken while the poll asked for **62**
registers, and that is the only width this BMS has ever been seen to answer.
The poll now asks for **125**, which is as many as one answer can carry: the
reply to a read of `n` registers is `5 + 2n` bytes, a Capture record's payload
length is a `uint8_t`, and `5 + 2 x 125` is 255 exactly. Registers 62-124 have
therefore never been read by anything, and nothing in that range is in the
Field Table. Rated Capacity and the cycle count are believed to be up there,
and until one Capture comes back containing them, State of Health is not
measurable at all.

**What to do, in order of how cheap it is.** The first is a bench session with
the ignition on and no riding at all:

```bash
./wf.sh --reset --capture captures/daly_wide.log \
        'scan 8@15' 'connect name DL@30' 'discover@40' 'daly probe@40'
```

`daly probe` now fires `d2 03 0000 007d` (the wide read the ride will use) and
`d2 03 007d 007d` (the block above it) alongside the four requests it always
sent. Three outcomes, and each one is worth knowing:

* a 255-byte answer to `d2_wide` - the ride will record registers 62-124, so
  go and take the ride;
* an exception frame, or silence - the BMS does not serve that block, the
  capture will fall back to 62 registers on its own after five seconds, and
  finding the real width is a matter of bisecting with
  `daly read 0xd2 0 <count>`;
* an answer to `d2_above` as well - there is more above register 124 than one
  request can carry, and a second poll at a second address is worth building.

Then the ride. **Two Captures separated by a full charge** is what actually
closes it: a register that does not move within either ride but is one higher
in the second is the cycle count, and a register near 500 that does not move at
all is Rated Capacity in 0.1 Ah - `remaining_ah / (soc_pct / 100)` says to
expect 50.0 Ah, and the check is that the read value and the derived one agree.
A single parked Capture cannot say either thing, because over 47 s almost
everything is constant.

While the charger is plugged in, take a third short Capture. That one settles
register 47 and the four switch-state registers 50, 52, 53 and 54, which are
constant in everything we hold and therefore indistinguishable from each other;
see "Not decoded" in `docs/field-table.md`.

Each Capture records the width it actually used, in the `bms subscribed ...
regs=N` event, so a Capture that came back narrow says whether that was asked
for or fallen back to.
