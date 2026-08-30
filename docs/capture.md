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
Table, the Confidence of each entry and what it's sourced from. Bytes
8..9/10..11 of the motion block remain genuinely unassigned. Assigning
meaning to a field from scratch still needs a capture taken **while riding**,
which the host-driven flow above cannot deliver, because it needs a PC on the
serial link. That is what the standalone capture below is for.

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
wifi on              # Wi-Fi readout mode, for pulling files off without a cable
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
