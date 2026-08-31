# wildfire_monitor

Firmware for an M5StickC PLUS2 (ESP32-PICO-V3-02: single-core-package ESP32,
8 MB flash, 2 MB PSRAM), built with ESP-IDF **v6.1** inside Docker. No toolchain
is installed on the host.

## Requirements

* Docker (the user must be in the `docker` group)
* The M5StickC PLUS2 connected over USB-C. Its CH9102 USB-serial bridge
  (`1a86:55d4`) shows up as `/dev/ttyACM0`.

The wrapper script passes the device into the container and joins the device's
host group, so no host permission changes (`uucp` group membership, udev rules)
are needed.

## Usage

```bash
./idf.sh build                # build
./idf.sh -p /dev/ttyACM0 flash
./idf.sh monitor              # Ctrl-] to exit
./idf.sh shell                # interactive shell with IDF exported
./idf.sh menuconfig
```

Override defaults via environment: `ESPPORT` (default `/dev/ttyACM0`),
`ESPBAUD` (default `460800`), `IDF_IMAGE` (default `espressif/idf:v6.1`).

## Tests

```bash
make test                     # no board, no Docker, no BLE stack
```

That generates the decoders from the Field Table, builds the decoding in
`main/wfdecode/` and the estimation in `main/wfest/` - both pure C99, the same
files the firmware compiles - with plain gcc, replays every recorded Capture in
`tests/fixtures/` through them and asserts on what comes out: every Controller
frame's checksum, the BMS register decode against the values that ride is known
to have produced, the Remaining Energy, distance and Range curves the estimator
draws from it, and a
handful of invariants any Capture has to satisfy. Each Capture is replayed
twice and the two curves have to match bit for bit, which is what makes the
estimator's determinism a test rather than an intention. Each is then replayed
twice more with its Odometer rewritten to a synthesised ramp, started either
side of the u16 wrap, and the two distance curves have to match bit for bit as
well - no Capture we hold crosses the wrap, and a wrap has to be a non-event.
It then decodes the
same Captures with
the generated Python decoder and asserts the two languages produce identical
numbers for every field of every record, which is what keeps ADR-0002 honest.
It also checks that the fixtures can still be rebuilt byte for byte from the
console dumps they came from, and that `docs/field-table.md` is still what
the Field Table generates.

Adding a Capture to the suite is a `.wfl` and a `.expect` file dropped into
`tests/fixtures/`; the runner discovers them and needs no change. See
`tests/fixtures/README.md`. `make fixtures` rebuilds the `.wfl` files from the
dumps in `captures/` via `scripts/dump2wfl.py`.

`make fit` refits Consumption against speed across that same archive and
rewrites `main/wfest/wf_fit.h`, the constants the firmware compiles in. The
samples come out of the real estimator - `./build-host/replay --samples`
walks every Capture through `main/wfest` and prints differences of its own
totals, so there is no second energy-and-distance integrator anywhere - and
`scripts/fit_consumption.py` does nothing but the least-squares regression of
`Wh/km = a + c*v^2`. It refuses rather than inventing coefficients when the
archive cannot support a fit, which is what it does today.

## What the firmware does

Stage one of the project: a BLE central that finds the two devices on the
motorbike, connects to them and pulls out everything a later, permanent
connection will need. It is deliberately protocol agnostic - it dumps rather
than decodes - and every command is driven from the console, so the host does
the thinking and the ESP32 only does the radio work.

Targets:

| Device | Module | Address | Known GATT |
| --- | --- | --- | --- |
| Fardriver GL96530_24_A_H994 motor controller | YuanQuFOC274 | rotates, scan by name | service `0xffe0`, data characteristic `0xffec` (notify + write-no-response), unsolicited 16-byte frames |
| Daly Smart BMS | CH10250DD0097 | `40:18:03:01:20:e9`, random | service `0xfff0`, notify `0xfff1`, control `0xfff2`, Modbus request/response |

The Fardriver changes its BLE address between sessions - two scans minutes
apart saw `c0:0b:f8:74:00:a5` and `c0:01:e1:ae:00:3d` - so reconnect logic has
to match on the advertised name or on service `0xffe0` rather than on the MAC.
The Daly address is stable but advertises as *random*, not public, so a
`connect` by MAC has to say so.

## The Fardriver link, as measured

Confirmed against a 709-frame capture with the ignition on and the bike
stationary. The controller needs no request at all: subscribing to the CCCD is
the whole protocol.

| | |
| --- | --- |
| Data characteristic | `0xffec`, value handle `0x0010`, CCCD `0x0011` |
| Properties | `0x16` - read, write-without-response, notify |
| Frame | exactly 16 bytes, `aa <type> <12 payload> <crc_lo> <crc_hi>` |
| Rate | ~35.5 frames/s, one type per frame |
| Type byte | `0x80`..`0xb6`, all 55 in ascending order, so a full sweep every ~1.55 s |
| CRC | Modbus CRC-16 (poly `0xa001`), init **`0x7f3c`**, over bytes 0..13, appended little endian |

Running the same CRC across all 16 bytes leaves a residue of 0, which is the
cheapest way to validate a frame.

Of the 55 types, only eight carry data that moves while parked: `0x80`,
`0x87`, `0x8e`, `0x95`, `0x9c`, `0xa3`, `0xaa`, `0xb0` - every seventh type,
all sharing the payload prefix `0118 0000 0000 0000`. Within these, only
payload bytes 8..9 and 10..11 (frame bytes `10..11` and `12..13`) change;
parked they idle at -1/-2 and 0/0xffff and remain unassigned. The other 47
types looked byte-identical for the whole parked capture, but "parked" is the
catch: these eight are the `motion` group, their payload byte 0 is gear and
the motion flags, and their bytes 6..7 carry `cur_rpm`, which reads 0 while
stationary and so is indistinguishable from a genuinely static type in a
parked capture alone. Two of the other 47 are eight-type blocks in exactly the
same way: `0x81`+ is pack voltage and line current, and `0x82`+ is live and
still unidentified. See `docs/field-table.md` for the Field Table
(gear, rpm, brake, temps, odometer, and the BMS registers), the Confidence of
each entry, and what's still open. That document is generated from
`field-table.json`, as are both decoders - see `docs/adr/0002-*`. Confirming anything
new still needs a capture taken **while riding**, not at a standstill:

```bash
./wf.sh 'connect name Yuan' 'discover' 'sub all' --listen 120 --capture ride.log
```

The 20 KB record buffer holds ~853 frames, so `rec` fills in about 24 s at the
observed rate; longer sessions must stream with `--listen` instead.

## Driving it from the host

`./wf.sh` sends console commands over the serial link and collects the output
up to the next prompt, so a capture is scriptable:

```bash
./wf.sh info                          # chip, heap, BLE state
./wf.sh --reset 'scan 10' devs        # scan and list what answered
./wf.sh 'dev 0'                       # decode one device's advertising data
./wf.sh 'connect name DL' 'probe 30@60' 'rec dump@180'
./wf.sh --listen 60 --capture run.log # just stream, e.g. while riding
```

A command may carry its own timeout with `@seconds`; `--capture FILE` appends
the raw session to a file.

## Console commands

| Command | Effect |
| --- | --- |
| `scan [secs] [passive] [dedup]` | Scan; results accumulate until `scanclear` |
| `devs` / `dev <idx>` | List devices / decode one device's AD structures |
| `scanclear` | Empty the scan table |
| `connect <idx>\|<mac> [public\|random]\|name <substr>` | Connect, print parameters |
| `disconnect` / `conn` | Terminate / print connection state |
| `mtu` | Run an ATT MTU exchange |
| `params <min> <max> <lat> <to>` | Request new connection parameters |
| `sec` | Initiate pairing, to find out whether anything needs it |
| `discover` / `gatt` | Walk the GATT database / reprint the last walk |
| `read <handle>` / `readall` | Read one attribute / every readable one |
| `write <handle> <hex> [nrsp]` | Write an attribute |
| `sub all\|<chr>\|h <cccd> [value]` | Write CCCDs to subscribe |
| `nlog on\|off [handle]` | Live notification logging |
| `nstat [reset]` | Per-handle frame counts, rates, lengths, header byte sets |
| `rec start\|stop\|<secs>\|dump [max]\|clear\|stat` | Record notifications to RAM, then dump |
| `crc <hex> [init]` | Modbus CRC16; the Fardriver uses init `0x7f3c` |
| `daly probe\|read <start> <addr> <count>` | Send Daly request frames |
| `probe [secs]` | MTU, discover, readall, subscribe all, record, statistics |
| `info` `ping` `led` `btn` `hb` `help` | Board and console basics |

Notifications are recorded into a 20 KB RAM buffer rather than printed live,
because a fast stream overruns the 115200 baud console. `rec dump` prints the
capture afterwards, timestamped in microseconds.

Output is written as tagged single-line records (`DEV`, `SVC`, `CHR`, `DSC`,
`RD`, `NTF`, `REC`, `NSTAT`) so a capture can be parsed offline.

## Typical capture session

```bash
./wf.sh --reset 'scan 15'          # find the bike, note the indices
./wf.sh 'scan 15'                  # again: a second address under the same
                                   # name is the Fardriver rotating its MAC
./wf.sh 'connect name DL' 'probe 30@90'
./wf.sh 'rec dump@180'             # the raw frames
./wf.sh 'daly probe@40' 'rec dump@180'   # which Modbus variant answers, and
                                         # how wide a read it will serve
```

## Boot

The board blinks the red LED twice at boot as a visible sign that the freshly
flashed image is running.

## The button menu

A is the capture workflow - scan, start, stop - and short B is the backlight,
or a Marker while a Capture is recording. Holding B opens a menu instead of
going straight to readout mode as it used to: B cycles the entries, A picks
one, and holding B again, a short press of the power button, or ten seconds of
nothing at all closes it.

| Entry | What it does |
| --- | --- |
| `READOUT` | Wi-Fi access point, Captures over HTTP, back only by reboot |
| `UPDATE` | joins a hotspot it knows and says what release is on offer |

The menu will not open while a Capture is recording or connecting, and says so
on the panel: both modes need the radio the Capture is using. See ADR-0006.

## Firmware slots and rollback

The flash carries two app slots, `ota_0` and `ota_1`, of 1.875 MB each, and the
bootloader's rollback is on - so a firmware written over the air can be undone
without a cable. See ADR-0006 and `partitions.csv`.

An image written into the spare slot boots on probation and keeps its place
only once NVS has opened, the capture store has mounted, the display has drawn
a frame and sixty seconds have passed. Anything else - a crash loop, a hang, a
store that will not mount - and the next reset takes the bootloader back to the
slot that worked. No Controller or BMS link is required, deliberately: updates
happen with the bike switched off. `ota` on the console prints which slot is
running and how far the check has got, and `info` names the slot too.

Moving onto this table moved the capture store from `0x210000` to `0x3E0000`,
so it reformats on the first boot after the change: **pull every Capture off
the board before flashing it.** A partition table cannot be replaced over the
air, so that flash is the cable this whole arrangement exists to be the last
of.

## Update mode

`UPDATE` on the menu joins a hotspot and reads the manifest of the newest
release, so the panel says either `UP TO DATE` or the tag on offer, with the
address it was given underneath. Downloading and installing the image is issue
#28; nothing is written to flash by the check itself.

It shares readout mode's shape: Wi-Fi and NimBLE do not fit in this chip's RAM
together, so BLE goes down when it starts and the way back to capturing is a
reboot. A failure is left failed - it names which failure it was and hands the
panel back, and nothing retries on its own.

The networks it may join live in NVS and arrive from the console, never from
this repository, which is public:

| Command | Effect |
| --- | --- |
| `wifi add <ssid> [passphrase]` | Remember a network, up to four; quote anything with a space in it |
| `wifi list` | What is stored, passphrases by length only |
| `wifi del <ssid>` | Forget one |
| `ota` | Slot, rollback health, the pin, and the URL the next check reads |
| `ota pin <tag>` / `ota pin` | Read one release instead of `latest` / go back to `latest` |
| `ota check` | Run the check from the console, without the menu |

The passphrases sit in NVS unencrypted, deliberately: a phone hotspot key is
the right kind of secret to keep on a bike, and it can be rotated on the
phone. Trust for the fetch comes from the ESP-IDF certificate bundle and not
from a pinned certificate - GitHub rotates its certificates, and a pinned one
would eventually stop every update. Versions are compared for inequality and
never for order, which is what makes `ota pin <tag>` a way of backing a bad
release out. See ADR-0006.

## Publishing a release

Update mode reads a manifest published on a GitHub release of this repository,
and `scripts/release.sh` is what makes one:

```bash
git tag v0.2.0 && git push origin v0.2.0
./scripts/release.sh v0.2.0
```

It builds in the usual container, writes a four-field manifest - `version`,
`url`, `size`, `sha256` - and attaches both files to the release, so the
newest one is always at `releases/latest/download/manifest.json`. It refuses
to publish from a dirty tree, from a commit the tag does not sit on, or when
the image's own version is not the tag, because there is no CI between the
editor and the bike. `--dry-run` stops before publishing. See
`docs/release.md` and ADR-0006.

## Board pins (M5StickC PLUS2)

| Pin | Function |
| --- | --- |
| GPIO4 | power hold — must be driven high or the board switches off on battery |
| GPIO19 | red LED, active high |
| GPIO37 / GPIO39 / GPIO35 | button A / button B / power button, active low |
| GPIO38 | battery voltage (divider ratio 2.0) |
| GPIO2 | buzzer |
| GPIO0 / GPIO34 | PDM microphone WS / data |
| GPIO21 / GPIO22 | internal I2C (MPU6886 IMU, BM8563 RTC) |
| GPIO32 / GPIO33 | Port A external I2C (Grove) |
| GPIO15 / GPIO13 / GPIO14 / GPIO12 / GPIO5 / GPIO27 | LCD (ST7789v2) MOSI / SCLK / DC / RST / CS / backlight |

PSRAM is left disabled in `sdkconfig.defaults`; enable it when the application
actually needs it.
