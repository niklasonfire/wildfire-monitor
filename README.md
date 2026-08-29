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
catch: type `0xb0`'s payload bytes 6..7 carry `cur_rpm`, which reads 0 while
stationary and so is indistinguishable from a genuinely static type in a
parked capture alone. See `docs/fardriver-fields.md` for the decoded field
table (gear, rpm, brake, temps, odometer), sourced from a public firmware
project for the same bike, and for what's still open. Confirming anything
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
./wf.sh 'daly probe@30' 'rec dump@180'   # which Modbus variant answers
```

## Boot

The board blinks the red LED twice at boot as a visible sign that the freshly
flashed image is running.

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
