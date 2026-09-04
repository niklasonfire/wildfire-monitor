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

`scripts/gps.py` reads a GPS Track recorded alongside a ride, measures the
Monitor's clock error against it by cross-correlation, and finds the ride's
Manoeuvres - rests, steady holds, full-throttle launches, rollouts, regen
decelerations - from the speed curve and the Capture's own throttle and
current, so a test ride no longer depends on the rider pressing a button at
speed. A Track is the only instrument this project has that stands outside it,
and it is why two Field Table entries are `proven`; it settles speed and
distance against the ground and nothing else. A rollout is the exception, and
measures the bike's drag without going through the current scale at all. See
`docs/gps.md` and ADR-0007.

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

## What the panel shows while recording

Measurements, not counters. The band under the red `REC` banner holds the line
current at the largest type the panel fits, then Pack voltage, road speed, the
BMS's own State of Charge and engine temperature; the elapsed time sits in the
banner beside the word, and the frame counts, drops, file name and free space
are a three-line footer at the bottom.

The reason is that a climbing frame count says the radio is up and says nothing
about whether the bytes underneath decode - a Controller sending a layout the
Field Table does not describe would run that counter at exactly the same rate.
A current that moves with the throttle and a speed that matches the road say
the decode is alive, which is the question worth answering while a ride is
still recoverable.

Every one of those is a Field Table entry formatted and nothing else. Nothing
on this screen is divided by anything - no Range, no Consumption, no Remaining
Energy - because those rest on an assumed Limp Point and on a Consumption curve
this build has no fit for. They are the right thing to ride by and the wrong
thing to judge a Capture by, and they are one short press of the power button
away on the live screen.

A value is drawn only while the link carrying it is still delivering: the
decoder latches every field it has ever seen, so a Controller that goes off the
air would otherwise leave its last current on the panel looking exactly like
the current flowing now. Two seconds of silence - seventy missing frames at the
Controller's 35.5 Hz - and the row becomes a dash. The same gate applies to the
live screen.

## The button menu

A is the capture workflow - scan, start, stop - and short B is a Marker while
a Capture is recording, and nothing otherwise. It used to toggle the backlight
and no longer does: an unlit panel was one stray press away, and the same
press means Marker as soon as a Capture starts. The backlight is `disp on|off`
on the console. Holding B opens a menu instead of going straight to Wi-Fi as
it used to: B cycles the entries, A picks one, and holding B again, a short
press of the power button, or ten seconds of nothing at all closes it.

| Entry | What it does |
| --- | --- |
| `SERVICE` | Wi-Fi: joins a network it knows or puts up its own, serves the Captures, the settings and the update, back only by reboot |
| `INFO` | the running firmware version and app slot; any key goes back |

`INFO` is the one entry that takes nothing down - no radio, no BLE shutdown,
and no refusal while a Capture is recording - so the version can be read on
the bike without a cable. A `git describe` version is wider than the panel, so
it is wrapped at a `-` rather than clipped, which is what keeps a `-dirty` on
screen.

The menu will not open while a Capture is recording or connecting, and says so
on the panel: `SERVICE` needs the radio the Capture is using.

Hold B to open the menu, keep holding it, and press A: `SERVICE` is already
the selected entry, and picking it with B still down skips the scan and goes
straight to the access point. That is worth knowing in a car park with no
hotspot in range and is not otherwise required - let go of B first and the
same access point arrives twenty seconds later on its own. Letting go and
holding B again does not do it: a second hold of eight hundred milliseconds is
the gesture that closes the menu. See ADR-0006.

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
running, how far the check has got and whether the last update was rolled
back, and `info` names the slot too.

Moving onto this table moved the capture store from `0x210000` to `0x3E0000`,
so it reformats on the first boot after the change: **pull every Capture off
the board before flashing it.** A partition table cannot be replaced over the
air, so that flash is the cable this whole arrangement exists to be the last
of.

## Service Mode

`SERVICE` on the menu is the only Wi-Fi mode there is, and it is a station
first. It tries to join the strongest of the networks it knows and serves
everything over that link; failing to join anything at all - nothing stored,
nothing in range, nothing known among what was in range, a passphrase the
access point refused - it puts up its own access point instead and serves the
same pages over that. The panel says which of the two it landed on, the
address to type into a phone, and either the access point's password or the
signal strength of the network it joined.

| Landed on | Panel shows | What is served |
| --- | --- | --- |
| a network it knows | the SSID, `http://<address>`, the signal in dBm | Captures, settings, and the update |
| its own access point | `wildfire-xxxxxx`, `pw wildfire`, `http://192.168.4.1` | Captures and settings |

The fallback is the point of the arrangement, not a nicety. The settings page
is where the list of networks is edited without a cable, so it has to be
reachable from the failure that list caused: a hotspot key rotated on the
phone, or an SSID typed with a capital in the wrong place, would otherwise
cost the USB lead and an unclipped Monitor. It is also the whole of the
bootstrap - a Monitor that has never been told a network puts its access point
up without scanning first, the rider adds their hotspot on the page that is
already there, and every entry after that joins it.

BLE goes down before the radio comes up, because Wi-Fi and NimBLE do not fit
in this chip's RAM together, so the way back to capturing is a reboot - which
is what `A` does from the mode's screen. The mode is refused while a Capture
is running.

### The update

The update is on the settings page, and the mode does not go looking for one
on its way in. **Check for updates** reads the manifest of the newest release
on the channel selected just above it, over the link the mode came up on, and
prints the answer on the page: up to date, naming the version running, or a
tag on offer. A version on offer gets a second button beside the first, and
pressing that installs it. Two presses, and nothing at all if nobody presses -
see ADR-0006. Over the access point there is no upstream, so the page says why
instead of offering a button that could only fail.

Because the check runs on demand, the channel is read at the moment it runs:
pick a channel and press check, and that is the channel that was read. The
Monitor's screen carries the check while it happens and the percentage while
an image comes down, but the answer is on the phone that asked for it.

Neither button does the work on the server's own task. The install stops the
server for the length of the transfer - TLS, the image buffer and the flash
write want the heap the httpd is holding - so the install page is sent in full
before that happens, and it says to watch the Monitor's screen rather than the
browser.

What the download checks, in this order, is what makes the reboot safe:

* the slot written is the inactive one, never the running one;
* every piece of the body is length-checked before it is written, so a server
  sending more than the manifest said cannot put a byte of it into the slot;
* the SHA-256 is complete and compared to the manifest **before** the boot
  partition is switched, so an image that does not match never becomes
  bootable;
* the new image then comes up on probation and has to pass the health check
  above, or the next reset takes the previous firmware back.

A rollback is not silent: the firmware that comes back up says `ROLLED BACK` on
the panel and `ota` prints which slot was abandoned. A failure is left failed -
it names which failure it was, on the panel and on the page, and nothing
retries on its own - but the link and the server both come back underneath the
message, so a cut download leaves the settings page in reach and the second
attempt is a button on the page rather than a walk back to the bike.

### The networks it may join

They live in NVS and arrive either from the console or from the settings page,
never from this repository, which is public:

| Command | Effect |
| --- | --- |
| `wifi on` / `wifi off` | Enter Service Mode / take it back down |
| `wifi add <ssid> [passphrase]` | Remember a network, up to four; quote anything with a space in it |
| `wifi list` | What is stored, passphrases by length only |
| `wifi del <ssid>` | Forget one |
| `ota` | Slot, rollback health, the channel, the pin, and the URL the next check reads |
| `ota channel [<name>]` | Read the `debug` channel instead of `stable` / bare goes back to `stable` |
| `ota pin <tag>` / `ota pin` | Read one release instead of the channel's newest / go back to it |
| `ota check` | Read the manifest, bringing the mode up first if it is not |
| `ota install` | The same, and install what the check found - how this is exercised on the bench |

`wifi on` stops at the mode, and so does `SERVICE` on the menu: neither goes
looking for an update, because the button that does is on the page. `ota
check` asks the same question through the same worker the page's button uses,
so the console and the phone cannot be shown two different manifests and only
one thing at a time is doing TLS. Both `ota` subcommands refuse when the mode
fell back to the access point: there is no upstream behind it, and the honest
answer to "check" is that there was nothing to ask.

### The settings page

`http://<address>/settings`, linked from the capture listing, is `wifi
add|list|del` without a PC: it shows what is stored, adds a network and
forgets one. Typing a stored network's name again replaces its passphrase,
which is the rotation a phone hotspot eventually needs. It never shows a
passphrase back, only counts its characters - so what reaching the page buys
is the ability to change the list, not to read it. Over the access point the
shared key is the whole of what stands in front of it, and over a joined
network so is that network.

The page picks the update channel too: `stable`, which is the newest published
release and where a Monitor is unless somebody has said otherwise, or `debug`,
a build meant for one bike and invisible to every other. It shows the URL the
choice resolves to, which is what makes *which channel am I on* answerable
without a cable. The channel is read when the check runs, so one saved here
takes effect on the next press rather than at the next entry into the mode.
Nothing that can be stored there, and no rollback, can leave a Monitor unable
to get back to `stable`: see ADR-0008.

It also carries the update, which means reaching this page is now the whole of
the authority to install one. That is bounded rather than guarded: the image
is named by a manifest fetched over TLS from a public GitHub release and
checked by length and SHA-256 before the bootloader is pointed at it, so what
somebody else on the network gains is the ability to force a *published*
build, forward or back, and not to run one of their own. Every form on this
server refuses a submission that came from another origin, so that stays "on
the network and on this page" rather than becoming "any page the rider's
phone happens to open". ADR-0006's amendment says what that costs and why
nothing here asks for a password.

The passphrases sit in NVS unencrypted, deliberately: a phone hotspot key is
the right kind of secret to keep on a bike, and it can be rotated on the
phone. Trust for the fetch comes from the ESP-IDF certificate bundle and not
from a pinned certificate - GitHub rotates its certificates, and a pinned one
would eventually stop every update. Versions are compared for inequality and
never for order, which is what makes `ota pin <tag>` a way of backing a bad
release out. See ADR-0006.

## Publishing a release

The Monitor reads a manifest published on a GitHub release of this repository,
and pushing a version tag is what makes one:

```bash
make test                          # the host suite, which CI does not run
git tag v0.2.0                     # on the commit you mean to release
git push origin v0.2.0             # this is the publish
```

`.github/workflows/release.yml` fires on any `v*` tag. It builds in the same
`espressif/idf:v6.1` image `./idf.sh` uses, writes a four-field manifest -
`version`, `url`, `size`, `sha256` - and attaches it to the release beside the
app image, a merged image, the bootloader and the partition table, so the
newest manifest is always at `releases/latest/download/manifest.json`. It
refuses to publish when the version baked into the image is not the tag, which
is what stops the release naming one build and holding another. A tag with a
dash in it is a prerelease, invisible to every unpinned Monitor, and moves the
`debug` channel instead; see ADR-0008. There is no local publish path. See
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
