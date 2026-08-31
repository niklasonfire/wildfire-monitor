---
status: accepted
---

# Firmware updates come from a public release over the rider's hotspot

The Monitor updates itself by joining a phone hotspot, reading a four-field
manifest published as an asset on a GitHub release of this repository,
downloading the image named there over HTTPS, and writing it into the
inactive one of two app slots. It is never silent and never automatic: the
rider opens a menu and chooses it, the same way readout mode is chosen
today.

The repository is public so that the Monitor carries no credential of any
kind. The release is built on the same development machine that already
builds the firmware, by a script, and not by CI.

## Why

**The cable is the wrong tool for a device on a handlebar.** Flashing means
finding the USB lead, unclipping the Monitor, and sitting at the desk. That
cost is paid every time a decode is wrong or a constant moves, which on this
project is often. A hotspot and two button presses is the difference between
a fix that gets tried on the next ride and a fix that waits for a free
evening.

**Wi-Fi is already there and Bluetooth is not.** `webdump` brings Wi-Fi up
today, and NimBLE is built central-only because the Monitor's whole job is
to listen to the Controller and the BMS. Receiving an image over Bluetooth
would mean adding the peripheral role, a GATT service, and a chunking
protocol, and would spend RAM on a radio that must be down anyway - Wi-Fi
and NimBLE do not fit in this chip's RAM together, which is why readout mode
is entered by a reboot. Update mode inherits that shape for nothing. Over
HTTPS, the redirects, the streaming and the resume are already written.

**A manifest is a contract; an API answer is somebody else's.** The GitHub
release API returns a large JSON document that would have to be parsed on
the device, is rate-limited, and can change under us. Four fields -
`version`, `url`, `size`, `sha256` - published at
`releases/latest/download/manifest.json` are a stable redirect, cost no API
call, and are ours to keep still. What the device does not have to parse it
cannot fail to parse on a bike.

**A secret on the bike is not a secret.** A private repository would force a
GitHub token into the Monitor's flash, where anyone with a USB lead can read
it back; NVS is not encrypted here. Making the repository public removes the
secret rather than guarding it. It is also honest about what this project
is - a firmware whose value is the Field Table and the estimator, neither of
which is harmed by being read.

**Trust comes from the certificate bundle, not from a pinned certificate.**
GitHub rotates its certificates. A pinned one would eventually stop every
update, and the only way back would be the cable this decision exists to
avoid.

**The rollback is the point, not a refinement.** The Monitor is mounted on a
vehicle. A firmware that will not boot is a Monitor that has to come off the
bike. So the bootloader's rollback is enabled and a new image must earn its
place: NVS opens, the Capture store mounts, the display draws one frame, and
sixty seconds pass. Deliberately, no Controller or BMS link is required -
updates happen with the bike switched off, and a healthy firmware that
rolled itself back for want of a Pack would be worse than no rollback at
all.

**The release is built where the firmware is built.** This is ADR-0005's
argument one level out: a thing produced by a machine you can inspect, from
a tree you can name, beats a thing produced somewhere else. The repository
has no CI, and standing one up would mean pinning an ESP-IDF version in a
container and keeping it matched to the desktop - a second toolchain to
disagree with the first. `scripts/release.sh` builds and calls
`gh release create`, and refuses to publish from a dirty tree or from a
commit the tag does not sit on. Without those two guards the local build
ships whatever was in the editor.

## Consequences

**The Capture store shrinks and moves, and that costs one last cable.** The
partition table gains `otadata` and a second app slot:

    nvs,        data, nvs,     0x9000,   0x6000,
    otadata,    data, ota,     0xF000,   0x2000,
    phy_init,   data, phy,     0x11000,  0x1000,
    ota_0,      app,  ota_0,   0x20000,  0x1E0000,
    ota_1,      app,  ota_1,   0x200000, 0x1E0000,
    storage,    data, fat,     0x3E0000, 0x420000,

A partition table cannot be replaced over the air, so the change is
installed over USB once. The store moves from `0x210000` to `0x3E0000`, so
its old contents are unreachable and it reformats: **every Capture must be
off the board before that flash.** The store falls from 5.94 MB to 4.1 MB,
about ninety minutes of both links at 780 B/s. That is sized above the
forty minutes a ride is expected to need, because when the store fills it
is the tail that is lost, and the tail is the low State of Charge part that
the Consumption and Usable Capacity work needs most.

**The app has 1.875 MB and today uses 1.16 MB.** The certificate bundle and
mbedtls come out of that headroom. An app that ever outgrows a slot needs
another partition table, which is another cable, so the margin is a number
to watch and not a number to spend.

**Every Capture committed from here on is published.** Going public required
stripping two passive BLE scans out of the history: 41 third-party MAC
addresses and five named household devices, including two non-rotating
public MACs, which together fingerprint where those Captures were taken.
ADR-0001 already excludes them - a scan of bystanders' devices is not the
Controller or the BMS talking, so it was never Capture content - but the
rule now has a consequence outside the repository. The bike's own
identifiers stay: the BMS advertises its MAC and serial over the air to
anyone standing near the bike, and the docs are worth less without them.

**A release is a tag on a clean tree.** The app's version comes from
`git describe`, and the Monitor compares it to the manifest's `version` for
inequality, not for ordering. Inequality removes all version-comparison code
from the device and makes going back to an older build the same operation as
going forward - which, with a console `ota pin <tag>`, is how a suspect
release is backed out without publishing anything.

**Wi-Fi credentials are a list from the start.** NVS namespace `wifi` holds
up to four networks, and update mode scans and joins the strongest one it
knows. Only `wifi add`, `wifi list` and `wifi del` on the console write to
it today, but a second phone must not be a data-model change, and a
provisioning screen later should add a way in, not a second store. The
passphrases sit unencrypted in NVS; a phone hotspot key is the right thing
to have there, and it can be rotated on the phone.

**Long-pressing B now opens a menu.** Readout mode costs one more keypress
than it did, which buys a place to put update mode and whatever follows it
without inventing a third gesture on a two-button device. Update mode
refuses while a Capture is running, exactly as `app_readout_enter()`
already does.

**A failed download is left failed.** The half-written slot is harmless
because `otadata` still points at the running app, so a broken transfer
shows an error and returns to the menu. It does not retry by itself: the
rider is standing next to the Monitor during an update, and an automatic
retry would only hide a hotspot that is too weak to finish.
