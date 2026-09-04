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
disagree with the first. So a release script on the development machine builds
and calls `gh release create`, and refuses to publish from a dirty tree or
from a commit the tag does not sit on. Without those two guards the local
build ships whatever was in the editor. (There is CI now, and that script has
since been deleted; the amendment at the end of this document says what
changed and what did not.)

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

## Amendment: there is CI, and it publishes

The "no CI" half of the argument above no longer holds, and it is worth
saying exactly which half.

`.github/workflows/build.yml` builds the firmware on every push to `main`,
on every pull request, and on demand. It runs on a **self-hosted runner** -
a build machine outside GitHub, so the builds cost nothing and there is no
minute budget to ration them against.

**The objection it answered is gone rather than overruled.** The worry was a
second toolchain: a container pinned in CI, drifting away from the one on
the desktop, so that "it builds" stopped meaning "it builds the thing you
would build". The workflow runs `idf.py build` inside `espressif/idf:v6.1` -
the same image `./idf.sh` runs it in, named in the same place. The runner is
docker-in-docker and carries no toolchain of its own, so there is nothing on
it that can drift. Two callers, one image; when the image moves, both move.

**The trust boundary moved onto the runner, and the guard that matters
moved with it.** This paragraph used to say CI did not tag, did not write
a manifest and did not call `gh release create`. Since `9e51bdc` that is
false: `.github/workflows/release.yml` fires on a pushed `v*` tag, builds
in the same image, writes the same four fields, and publishes the release
with `softprops/action-gh-release@v2`. The check this ADR actually leans
on survived the move - the version is read back out of `esp_app_desc_t`
and the release refuses to go out unless it is the tag, so the image and
the manifest still cannot disagree. The two local guards had nothing to
move to: the workflow builds from a checkout of the tag itself, so there
is no dirty tree to refuse and no way for `HEAD` to be a different commit.
They were guards against a desk, and on the runner there is no desk -
which is why the script they lived in could be deleted without losing
anything. What did change is *which* machine - a build machine outside
GitHub rather than the one the firmware is written on. It runs the image
`./idf.sh` names, so "an image built from a tree you can name, by a
machine you can inspect" still holds; it is no longer the same machine,
which is a smaller move than it sounds and is still a move.

**The desktop publisher has been retired, and there is one release and one
publisher again.** For a while both existed, so a tag could be published
twice - once by the push and once from the desktop - and both wrote
`manifest.json` onto the same release, the second writer winning. That
happened on `v0.3.0`, and because the two did not even attach the same
image, which bytes landed on a bike was decided by a race: exactly the
kind of "the release names one thing and holds another" this decision
exists to prevent. The script is deleted. The argument that put it there -
a release built from a tree you can name, by a machine you can inspect -
is now carried by the workflow, which builds a checkout of the tag in the
image `./idf.sh` names. Publishing is `git tag vX.Y.Z && git push origin
vX.Y.Z` and nothing else; `docs/release.md` says what happens after that.

What CI buys is the thing local-only building never gave: a pull request
that will not compile says so before it is merged, rather than at the
moment somebody wants to cut a release from it. It also does not run the
host suite - `make test` is still the developer's to run, and nothing between
the editor and a published release runs it. That is a gap worth closing and it
is not closed here.

## Amendment: there is one Wi-Fi mode, and it falls back to the access point

The mode shape above no longer holds, and as with CI it is worth saying
exactly which half went.

**What went is the sentence at the top of this document.** "The rider opens a
menu and chooses it, the same way readout mode is chosen today" described two
modes that a rider had to choose between, and "Long-pressing B now opens a
menu" was justified by the need for somewhere to put the second of them.
There is one now. Entering it, the Monitor joins the strongest network it
knows and serves everything - the Captures, the settings page, the update -
over that link; failing to join anything, it puts up the same
`wildfire-xxxxxx` access point readout mode used to put up, serves the same
pages over that, and simply does not offer an update, because there is
nothing upstream of an access point to fetch one from. A Monitor that has
never been told a network takes the second path without scanning first: the
answer does not depend on what is in range, and a fresh Monitor should not
have to wait to be told what it already knows.

**Nothing else here moved.** The update still comes from a manifest published
on a release of this public repository and read over the rider's own hotspot.
The four fields are still the contract, the versions are still compared for
inequality, the certificate bundle is still the trust, the rollback is still
the point, the install is still confirmed on the device and never automatic,
and the credentials are still a list of up to four networks in NVS with
nothing in the repository carrying one. The RAM argument is untouched too:
NimBLE and Wi-Fi still do not fit together on this chip, so BLE still goes
down on the way in and the way back to capturing is still a reboot. What
changed is which link the pages are served over, and that a failure to join
is now a fallback rather than a dead end.

**The fallback is what keeps the cable from coming back, and that is the
whole reason for it.** The old arrangement had a hole in exactly the place
this decision was written to close. The list of networks lived in NVS; the
only ways to edit it were the console, over the USB lead, and a settings page
served by readout mode. Update mode could not reach that page - the two modes
were mutually exclusive one-way doors - so a rider whose hotspot key had been
rotated on the phone, or who had typed an SSID with a capital in the wrong
place, watched update mode fail and had no way to repair it from the mode it
failed in. Rebooting into readout mode was the answer as long as the rider
knew that was the answer. Now the failure lands on the page that fixes it,
which is the difference between a fault a rider clears in a car park and a
fault that costs the USB lead and an unclipped Monitor. Serial provisioning
must never be the only way in, and with a station-only mode it very nearly
was.

**Bootstrapping stopped needing a mechanism.** "A provisioning screen later
should add a way in, not a second store" is satisfied twice over: the
settings page writes through `wifi_store`, and it is now reachable on a
Monitor that has been flashed and never spoken to. There is no first-run
special case and no serial step to document - a Monitor with an empty list is
just the ordinary fallback, and the ordinary fallback serves the page.

**The consequence is that the door is heavier.** Readout mode was cheap: it
took BLE down and put an access point up, and that was all. The one mode
attempts a scan and a join first, which is a few seconds when a known hotspot
is there and twenty-odd when it is not, and only then falls back. A rider who
wants the Captures and knows there is no hotspot in range pays that wait. B
held while A picks the entry skips straight to the access point, which costs
one GPIO read and is worth having, but it is a shortcut and not the design:
the design is that doing nothing gets the same access point twenty seconds
later.

**`app_readout_enter()` is gone, and so is the pair of names.** The
"Consequences" section above names it as the thing update mode refuses like;
the refusal survives under `service_enter()`, and the glossary in `CONTEXT.md`
now carries **Service Mode** where it carried Readout Mode and Update Mode.
ADR-0008 leans on the old shape in passing - "readout mode and update mode are
mutually exclusive one-way doors, so a rider..." - and that sentence is now
history rather than fact. Its conclusion is not affected: a channel is still
read at the start of a mode and not while one is running, so a channel saved
on the settings page is still read at the next entry, and there is still no
"check now" button, because an install has to be confirmed on the device and
a button on a phone could only start something the rider would have to walk
back to the bike to answer.
