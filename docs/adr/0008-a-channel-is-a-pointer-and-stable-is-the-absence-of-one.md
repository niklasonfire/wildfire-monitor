---
status: accepted
---

# A channel is a pointer release, and `stable` is the absence of one

The Monitor reads one of two channels. `stable` reads
`releases/latest/download/manifest.json`, which is the one URL it has always
known. `debug` reads `releases/download/channel-debug/manifest.json` — a fixed
release that is not a build and holds nothing but a copy of the newest debug
release's manifest. A debug build is still published as an ordinary immutable
release of its own, tagged `vX.Y.Z-debug.N`.

The choice lives in NVS namespace `ota` under key `chan`, and `stable` is
stored as nothing at all: an absent, unreadable or unrecognised value **is**
stable, by construction rather than by a branch somebody has to remember to
write. `ota pin <tag>` still outranks the channel.

## Why

**One stream makes every release a release for every bike.** The BMS
cell-imbalance work is the case that forced this: a build worth having on the
bench and on one bike, and no business at all on a rider's. Published as a
normal release it goes to everyone; left unpublished it can only be installed
over USB, which is the cable ADR-0006 exists to be the last of. A second
channel is what gives that build somewhere to go.

**GitHub has no "latest prerelease" URL.** `/releases/latest/download/`
resolves to the newest release with `make_latest` set, and the API will not
mark a prerelease as latest — asked for both it drops the prerelease flag
instead, which is how `v0.2.1-rc2` once became a full release and took
`latest` with it. There is no second redirect to ask for. So a device that
knows exactly one URL per channel needs a tag that never moves while its
contents do, and the only thing GitHub offers with that shape is a release
that is not a build. The alternative is teaching the Monitor to list releases
and pick the newest prerelease itself, which is the large JSON document
ADR-0006 refused to parse on a bike, plus a rate limit, plus a version
*ordering* comparison the device deliberately does not have.

**A manifest committed to a branch would separate the manifest from the image
it names.** ADR-0006 made the manifest a release asset because it is written in
the same act that publishes the binary: the size and the sha256 are read back
out of the image that was just built, and both files are attached together, so
neither can exist without the other. Put the manifest on a branch and the two
drift apart — a commit naming an image nobody uploaded, or an image whose
manifest is one push behind — and it arrives from a second host,
`raw.githubusercontent.com`, which is one more name that has to be reachable
from a hotspot at the side of a road. The pointer release keeps the manifest
exactly what it was: an asset on a release. It only makes one release's asset a
byte-identical copy of another's.

**The debug image keeps its own immutable release, and the pointer holds only
paper.** If `channel-debug` carried the binary, every debug build would
overwrite the previous one's bytes under a URL that older manifests still
name — the same address serving different firmware on different days, so a
manifest fetched last week would install something nobody could work out
afterwards, and `ota pin` would have nothing immutable left to point at.
Instead the copied manifest's `url` field still names an asset on
`v0.4.0-debug.1`'s own release, exactly as the manifest on that release does:
the pointer moves and nothing it points at ever does. `release.yml` already
marks any tag containing a dash `prerelease: true, make_latest: false`, so the
versioned debug release is invisible to every unpinned Monitor the moment it is
published, and the pointer is the only thing that reveals it.

**`stable` is an absence because absence is the one value that cannot be
corrupted into something else.** An NVS erase, a bit flip in a stored string,
or firmware that predates the key all read back as nothing stored, and nothing
stored is stable. Were `stable` a word in flash, those same failures would read
back as a name this build does not recognise, and the device would need a rule
for what to do about it — a rule that is right until the day it is not, and the
day it is not is a Monitor coming off a handlebar. `otaup_pin_get()` already
treats an unreadable pin as no pin; the channel copies that convention exactly,
validating the name on the way out of flash as well as on the way in.

## Why this cannot strand a Monitor off `stable`

1. `stable` is the absence of a key, so an NVS erase, a corrupt entry, or a
   firmware that has never heard of `chan` all land on stable.
2. A name that is not in the compiled-in table is refused on write and ignored
   on read, so a channel with no URL behind it can never be stored.
3. Every build carries the whole table with `stable` first — a debug image
   knows how to go back, and the settings page can offer it.
4. A rolled-back image reverts the channel to stable by itself. It is one-shot,
   keyed on the `trial` channel written the instant the boot partition is
   switched, so re-selecting `debug` afterwards sticks.
5. The settings page needs only a phone and the console `ota channel` needs
   only a cable, so no single broken surface closes both doors; `ota pin <tag>`
   still overrides either for a one-off.
6. There is no anti-rollback and no ordering comparison, so installing stable
   over debug is an ordinary install and not a downgrade anything has to be
   talked into.

## Consequences

**There is a release in the list that is not a build.** `channel-debug` sits on
the releases page permanently, carrying one file, dated whenever the last debug
build went out, under a tag no commit is named after. Anybody reading that list
has to know it is furniture. Its notes say so in the first line, and it is
marked prerelease so it can never compete for `latest`, but this is a real cost:
the releases page stopped being a list of builds.

**Publishing by hand has one more step that can be forgotten.** A debug tag
published without the pointer being updated leaves `channel-debug` naming an
older build. Nothing is stranded — versions are compared for inequality, so
every Monitor on debug simply keeps being offered the tag the pointer names —
but the update that was just cut reaches nobody, and the panel gives no hint
that the pointer is the reason. `release.yml` does it as a step conditioned on
the dash in the tag, so a pushed tag cannot forget it; the failure is only
there when a release, or a rollback of one, is assembled by hand.

**A channel says where a build came from, not how it was compiled.** A debug
release is the same build configuration as a stable one — a different tag, not
`-Og` and asserts. `sdkconfig.defaults` builds for size and pushes Wi-Fi's and
NimBLE's ISRs out of IRAM because the two together overflow the 128 KB segment,
and the app is already well over half of its 1.875 MB slot, so a genuinely
debug-compiled variant is a separate piece of work that this decision neither
does nor blocks.

**The channel takes effect at the next update, not at the moment it is set.**
Readout mode and update mode are mutually exclusive one-way doors, so a rider
who picks a channel on the settings page has to reboot and choose `UPDATE`
before anything about it is visible. The page and the console both say so,
because a setting that appears to do nothing is a setting that gets pressed
twice.

**`CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK` must stay off.** It is the obvious next
hardening and it would break exactly the property this decision rests on: going
back to stable from a debug build is frequently a lower version number, and a
bootloader that refuses those is a bootloader that strands a Monitor on debug.
ADR-0006 already chose inequality over ordering on the device; this extends the
same rule down into the bootloader configuration.

## What this does not change

ADR-0006 stands. The Monitor still knows one URL at a time, still parses four
fields and nothing else, still carries no credential, and still installs only
what a rider chose from the menu. A channel changes which release that one URL
resolves to; it does not add a second protocol, a second host, or a second
thing on the bike that can hold a secret.
