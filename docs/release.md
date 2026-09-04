# Publishing a release

A release is what the Monitor installs over the air. It is two files attached to a GitHub
release of this repository: the app image, and a manifest of four fields that
names it. Pushing a version tag is what produces both, and ADR-0006 explains
why a release is a public GitHub release at all - its amendment says why the
machine that builds the firmware is now a build machine and not this desk.

**The tag push is the release.** There is nothing to run afterwards and nothing
to run instead:

```bash
make test                          # the host suite, which nothing in CI runs
git tag v0.2.0                     # on the commit you mean to release
git push origin v0.2.0             # this is the publish: the tag starts release.yml
```

`.github/workflows/release.yml` fires on a pushed `v*` tag, builds in the same
`espressif/idf:v6.1` image `./idf.sh` uses, writes the four-field manifest -
checking the tag against the version baked into the image - and publishes the
release with `softprops/action-gh-release@v2`. There is one publisher. A
desktop script used to be a second one, publishing the same release off this
machine; two publishers writing the same `manifest.json` onto the same release
is a race, and the last writer won - which happened on `v0.3.0`. It was deleted
rather than repaired, because everything it guarded against the runner rules
out by building a fresh checkout of the tag.

`.github/workflows/build.yml` is the workflow that still publishes nothing: it
builds every push to `main` and every pull request, so a branch that will not
compile says so before it is merged, and stops there.

Two minutes later:

```bash
curl -sL https://github.com/niklasonfire/wildfire_monitor/releases/latest/download/manifest.json
```

That URL is the one thing the Monitor knows. It is a redirect GitHub keeps
pointing at the newest release, so nothing on the device has to be told a
version number, and nothing here has to edit a file to publish one.

## What the workflow does

1. **Checks out the tag with its full history.** `fetch-depth: 0`, because
   ESP-IDF names the image from `git describe` and a shallow clone has no tag
   to describe it with.
2. **Builds in the ESP-IDF container**, `idf.py set-target esp32` then
   `idf.py build`, inside `espressif/idf:v6.1` - the image `./idf.sh` names,
   driven by `docker run` because the runner is docker-in-docker and carries
   no toolchain of its own. Then `idf.py merge-bin` for the single-file image.
3. **Collects the assets**: `wildfire_monitor-<tag>-app-ota.bin`, the app-only
   image the Monitor installs; `wildfire_monitor-<tag>-merged.bin` for flashing
   a blank board in one write; `bootloader-<tag>.bin` and
   `partition-table-<tag>.bin` for flashing at the esp32 offsets.
4. **Writes `manifest.json`**, four fields and no more, from a heredoc - and
   refuses rather than writing a manifest it cannot stand behind; see *When it
   refuses*.
5. **Publishes** with `softprops/action-gh-release@v2`, every collected file
   attached, notes generated from the commits. Prerelease exactly when the tag
   has a dash in it, with `make_latest` the inverse of that - both flags, never
   one, because the API cannot mark a prerelease as latest and asked for both
   it drops the prerelease flag instead.
6. **Moves the debug channel**, but only for a dashed tag: the same action runs
   again against the fixed `channel-debug` release and uploads a copy of the
   manifest published above. See *Channels*.

The build runs on the self-hosted runner, so nothing about the tree on your
desk reaches it: it builds the tag, from a fresh checkout, in the pinned image.
What it does not do is run `make test` - that is still the developer's, and CI
runs no test suite at all.

## When it refuses

Every refusal prints `release: ` and a reason. All of them come from the
manifest step, which runs before anything is published, so nothing exists on
the releases page when one fires:

| Message | What happened |
| --- | --- |
| `release/wildfire_monitor-<tag>-app-ota.bin was not produced by the build` | the build did not leave the app image where the collect step put it |
| `tag '<tag>' holds a character the Monitor will not accept` | the tag is outside letters, digits, dot, dash and underscore, so `wfota_tag_ok()` on the device would refuse it |
| `tag '<tag>' is longer than the 31 characters an image can carry` | `esp_app_desc_t::version` is 32 bytes with a terminator |
| `the image calls itself '<x>', not '<tag>'` | two tags sit on this commit and `git describe` picked the other one |

The last one is the check the whole arrangement leans on: the version is read
back out of the image at byte offset 48 rather than recomputed, so the image
and the manifest cannot disagree. A failure earlier than that - the build
itself - also publishes nothing, because the publish step never runs.

## The manifest

```json
{
  "version": "v0.1.0",
  "url": "https://github.com/niklasonfire/wildfire_monitor/releases/download/v0.1.0/wildfire_monitor-v0.1.0-app-ota.bin",
  "size": 1169600,
  "sha256": "3056b696feaaed678d650caed5877c712817504d819666cb699d77957b039213"
}
```

Four fields, because every field is one the Monitor must parse on a bike and
one that can be malformed. `version` is compared for inequality, never for
ordering - see ADR-0006 - so going back to an older build is the same
operation as going forward. `url` names that release's own image, not
`latest`, so an old manifest keeps naming the image it was published with.
`size` and `sha256` are what the download is checked against before anything
is written to a slot.

Anything a human wants to know goes in the release notes instead. The
manifest is not documentation.

## Channels

A Monitor reads one channel, and there are two:

| Channel | The URL the device knows | Where it points |
| --- | --- | --- |
| `stable` | `releases/latest/download/manifest.json` | the newest full release, by GitHub's redirect |
| `debug` | `releases/download/channel-debug/manifest.json` | a fixed pointer release, moved by the workflow |

**The dash in the tag is the whole mechanism.** A debug release is tagged
`vX.Y.Z-debug.N` - the version it is based on, then a counter for that version,
`v0.4.0-debug.1`, `v0.4.0-debug.2`. The workflow reads that dash and marks the
release `prerelease` with `make_latest` off, and `/releases/latest/download/`
skips prereleases, so a dashed tag is invisible to every unpinned Monitor on
stable the moment it exists. There is no second list to keep in agreement: the
tag says which channel it belongs to and nothing else does.

**The pointer release is not a build.** `channel-debug` is created once and
then only uploaded to. It carries exactly one file, a byte-identical copy of
the newest debug release's `manifest.json`, and that copy's `url` field still
names the image on `v0.4.0-debug.1`'s own release. So the channel moves while
nothing it names ever does: `ota pin v0.4.0-debug.1` keeps resolving to the same
bytes, and a manifest a Monitor fetched last week still names the image it
shipped with. Nothing is ever released under the `channel-debug` tag, and no
commit is named after it. It must never be `latest`, which is why the workflow
passes `prerelease: true` and `make_latest: false` on every upload to it and
not only on the first - a pointer release that ever became latest would serve a
debug manifest to every bike on stable. ADR-0008 says why it has to exist at
all: GitHub has no "latest prerelease" download URL.

### Cutting one

```bash
make test                                   # the host suite, which nothing else runs
git tag v0.4.0-debug.1                      # on the commit you mean to ship to your bike
git push origin v0.4.0-debug.1              # CI builds, publishes, and moves the pointer
```

Check both channels rather than one - the point of the exercise is that stable
did not move:

```bash
curl -sL https://github.com/niklasonfire/wildfire_monitor/releases/download/channel-debug/manifest.json
curl -sL https://github.com/niklasonfire/wildfire_monitor/releases/latest/download/manifest.json
```

The first names `v0.4.0-debug.1`; the second still names whatever full release
it named before. On the bike, `ota channel debug` then `ota check`, or the
channel selector on the settings page and a reboot into `UPDATE`.

### Backing one out

`latest` falls back when a release is deleted. **The pointer does not.** Delete
a debug release without moving the pointer and `channel-debug` serves a
manifest naming an asset that is gone, which the Monitor reports as a failed
download and hands the panel back. So move the pointer first, by copying an
older debug release's manifest onto it:

```bash
gh release download v0.4.0-debug.1 --pattern manifest.json --dir /tmp --clobber
gh release upload channel-debug /tmp/manifest.json --clobber
gh release delete v0.4.0-debug.2 --cleanup-tag      # optional, and only after
```

That is the one thing still done by hand, and it is a move of an existing
asset rather than a publish: no image is built and no new release is cut.

Usually nothing needs publishing at all. Versions are compared for inequality,
so a Monitor on debug installs the older manifest as an ordinary update; and
the shortest way out of a bad debug build is to leave the channel alone and put
the device back on stable - `ota channel` on the console, or the selector on
the settings page. `ota pin <tag>` overrides both for a one-off. A debug build
that will not boot needs nothing done to it by anyone: it never confirmed
itself, the bootloader has already put the previous slot back, and the rollback
puts the channel back to stable on the way.

Retiring the debug channel entirely means deleting the `channel-debug` release.
Monitors left on debug then fetch a 404 and say so; none of them is stranded,
because stable is one console command or one press on the settings page away.

## Backing a release out

Publish nothing. The Monitor compares versions for inequality, so pointing it
at an older tag is a downgrade it will happily perform, and the older
release's manifest is still at
`releases/download/<tag>/manifest.json`. Deleting the bad release on GitHub
also works, and still does: `latest` falls back to the one before it, so every
Monitor on stable is offered the previous release at its next check. On debug
it does not, because a pointer has no fallback - move `channel-debug` first,
as above. Either way the bike does not need a cable, which is the whole point.

If a released image is bad enough that it will not boot, nothing has to be
done at all: it never confirmed itself, and the bootloader has already put
the previous slot back. See the rollback section in the README.
