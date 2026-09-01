# Publishing a release

A release is what update mode installs. It is two files attached to a GitHub
release of this repository: the app image, and a manifest of four fields that
names it. `scripts/release.sh` produces both from the working tree and
publishes them, and ADR-0006 explains why the machine that builds the firmware
is also the machine that publishes it.

CI does not come into it. `.github/workflows/build.yml` builds every push and
every pull request on a self-hosted runner, in the same `espressif/idf:v6.1`
image `./idf.sh` uses, so a branch that will not compile says so before it is
merged - but it does not tag, does not write a manifest and does not publish.
Nothing a rider's Monitor installs has been through it.

## The short version

```bash
make test                          # the host suite, which the script does not run
git tag v0.2.0                     # on the commit you mean to release
git push origin main v0.2.0        # the tag has to be on GitHub first
./scripts/release.sh v0.2.0
```

Two minutes later:

```bash
curl -sL https://github.com/niklasonfire/wildfire_monitor/releases/latest/download/manifest.json
```

That URL is the one thing the Monitor knows. It is a redirect GitHub keeps
pointing at the newest release, so nothing on the device has to be told a
version number, and nothing here has to edit a file to publish one.

## What the script does

1. **Checks the tag exists.** The tag is the release's name and the image's
   version; there is nothing to build without one.
2. **Refuses a dirty tree.** Dirty in the sense `git describe` means it -
   modifications to tracked files. The version comes from `git describe`, so
   a tree it would call dirty is a tree whose image cannot be named honestly.
3. **Refuses a tag that is not on `HEAD`.** Otherwise the release carries the
   name of one commit and the code of another, which is the failure this
   whole arrangement exists to make impossible.
4. **Reconfigures and builds** in the usual container, `./idf.sh build`. The
   reconfigure is what makes the new tag reach the image: ESP-IDF runs
   `git describe` at CMake configure time only, and adding a tag to a commit
   that has already been built changes nothing CMake watches.
5. **Reads the version back out of the image** - out of `esp_app_desc_t`, 48
   bytes into `build/wildfire_monitor.bin` - and refuses unless it is the tag.
   This is what makes the image and the manifest incapable of disagreeing.
6. **Writes `build/manifest.json`**, four fields and no more.
7. **Refuses if the tag is not on `origin`.** `gh` would otherwise create a
   missing tag at the default branch's head, putting the release's name on a
   commit that is not the one just built. Pushing is left to you: it is the
   one irreversible step, and a script that publishes should not also rewrite
   the remote.
8. **Publishes** with `gh release create`, both files attached, neither draft
   nor prerelease - `/releases/latest/download/` skips both.

`--dry-run` does everything up to step 6 and then prints the manifest instead
of publishing it. It is the way to see what a release would say without
making one.

## The manifest

```json
{
  "version": "v0.1.0",
  "url": "https://github.com/niklasonfire/wildfire_monitor/releases/download/v0.1.0/wildfire_monitor.bin",
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

## When it refuses

Every refusal prints `release: ` and a reason, and nothing has been published
when it does. The awkward ones:

| Message | What happened |
| --- | --- |
| `the working tree is dirty` | commit or stash; the listing that follows is `git status --short` |
| `tag v… is on <sha> but HEAD is <sha>` | you tagged an older commit, or built on a newer one |
| `the image calls itself '<x>', not '<tag>'` | two tags sit on this commit and `git describe` picked the other one |
| `tag v… is not on origin yet` | `git push origin <tag>` and run it again |

## Backing a release out

Publish nothing. The Monitor compares versions for inequality, so pointing it
at an older tag is a downgrade it will happily perform, and the older
release's manifest is still at
`releases/download/<tag>/manifest.json`. Deleting the bad release on GitHub
also works: `latest` falls back to the one before it. Either way the bike
does not need a cable, which is the whole point.

If a released image is bad enough that it will not boot, nothing has to be
done at all: it never confirmed itself, and the bootloader has already put
the previous slot back. See the rollback section in the README.
