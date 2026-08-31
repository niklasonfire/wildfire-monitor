#!/usr/bin/env bash
# Build the firmware and publish it as the GitHub release the Monitor reads.
#
#   ./scripts/release.sh v0.2.0             # build, write the manifest, publish
#   ./scripts/release.sh --dry-run v0.2.0   # everything except the publish
#
# The release carries two assets: the app image, and a manifest of exactly four
# fields - version, url, size, sha256 - that update mode fetches from
#
#   https://github.com/<owner>/<repo>/releases/latest/download/manifest.json
#
# ADR-0006: the release API would hand the Monitor a large JSON document to
# parse on a bike, is rate-limited, and can change under us. Four fields at a
# stable redirect are ours to keep still.
#
# This script is the trust boundary, because there is no CI: the machine that
# builds the firmware is the machine that publishes it. So it refuses to
# publish from a dirty tree, and refuses when the tag does not sit on the
# commit that was built. Without those two guards a local build ships whatever
# happened to be in the editor, under a name that says otherwise.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="build/wildfire_monitor.bin"
MANIFEST="build/manifest.json"

DRY_RUN=0
TAG=""

die() { echo "release: $*" >&2; exit 1; }

usage() {
    echo "usage: $0 [--dry-run] <tag>" >&2
    exit 2
}

while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run) DRY_RUN=1 ;;
        -h|--help) usage ;;
        -*)        usage ;;
        *)         if [ -n "$TAG" ]; then usage; fi
                   TAG="$1" ;;
    esac
    shift
done
[ -n "$TAG" ] || usage

cd "$PROJECT_DIR"

# ---- the guards ----------------------------------------------------------
#
# All three run before the build, so a refusal costs seconds rather than the
# two minutes a full build takes.

command -v gh >/dev/null || die "gh is not installed; it is what publishes the release"

git rev-parse -q --verify "refs/tags/$TAG" >/dev/null \
    || die "no tag $TAG in this repository: tag the commit you mean to release first"

# Dirty in the sense git describe means it, which is the sense that matters
# here: the version baked into the image comes from git describe, and it is
# the tracked files it compares. update-index --refresh first, or a file whose
# timestamp moved but whose content did not reads as a modification.
git update-index -q --refresh
git diff-index --quiet HEAD -- \
    || die "the working tree is dirty; commit or stash before releasing:
$(git status --short)"

HEAD_SHA="$(git rev-parse HEAD)"
TAG_SHA="$(git rev-parse "$TAG^{commit}")"
[ "$TAG_SHA" = "$HEAD_SHA" ] \
    || die "tag $TAG is on $TAG_SHA but HEAD is $HEAD_SHA; check out the tag you mean to release"

# The manifest's url has to name this repository, and it is the remote that
# knows which one that is. Both forms it can take end in <owner>/<repo>[.git],
# so take the last two path components of either and drop the suffix - which
# also survives an ssh host alias such as git@github-me:owner/repo.git.
REMOTE_URL="$(git remote get-url origin 2>/dev/null)" \
    || die "no 'origin' remote, so there is nowhere to publish to"
SLUG="$(echo "${REMOTE_URL%.git}" | sed -E 's#^.*[:/]([^:/]+/[^:/]+)$#\1#')"
case "$SLUG" in
    */*) ;;
    *)   die "cannot read <owner>/<repo> out of the origin remote: $REMOTE_URL" ;;
esac

# ---- the build -----------------------------------------------------------
#
# The same container the firmware is always built in, so the image published
# is the image a plain ./idf.sh build produces. Nothing is cleaned first: the
# tree is known clean, so an incremental build cannot differ from a full one,
# and a release that took three minutes is a release that gets made.
#
# The reconfigure is not optional, though. ESP-IDF runs git describe once, at
# CMake configure time, and bakes the answer into the image; a new tag on a
# commit that has already been built changes nothing CMake watches, so the
# next `idf.sh build` is a no-op and the image keeps calling itself by the
# commit hash it was built under. Reconfiguring re-runs the describe, which is
# the normal case here: tagging what you just built is exactly how a release
# is made.
echo "release: building $TAG"
./idf.sh reconfigure
./idf.sh build

[ -f "$BIN" ] || die "$BIN was not produced by the build"

# What the image actually says its version is, read back out of the image
# rather than recomputed. ESP-IDF puts esp_app_desc_t immediately after the
# 24-byte image header and the 8-byte segment header, and its version[32]
# field sits 16 bytes into that: 32 + 16 = 48. Reading it here is what makes
# the manifest and the image incapable of disagreeing - the whole reason the
# version is a field in the manifest at all.
BAKED="$(dd if="$BIN" bs=1 skip=48 count=32 2>/dev/null | tr -d '\0')"
[ "$BAKED" = "$TAG" ] || die "the image calls itself '$BAKED', not '$TAG'.
git describe is what bakes that in and it was just re-run, so this means it
picked another name for this commit - a second tag sitting on it, most
likely. Delete the one you do not mean to release."

SIZE="$(stat -c %s "$BIN")"
SHA256="$(sha256sum "$BIN" | cut -d' ' -f1)"
URL="https://github.com/$SLUG/releases/download/$TAG/wildfire_monitor.bin"

# Four fields and no more. The device parses this on a handlebar, so every
# field here is one it must handle and one that can be malformed; anything a
# human wants to know is on the release page instead. Written into build/,
# which is gitignored: it is a build artefact of this tag, not a file the
# repository carries.
cat > "$MANIFEST" <<EOF
{
  "version": "$TAG",
  "url": "$URL",
  "size": $SIZE,
  "sha256": "$SHA256"
}
EOF

echo "release: manifest for $TAG"
cat "$MANIFEST"

# ---- the publish ---------------------------------------------------------

if [ "$DRY_RUN" = "1" ]; then
    echo "release: --dry-run, not publishing. To publish:"
    echo "    git push origin $TAG"
    echo "    $0 $TAG"
    exit 0
fi

# gh creates a missing tag on the remote at the target branch's head, which
# would put the release's name on a commit that is not the one just built -
# exactly what the tag-on-HEAD guard exists to prevent, one machine further
# out. So the tag has to be on the remote already, at this commit. Pushing it
# is left to the caller: it is the one irreversible step in here, and a script
# that publishes should not also be a script that rewrites the remote.
#
# Compared as objects rather than as commits: whatever refs/tags/$TAG names
# here - the tag object of an annotated tag, the commit of a lightweight one -
# origin has to name the same thing. The local tag was already checked to sit
# on HEAD, so one equality settles both, and nothing has to peel a tag object
# over the network.
REMOTE_TAG_OBJ="$(git ls-remote origin "refs/tags/$TAG" | cut -f1)"
[ -n "$REMOTE_TAG_OBJ" ] \
    || die "tag $TAG is not on origin yet; push it first:
    git push origin $TAG"
[ "$REMOTE_TAG_OBJ" = "$(git rev-parse "$TAG")" ] \
    || die "origin's $TAG is a different tag object than this one; it was moved
after it was pushed, and the release would not name this build"

# Not a draft and not a prerelease: /releases/latest/download/ resolves to the
# newest full release, and skips both. A draft release would publish a
# manifest the Monitor can never see.
echo "release: publishing $TAG to $SLUG"
gh release create "$TAG" "$BIN" "$MANIFEST" \
    --title "$TAG" \
    --notes "Firmware $TAG for the Wildfire Monitor.

Update mode reads \`manifest.json\`; \`wildfire_monitor.bin\` is the image it
names. sha256 \`$SHA256\`, $SIZE bytes.

Installed from the Monitor's B menu, or over USB with \`./idf.sh flash\`."

echo "release: published"
echo "    https://github.com/$SLUG/releases/latest/download/manifest.json"
