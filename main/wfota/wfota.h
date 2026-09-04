/*
 * wfota - the part of the update that has no radio in it.
 *
 * The update is three questions - which network do I join, what is the URL,
 * and is what came back a manifest - and none of the three needs Wi-Fi, TLS
 * or a board to answer. They live here so they can be driven off the bike,
 * where a malformed manifest is a string literal instead of a release nobody
 * wants to publish. main/ota_update.c is the half that owns the radio.
 *
 * Same rules as main/wfdecode and main/wfest: pure C99, no ESP-IDF, no
 * allocation. See ADR-0006 for why the manifest is four fields and why the
 * version is compared for inequality rather than for order.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* A git tag; ESP-IDF's own esp_app_desc_t::version is 32 bytes including the
 * NUL, so a version that does not fit here could not have been baked into an
 * image either. */
#define WFOTA_VERSION_MAX 31
#define WFOTA_URL_MAX     255
#define WFOTA_SHA256_HEX  64
/* 802.11 caps an SSID at 32 octets. They are octets and not text, so an SSID
 * carrying a NUL cannot be typed into the console and cannot be stored - a
 * limit worth writing down rather than discovering next to a hotspot. */
#define WFOTA_SSID_MAX    32

/* What the fetch is allowed to bring back. The manifest the release
 * workflow writes is about 250 bytes; a kilobyte is room for a fifth field
 * and for the pretty printing, and it is the point at which "this is not
 * our manifest" is a better answer than reading further. */
#define WFOTA_BODY_MAX    1024

/* An app slot from partitions.csv. A manifest naming an image that cannot fit
 * in one is wrong wherever it came from, so it is rejected here rather than
 * three minutes into a download. */
#define WFOTA_IMAGE_MAX   0x1E0000u

/* The one URL the Monitor knows, less the tail that says which release. It is
 * a literal rather than something read out of a build variable because the
 * device has no remote to ask; .github/workflows/release.yml builds the same
 * slug out of GITHUB_SERVER_URL and GITHUB_REPOSITORY when it writes the
 * manifest's own url field. */
#define WFOTA_RELEASES "https://github.com/niklasonfire/wildfire_monitor/releases"

/* A channel is the tail of that URL and nothing more. `stable` is the empty
 * tag - GitHub's `latest` redirect - so that an absent, unreadable or unknown
 * setting resolves to stable by construction rather than by a branch somebody
 * has to remember to write. Longest name in the table, plus room to type a
 * wrong one and be told so. */
#define WFOTA_CHANNEL_MAX    15
#define WFOTA_CHANNEL_STABLE "stable"

typedef struct {
    char     version[WFOTA_VERSION_MAX + 1];
    char     url[WFOTA_URL_MAX + 1];
    uint32_t size;
    char     sha256[WFOTA_SHA256_HEX + 1];
} wfota_manifest_t;

typedef enum {
    WFOTA_OK = 0,
    WFOTA_ERR_EMPTY,     /* nothing came back at all */
    WFOTA_ERR_BIG,       /* longer than WFOTA_BODY_MAX */
    WFOTA_ERR_JSON,      /* not the flat object the contract describes */
    WFOTA_ERR_FIELD,     /* a field is missing, repeated, or unusable */
} wfota_err_t;

/* Two words at most: this ends up on a 135x240 panel under an error title. */
const char *wfota_err_str(wfota_err_t err);

/*
 * Reads the four fields out of `body`, which need not be NUL terminated.
 *
 * Deliberately not a JSON parser: it accepts a flat object of string and
 * number values and refuses everything else - nesting, arrays, literals,
 * escapes. The manifest is ours, written by .github/workflows/release.yml
 * from a shell heredoc, and what the Monitor does not have to parse on a
 * handlebar it cannot fail to parse there. A key it does not know is
 * skipped rather than refused, so that publishing a fifth field one day
 * does not stop a Monitor running today's firmware from reading the four
 * it does know.
 */
wfota_err_t wfota_manifest_parse(const char *body, size_t len,
                                 wfota_manifest_t *out);

/*
 * Whether a console-typed tag may be pasted into a URL. Letters, digits, dot,
 * dash and underscore only, so a typo cannot introduce a path segment, a
 * query or an escape and quietly fetch something else.
 */
bool wfota_tag_ok(const char *tag);

/*
 * The release tag a channel names, or "" for stable, which is the `latest`
 * redirect. NULL when `name` is not one of the channels this firmware was
 * built with - which every caller reads as stable, because a channel with no
 * URL behind it must not be a channel a Monitor can end up on.
 */
const char *wfota_channel_tag(const char *name);

/*
 * Walks the channels this firmware knows: 0 is always stable, NULL past the
 * end. It is a walk rather than a count so that the settings page and the
 * console list what the build actually has, and not what they remember.
 */
const char *wfota_channel_name(int i);

/*
 * The manifest URL for the pinned tag; failing that for the channel; failing
 * that for `latest`. A pin outranks a channel because it is the deliberate
 * one-off - `ota pin <tag>` is how a single suspect release is looked at
 * without moving the bike off the stream it follows.
 *
 * An unknown `channel` is stable rather than a refusal: it is a value read
 * back out of flash, and the answer to not recognising it is the stream every
 * Monitor can always reach. A `pin` that wfota_tag_ok() refuses is still a
 * refusal, because that one was typed.
 *
 * False when the pin is unusable or the result would not fit, and `out` is
 * then left as an empty string.
 */
bool wfota_manifest_url(const char *channel, const char *pin,
                        char *out, size_t cap);

/* One access point the scan saw. */
typedef struct {
    char ssid[WFOTA_SSID_MAX + 1];
    int  rssi;                       /* dBm, so less negative is stronger */
} wfota_seen_t;

/*
 * Index into `seen` of the strongest access point whose SSID is one of
 * `known`, or -1 if none of them is. SSIDs are compared as whole octet
 * strings: a hotspot called "wildfire2" is not the "wildfire" we know, and
 * case is not folded because 802.11 does not fold it either. Ties keep the
 * earlier entry, which is the stronger one in a scan the driver has sorted.
 */
int wfota_pick_network(const char *const *known, int n_known,
                       const wfota_seen_t *seen, int n_seen);

/* ---- the download --------------------------------------------------------
 *
 * The fourth question, and the only one whose wrong answer cannot be taken
 * back from a handlebar: are the bytes that just went into the spare app slot
 * the image the manifest named. Answering it is arithmetic over a byte
 * stream - a running digest, a length, and the percentage the panel shows -
 * so it lives here, where a truncated body is a string literal instead of a
 * hotspot somebody has to walk out of range of.
 *
 * SHA-256 is written out here rather than called out of mbedtls, which the
 * TLS in ota_update.c links anyway. mbedtls does not build in the host
 * harness, and this digest is the one thing standing between a corrupted
 * download and a Monitor that has to come off the bike: it is worth the two
 * hundred lines to have `make test` drive the same accumulator the Monitor
 * runs, rather than a second one that agrees with it in principle.
 */

/* Enough of SHA-256 for an image arriving in whatever sized pieces the HTTP
 * client hands over. Opaque in use; the fields are here only because this
 * seam does not allocate. */
typedef struct {
    uint32_t h[8];
    uint64_t bytes;
    uint8_t  block[64];
    size_t   fill;
} wfota_sha256_t;

void wfota_sha256_init(wfota_sha256_t *s);
void wfota_sha256_feed(wfota_sha256_t *s, const void *data, size_t len);
/* Ends the digest and writes it as 64 lowercase hex characters and a NUL, so
 * `cap` must be at least WFOTA_SHA256_HEX + 1. The state is finished after
 * this and must be re-initialised before it is fed again. */
void wfota_sha256_hex(wfota_sha256_t *s, char *out, size_t cap);

typedef enum {
    WFOTA_DL_OK = 0,
    WFOTA_DL_ERR_LONG,     /* more bytes arrived than the manifest promised */
    WFOTA_DL_ERR_SHORT,    /* the body stopped early - a dropped hotspot */
    WFOTA_DL_ERR_SHA256,   /* the right number of bytes, and the wrong ones */
} wfota_dl_err_t;

/* Two words at most, for the same 135x240 panel. */
const char *wfota_dl_err_str(wfota_dl_err_t err);

typedef struct {
    uint32_t       want;         /* the manifest's size, in bytes */
    uint32_t       got;
    int            shown;        /* the last percentage handed to the screen */
    wfota_sha256_t sha;
    char           want_sha[WFOTA_SHA256_HEX + 1];
    char           got_sha[WFOTA_SHA256_HEX + 1];  /* filled in by _end() */
    wfota_dl_err_t err;          /* sticky: a failed download stays failed */
} wfota_dl_t;

/* Starts a download of the image `m` names. The manifest is copied from, not
 * held, so the caller may reuse it. */
void wfota_dl_begin(wfota_dl_t *d, const wfota_manifest_t *m);

/*
 * Takes the next piece of the body. The caller writes those bytes to flash
 * only when this returns WFOTA_DL_OK, which is what keeps a body longer than
 * the manifest promised from ever reaching the slot - the length is checked
 * before the write, not after it.
 */
wfota_dl_err_t wfota_dl_feed(wfota_dl_t *d, const void *data, size_t len);

/*
 * Closes the download: the body has to have been exactly as long as the
 * manifest said and to hash to what it said. Call it before anything switches
 * the boot partition, because after that the answer is a reboot away from
 * mattering. Idempotent, and it leaves the digest it computed in `got_sha`,
 * which is the thing worth putting in the log when it does not match.
 */
wfota_dl_err_t wfota_dl_end(wfota_dl_t *d);

/* How far along, 0..100. */
int wfota_dl_percent(const wfota_dl_t *d);
/*
 * The percentage to draw, or -1 when it has not moved since the last call.
 * A repaint costs an SPI frame and the body arrives in pieces far smaller
 * than a percent of it, so the screen is told once per whole number rather
 * than once per chunk.
 */
int wfota_dl_step(wfota_dl_t *d);
