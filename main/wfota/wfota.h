/*
 * wfota - the part of update mode that has no radio in it.
 *
 * Update mode is three questions - which network do I join, what is the URL,
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

/* What the fetch is allowed to bring back. The manifest release.sh writes is
 * about 250 bytes; a kilobyte is room for a fifth field and for the pretty
 * printing, and it is the point at which "this is not our manifest" is a
 * better answer than reading further. */
#define WFOTA_BODY_MAX    1024

/* An app slot from partitions.csv. A manifest naming an image that cannot fit
 * in one is wrong wherever it came from, so it is rejected here rather than
 * three minutes into a download. */
#define WFOTA_IMAGE_MAX   0x1E0000u

/* The one URL the Monitor knows, less the tail that says which release. It is
 * a literal rather than something read out of a build variable because the
 * device has no remote to ask; scripts/release.sh derives the same slug from
 * `git remote get-url origin` when it writes the manifest's own url field. */
#define WFOTA_RELEASES "https://github.com/niklasonfire/wildfire_monitor/releases"

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
 * escapes. The manifest is ours, written by scripts/release.sh from a shell
 * heredoc, and what the Monitor does not have to parse on a handlebar it
 * cannot fail to parse there. A key it does not know is skipped rather than
 * refused, so that publishing a fifth field one day does not stop a Monitor
 * running today's firmware from reading the four it does know.
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
 * The manifest URL for the pinned tag, or for `latest` when `pin` is NULL or
 * empty. False when the tag is not one wfota_tag_ok() accepts or the result
 * would not fit, and `out` is then left as an empty string.
 */
bool wfota_manifest_url(const char *pin, char *out, size_t cap);

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
