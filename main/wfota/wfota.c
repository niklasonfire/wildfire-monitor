/* wfota - see wfota.h. Pure C99: no radio, no allocation, no ESP-IDF. */
#include "wfota.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------- scanning */

static const char *skip_ws(const char *p, const char *end)
{
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) {
        p++;
    }
    return p;
}

/*
 * One JSON string, from its opening quote to just past its closing one, or
 * NULL if what is there is not one. `out` may be NULL to skip the value
 * without keeping it; when it is not, `ovf` is raised if the value did not
 * fit, which is a different complaint from a body that is malformed.
 */
static const char *scan_string(const char *p, const char *end,
                               char *out, size_t cap, bool *ovf)
{
    size_t w = 0;

    if (p >= end || *p != '"') {
        return NULL;
    }
    for (p++; p < end && *p != '"'; p++) {
        unsigned char c = (unsigned char)*p;

        /* No escapes and no control characters. None of the four fields - a
         * git tag, an https URL, lowercase hex - can hold a character that
         * would have to be escaped, so a backslash here means the body is
         * not the file scripts/release.sh writes, and guessing at what it is
         * instead is exactly the work ADR-0006 keeps off the handlebar. */
        if (c == '\\' || c < 0x20) {
            return NULL;
        }
        if (out != NULL) {
            if (w + 1 < cap) {
                out[w++] = (char)c;
            } else {
                *ovf = true;
            }
        }
    }
    if (p >= end) {
        return NULL;                /* unterminated */
    }
    if (out != NULL) {
        out[w] = '\0';
    }
    return p + 1;
}

/*
 * One unsigned decimal number. No sign, no fraction and no exponent: `size`
 * is a count of bytes, and a manifest that writes it any other way is not
 * one we wrote.
 */
static const char *scan_uint(const char *p, const char *end,
                             uint32_t *out, bool *ovf)
{
    uint64_t v = 0;
    int digits = 0;

    for (; p < end && *p >= '0' && *p <= '9'; p++, digits++) {
        v = v * 10u + (uint64_t)(*p - '0');
        if (v > 0xffffffffu) {
            v = 0xffffffffu;
            *ovf = true;
        }
    }
    if (digits == 0) {
        return NULL;
    }
    *out = (uint32_t)v;
    return p;
}

/* A value belonging to a key we do not know: a string or a number, and
 * nothing else. An object or an array here would need a parser with a stack,
 * which is precisely what a four-field contract exists to avoid. */
static const char *skip_value(const char *p, const char *end)
{
    uint32_t ignored = 0;
    bool     ovf = false;

    if (p < end && *p == '"') {
        return scan_string(p, end, NULL, 0, NULL);
    }
    return scan_uint(p, end, &ignored, &ovf);
}

/* ------------------------------------------------------------- the fields */

#define F_VERSION (1u << 0)
#define F_URL     (1u << 1)
#define F_SIZE    (1u << 2)
#define F_SHA256  (1u << 3)
#define F_ALL     (F_VERSION | F_URL | F_SIZE | F_SHA256)

static bool is_lower_hex(const char *s, size_t want)
{
    size_t i;

    for (i = 0; i < want; i++) {
        char c = s[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {
            return false;
        }
    }
    return s[want] == '\0';
}

/* Every field checked for the one thing that would make it useless later:
 * a version that could not name a release, a url the download half would
 * refuse to open, a size no app slot could hold, a digest that is not one. */
static bool fields_usable(const wfota_manifest_t *m)
{
    if (!wfota_tag_ok(m->version)) {
        return false;
    }
    /* https only, and checked here rather than in the download: an image
     * fetched over plain http is trusted on the strength of the manifest
     * that named it, and the manifest itself came over TLS. */
    if (strncmp(m->url, "https://", 8) != 0 || m->url[8] == '\0') {
        return false;
    }
    if (m->size == 0 || m->size > WFOTA_IMAGE_MAX) {
        return false;
    }
    return is_lower_hex(m->sha256, WFOTA_SHA256_HEX);
}

wfota_err_t wfota_manifest_parse(const char *body, size_t len,
                                 wfota_manifest_t *out)
{
    const char *p, *end, *q;
    unsigned    seen = 0;

    if (out == NULL) {
        return WFOTA_ERR_FIELD;
    }
    memset(out, 0, sizeof(*out));

    if (body == NULL || len == 0) {
        return WFOTA_ERR_EMPTY;
    }
    if (len > WFOTA_BODY_MAX) {
        return WFOTA_ERR_BIG;
    }

    p   = skip_ws(body, body + len);
    end = body + len;

    if (p >= end || *p != '{') {
        return WFOTA_ERR_JSON;
    }
    p = skip_ws(p + 1, end);
    if (p < end && *p == '}') {
        p++;                        /* {} - well formed, and missing all four */
    } else {
        while (true) {
            /* 16 is longer than every key we know, so a key that overflows
             * it is by construction one we do not - and cannot be truncated
             * into one that we do. */
            char     key[16];
            bool     ovf = false;
            unsigned bit = 0;

            p = skip_ws(p, end);
            q = scan_string(p, end, key, sizeof(key), &ovf);
            if (q == NULL) {
                return WFOTA_ERR_JSON;
            }
            p = skip_ws(q, end);
            if (p >= end || *p != ':') {
                return WFOTA_ERR_JSON;
            }
            p = skip_ws(p + 1, end);

            ovf = false;
            if (strcmp(key, "version") == 0) {
                bit = F_VERSION;
                q = scan_string(p, end, out->version, sizeof(out->version), &ovf);
            } else if (strcmp(key, "url") == 0) {
                bit = F_URL;
                q = scan_string(p, end, out->url, sizeof(out->url), &ovf);
            } else if (strcmp(key, "sha256") == 0) {
                bit = F_SHA256;
                q = scan_string(p, end, out->sha256, sizeof(out->sha256), &ovf);
            } else if (strcmp(key, "size") == 0) {
                bit = F_SIZE;
                q = scan_uint(p, end, &out->size, &ovf);
            } else {
                q = skip_value(p, end);
            }
            if (q == NULL) {
                return WFOTA_ERR_JSON;
            }
            if (ovf) {
                return WFOTA_ERR_FIELD;
            }
            if (bit != 0) {
                /* The same key twice has no answer. Taking the last one would
                 * make what the Monitor installs depend on the order of a file
                 * nobody reads that far into. */
                if ((seen & bit) != 0) {
                    return WFOTA_ERR_FIELD;
                }
                seen |= bit;
            }

            p = skip_ws(q, end);
            if (p < end && *p == ',') {
                p++;
                continue;
            }
            if (p < end && *p == '}') {
                p++;
                break;
            }
            return WFOTA_ERR_JSON;
        }
    }

    /* Trailing whitespace is the newline the heredoc leaves; anything else
     * after the object means this is a document with a manifest in it, which
     * is not the same thing as a manifest. */
    if (skip_ws(p, end) != end) {
        return WFOTA_ERR_JSON;
    }
    if ((seen & F_ALL) != F_ALL) {
        return WFOTA_ERR_FIELD;
    }
    return fields_usable(out) ? WFOTA_OK : WFOTA_ERR_FIELD;
}

const char *wfota_err_str(wfota_err_t err)
{
    switch (err) {
    case WFOTA_OK:        return "ok";
    case WFOTA_ERR_EMPTY: return "empty";
    case WFOTA_ERR_BIG:   return "too big";
    case WFOTA_ERR_JSON:  return "bad json";
    case WFOTA_ERR_FIELD: return "bad field";
    }
    return "unknown";
}

/* ----------------------------------------------------------------- the URL */

bool wfota_tag_ok(const char *tag)
{
    size_t n;

    if (tag == NULL || tag[0] == '\0') {
        return false;
    }
    for (n = 0; tag[n] != '\0'; n++) {
        char c = tag[n];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '_')) {
            return false;
        }
        if (n >= WFOTA_VERSION_MAX) {
            return false;
        }
    }
    return true;
}

bool wfota_manifest_url(const char *pin, char *out, size_t cap)
{
    int n;

    if (out == NULL || cap == 0) {
        return false;
    }
    out[0] = '\0';

    if (pin == NULL || pin[0] == '\0') {
        /* The redirect GitHub keeps pointing at the newest release, which is
         * the whole reason nothing on the device carries a version number. */
        n = snprintf(out, cap, "%s/latest/download/manifest.json", WFOTA_RELEASES);
    } else if (!wfota_tag_ok(pin)) {
        return false;
    } else {
        n = snprintf(out, cap, "%s/download/%s/manifest.json", WFOTA_RELEASES, pin);
    }
    if (n < 0 || (size_t)n >= cap) {
        out[0] = '\0';
        return false;
    }
    return true;
}

/* ------------------------------------------------------------- the network */

int wfota_pick_network(const char *const *known, int n_known,
                       const wfota_seen_t *seen, int n_seen)
{
    int best = -1;

    if (known == NULL || seen == NULL) {
        return -1;
    }
    for (int i = 0; i < n_seen; i++) {
        for (int k = 0; k < n_known; k++) {
            if (known[k] == NULL || known[k][0] == '\0') {
                continue;
            }
            if (strcmp(known[k], seen[i].ssid) != 0) {
                continue;
            }
            if (best < 0 || seen[i].rssi > seen[best].rssi) {
                best = i;
            }
            break;
        }
    }
    return best;
}
