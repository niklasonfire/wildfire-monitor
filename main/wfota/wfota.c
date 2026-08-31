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

/* ------------------------------------------------------------ the digest */

/* FIPS 180-4, the first thirty-two bits of the fractional parts of the cube
 * roots of the first sixty-four primes. A table, not a computation: it is
 * checkable against the standard by eye, which is the only review this code
 * will ever get. */
static const uint32_t K_SHA256[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u,
    0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
    0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
    0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
    0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
    0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
    0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u,
    0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
    0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

static uint32_t ror32(uint32_t v, unsigned n)
{
    return (v >> n) | (v << (32u - n));
}

static void sha256_block(uint32_t h[8], const uint8_t *p)
{
    uint32_t w[64];
    uint32_t a, b, c, d, e, f, g, hh;
    int      i;

    for (i = 0; i < 16; i++) {
        w[i] = ((uint32_t)p[i * 4 + 0] << 24) | ((uint32_t)p[i * 4 + 1] << 16) |
               ((uint32_t)p[i * 4 + 2] << 8)  | ((uint32_t)p[i * 4 + 3]);
    }
    for (; i < 64; i++) {
        uint32_t s0 = ror32(w[i - 15], 7) ^ ror32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ror32(w[i - 2], 17) ^ ror32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    a = h[0]; b = h[1]; c = h[2]; d = h[3];
    e = h[4]; f = h[5]; g = h[6]; hh = h[7];

    for (i = 0; i < 64; i++) {
        uint32_t S1  = ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25);
        uint32_t ch  = (e & f) ^ (~e & g);
        uint32_t t1  = hh + S1 + ch + K_SHA256[i] + w[i];
        uint32_t S0  = ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t t2  = S0 + maj;

        hh = g; g = f; f = e; e = d + t1;
        d  = c; c = b; b = a; a = t1 + t2;
    }

    h[0] += a; h[1] += b; h[2] += c; h[3] += d;
    h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

void wfota_sha256_init(wfota_sha256_t *s)
{
    if (s == NULL) {
        return;
    }
    /* The fractional parts of the square roots of the first eight primes. */
    s->h[0] = 0x6a09e667u; s->h[1] = 0xbb67ae85u;
    s->h[2] = 0x3c6ef372u; s->h[3] = 0xa54ff53au;
    s->h[4] = 0x510e527fu; s->h[5] = 0x9b05688cu;
    s->h[6] = 0x1f83d9abu; s->h[7] = 0x5be0cd19u;
    s->bytes = 0;
    s->fill  = 0;
    memset(s->block, 0, sizeof(s->block));
}

void wfota_sha256_feed(wfota_sha256_t *s, const void *data, size_t len)
{
    const uint8_t *p = data;

    if (s == NULL || (p == NULL && len != 0)) {
        return;
    }
    s->bytes += (uint64_t)len;

    /* Fill whatever is left of the held block first, then take whole blocks
     * straight out of the caller's buffer. An image arrives in pieces the
     * HTTP client sized, and none of them is a multiple of 64. */
    if (s->fill != 0) {
        size_t want = 64 - s->fill;
        size_t take = (len < want) ? len : want;

        memcpy(s->block + s->fill, p, take);
        s->fill += take;
        p       += take;
        len     -= take;
        if (s->fill < 64) {
            return;
        }
        sha256_block(s->h, s->block);
        s->fill = 0;
    }
    while (len >= 64) {
        sha256_block(s->h, p);
        p   += 64;
        len -= 64;
    }
    if (len != 0) {
        memcpy(s->block, p, len);
        s->fill = len;
    }
}

void wfota_sha256_hex(wfota_sha256_t *s, char *out, size_t cap)
{
    static const char hex[] = "0123456789abcdef";
    uint64_t bits;
    int      i;

    if (out == NULL || cap == 0) {
        return;
    }
    out[0] = '\0';
    if (s == NULL || cap < (size_t)WFOTA_SHA256_HEX + 1) {
        return;
    }

    bits = s->bytes * 8u;
    s->block[s->fill++] = 0x80;
    if (s->fill > 56) {
        memset(s->block + s->fill, 0, 64 - s->fill);
        sha256_block(s->h, s->block);
        s->fill = 0;
    }
    memset(s->block + s->fill, 0, 56 - s->fill);
    for (i = 0; i < 8; i++) {
        s->block[56 + i] = (uint8_t)(bits >> (56 - 8 * i));
    }
    sha256_block(s->h, s->block);
    s->fill = 0;

    for (i = 0; i < 8; i++) {
        int j;
        for (j = 0; j < 4; j++) {
            uint8_t byte = (uint8_t)(s->h[i] >> (24 - 8 * j));
            out[i * 8 + j * 2]     = hex[byte >> 4];
            out[i * 8 + j * 2 + 1] = hex[byte & 0x0f];
        }
    }
    out[WFOTA_SHA256_HEX] = '\0';
}

/* ---------------------------------------------------------- the download */

const char *wfota_dl_err_str(wfota_dl_err_t err)
{
    switch (err) {
    case WFOTA_DL_OK:        return "ok";
    case WFOTA_DL_ERR_LONG:  return "too long";
    case WFOTA_DL_ERR_SHORT: return "truncated";
    case WFOTA_DL_ERR_SHA256: return "bad digest";
    }
    return "unknown";
}

void wfota_dl_begin(wfota_dl_t *d, const wfota_manifest_t *m)
{
    if (d == NULL) {
        return;
    }
    memset(d, 0, sizeof(*d));
    d->shown = -1;
    wfota_sha256_init(&d->sha);
    if (m != NULL) {
        d->want = m->size;
        memcpy(d->want_sha, m->sha256, sizeof(d->want_sha));
        d->want_sha[WFOTA_SHA256_HEX] = '\0';
    }
}

wfota_dl_err_t wfota_dl_feed(wfota_dl_t *d, const void *data, size_t len)
{
    if (d == NULL) {
        return WFOTA_DL_ERR_LONG;
    }
    if (d->err != WFOTA_DL_OK) {
        return d->err;              /* left failed, and not un-failed */
    }
    /* Checked before a byte of this piece is hashed, so that the caller -
     * which writes to flash only on WFOTA_DL_OK - cannot be asked to put
     * anything past the end of the image into the slot. */
    if ((uint64_t)d->got + (uint64_t)len > (uint64_t)d->want) {
        d->err = WFOTA_DL_ERR_LONG;
        return d->err;
    }
    wfota_sha256_feed(&d->sha, data, len);
    d->got += (uint32_t)len;
    return WFOTA_DL_OK;
}

wfota_dl_err_t wfota_dl_end(wfota_dl_t *d)
{
    if (d == NULL) {
        return WFOTA_DL_ERR_SHORT;
    }
    if (d->err != WFOTA_DL_OK) {
        return d->err;
    }
    if (d->got != d->want) {
        /* The dropped-hotspot ending. It is a length, not a digest, because
         * a body that stopped early has no digest worth printing. */
        d->err = WFOTA_DL_ERR_SHORT;
        return d->err;
    }
    if (d->got_sha[0] == '\0') {
        wfota_sha256_hex(&d->sha, d->got_sha, sizeof(d->got_sha));
    }
    if (strcmp(d->got_sha, d->want_sha) != 0) {
        d->err = WFOTA_DL_ERR_SHA256;
        return d->err;
    }
    return WFOTA_DL_OK;
}

int wfota_dl_percent(const wfota_dl_t *d)
{
    if (d == NULL || d->want == 0) {
        return 0;
    }
    return (int)(((uint64_t)d->got * 100u) / (uint64_t)d->want);
}

int wfota_dl_step(wfota_dl_t *d)
{
    int pct;

    if (d == NULL) {
        return -1;
    }
    pct = wfota_dl_percent(d);
    if (pct == d->shown) {
        return -1;
    }
    d->shown = pct;
    return pct;
}
