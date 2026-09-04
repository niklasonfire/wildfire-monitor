/*
 * webdump - the HTTP server behind Service Mode, and the access point that
 * mode falls back to.
 *
 * The console runs at 115200 baud, which is about four minutes per megabyte;
 * a ride is several megabytes. So instead of shipping the capture out over the
 * serial link the board serves the files over HTTP, where the same capture
 * takes seconds: connect a phone or a laptop, open the page, download.
 *
 * The same server carries the settings page, because a phone already joined
 * to pull a capture off is the one place on this bike a rider can type. What
 * it settles is the list of networks the Monitor may join, and the channel
 * the next update reads - ADR-0006 asked that a provisioning screen find its
 * way into wifi_store rather than start a second store, and this is that way
 * in. Passphrases go in and are never rendered back out; the page counts
 * their characters the way `wifi list` does on the console.
 *
 * Two halves, and the split is the point (#41). web_ap_start() puts up the
 * SoftAP - one netif, one DHCP server, one WPA2 key - and web_serve_start()
 * puts the httpd on top of whatever link is already there. That is what lets
 * the same handlers answer over a hotspot ota_update.c joined as a station,
 * so the settings page is reachable from the failure it exists to repair
 * rather than only from a mode entered instead of that one. Which link the
 * Monitor is on is main.c's decision; nothing below this line asks.
 *
 * On the access point there is no authentication beyond the pre-shared key,
 * and on a joined network there is none at all - anyone else on the rider's
 * hotspot can reach these pages. That is the same bargain either way: what is
 * on offer is a recording of a bike and a list of SSIDs, and the one secret
 * in reach, a stored passphrase, never leaves the board.
 *
 * BLE is already down when either half starts - the caller does that with
 * cap_ble_shutdown() - because NimBLE and Wi-Fi do not fit in RAM together on
 * this chip. That is also why the handlers keep almost nothing static and
 * borrow their buffers from the heap only for the length of a response.
 */
#include "webdump.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "capture_store.h"
#include "ota_update.h"
#include "wfota.h"
#include "wifi_store.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "web";

#define WEB_CHANNEL   6
#define WEB_MAX_CONN  2
#define WEB_FALLBACK_IP "192.168.4.1"
/* One read/send unit. 4 KB is a whole FAT cluster and a comfortable TCP write,
 * and it is heap allocated because the httpd task only has an 8 KB stack. */
#define WEB_CHUNK     4096
#define WEB_LIST_MAX  STORE_MAX_FILES
/* Length of "cap0001.wfl", the only file name shape the server accepts. */
#define WEB_NAME_LEN  11

/* The server, and separately the access point it may or may not be sitting
 * on. Two flags rather than one, because the mode can hold either without the
 * other: the link comes up first and goes down last, and over a joined
 * network there is no access point here at all. */
static httpd_handle_t s_server;
static esp_netif_t *s_netif;
static esp_event_handler_instance_t s_wifi_evt;
static bool s_wifi_inited;
static bool s_ap_running;

/* ------------------------------------------------------------------ utils */

/*
 * "/f/cap0007.wfl" -> 7, and nothing else gets through: exactly three letters,
 * four digits and ".wfl", with the query string cut off first. The request
 * text never reaches the filesystem - store_path() rebuilds the name from the
 * integer - so "/f/../../etc/passwd" cannot survive the round trip, it simply
 * fails the shape test and gets a 400.
 */
static bool parse_seq(const char *uri, const char *prefix, int *out_seq)
{
    size_t plen = strlen(prefix);
    if (strncmp(uri, prefix, plen) != 0) {
        return false;
    }
    const char *name = uri + plen;
    size_t len = strcspn(name, "?");
    if (len != WEB_NAME_LEN) {
        return false;
    }
    if (memcmp(name, "cap", 3) != 0 || memcmp(name + 7, ".wfl", 4) != 0) {
        return false;
    }
    int seq = 0;
    for (int i = 3; i < 7; i++) {
        if (name[i] < '0' || name[i] > '9') {
            return false;
        }
        seq = seq * 10 + (name[i] - '0');
    }
    *out_seq = seq;
    return true;
}

static void fmt_dur(uint32_t ms, char *out, size_t cap)
{
    uint32_t s = ms / 1000;
    snprintf(out, cap, "%02" PRIu32 ":%02" PRIu32, s / 60, s % 60);
}

/* The board has no time zone database and the header stores UTC seconds, so
 * the page says UTC and leaves the conversion to whoever reads it. */
static void fmt_time(int64_t unix_start, char *out, size_t cap)
{
    struct tm tm;
    time_t t = (time_t)unix_start;

    if (unix_start <= 0 || gmtime_r(&t, &tm) == NULL ||
        strftime(out, cap, "%Y-%m-%d %H:%M:%S UTC", &tm) == 0) {
        snprintf(out, cap, "unknown");
    }
}

/* httpd_err_code_t has no 503, so the status line goes out by hand. The string
 * literals passed to httpd_resp_set_status()/set_hdr() are not copied by the
 * server, which is fine for statics but matters for the stack buffers below. */
static esp_err_t send_unavailable(httpd_req_t *req)
{
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "capture store not mounted\n");
}

/* -------------------------------------------------------------- HTTP: shell */

/*
 * Both pages are the same shell with a different word in the title and a link
 * to the other one. The style block is inlined rather than served from a
 * second route: it is under a kilobyte, and a page that arrives in one
 * response cannot half-load on a phone that walked out of range.
 */
static const char k_head_a[] =
    "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>wildfire ";
static const char k_head_b[] =
    "</title><style>"
    "body{font-family:system-ui,sans-serif;margin:0;padding:16px;"
    "background:#111;color:#eee}"
    "h1{font-size:20px;margin:0 0 4px}"
    ".sub{color:#999;font-size:13px;margin:0 0 16px}"
    ".nav{margin:0 0 16px}"
    ".nav a{color:#6cf;font-size:15px;text-decoration:none}"
    ".c{border:1px solid #333;border-radius:8px;padding:12px;margin:0 0 12px}"
    "a.d{display:block;font-size:19px;color:#6cf;text-decoration:none;"
    "padding:8px 0}"
    /* An SSID is not a link, but it sits where one does on the other page,
     * and it can be 32 characters of anything - so it wraps rather than
     * pushing the forget button off the side of a phone. */
    ".n{font-size:19px;padding:8px 0;word-break:break-all}"
    ".m{color:#aaa;font-size:13px;margin:2px 0 10px}"
    ".msg{border:1px solid #444;border-radius:8px;background:#1b1b1b;"
    "padding:10px;margin:0 0 12px;font-size:14px}"
    "label{display:block;color:#aaa;font-size:13px;margin:0 0 12px}"
    /* 16px because anything smaller makes a phone zoom in when the field
     * takes focus and leave half the page off the side of the screen. */
    "input{display:block;box-sizing:border-box;width:100%;margin:4px 0 0;"
    "background:#1a1a1a;color:#eee;border:1px solid #444;border-radius:6px;"
    "padding:10px;font-size:16px}"
    /* A radio inherits the text input rule above otherwise, and a
     * full-width block per choice reads as a list of fields rather than
     * as one question with alternatives. */
    "input[type=radio]{display:inline-block;width:auto;margin:0 8px 0 0;"
    "vertical-align:middle}"
    "label.r{color:#eee;font-size:17px;margin:0 0 10px}"
    /* A manifest URL is ~90 characters with nowhere to break, so it is
     * told to break anywhere rather than push the page sideways. */
    ".u{color:#aaa;font-size:12px;margin:10px 0 0;word-break:break-all}"
    "button{background:#822;color:#fff;border:0;border-radius:6px;"
    "padding:12px 18px;font-size:15px}"
    /* The two destructive buttons are red; remembering a network is not. */
    "button.ok{background:#264}"
    ".e{color:#999}"
    "</style></head><body><h1>wildfire ";

static const char k_page_tail[] = "</body></html>";

static const char k_nav_settings[] =
    "<p class=\"nav\"><a href=\"/settings\">settings &rsaquo;</a></p>";
static const char k_nav_captures[] =
    "<p class=\"nav\"><a href=\"/\">&lsaquo; captures</a></p>";

/* The word after "wildfire" is both the tab title and the heading, so it goes
 * out twice; `nav` is the link to the page this one is not. */
static esp_err_t send_head(httpd_req_t *req, const char *title, const char *nav)
{
    esp_err_t err;

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    err = httpd_resp_sendstr_chunk(req, k_head_a);
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, title);
    }
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, k_head_b);
    }
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, title);
    }
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, "</h1>");
    }
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, nav);
    }
    return err;
}

/* ------------------------------------------------------------- HTTP: index */

static esp_err_t index_get(httpd_req_t *req)
{
    char sub[112], dur[24], when[40], name[24], row[448];
    uint64_t total = 0, freeb = 0;
    esp_err_t err = ESP_OK;

    if (!store_ready()) {
        return send_unavailable(req);
    }

    /* 64 store_entry_t are ~3.5 KB, far too much for the handler stack, and
     * Wi-Fi is holding most of the heap - so borrow it just for this page. */
    store_entry_t *ents = calloc(WEB_LIST_MAX, sizeof(*ents));
    if (ents == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_FAIL;
    }
    int count = store_list(ents, WEB_LIST_MAX);
    if (count < 0) {
        free(ents);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "store read failed");
        return ESP_FAIL;
    }

    if (store_space(&total, &freeb) == ESP_OK) {
        snprintf(sub, sizeof(sub),
                 "<p class=\"sub\">%d capture(s), %" PRIu64 " of %" PRIu64
                 " MB free</p>",
                 count, freeb / (1024 * 1024), total / (1024 * 1024));
    } else {
        snprintf(sub, sizeof(sub), "<p class=\"sub\">%d capture(s)</p>", count);
    }

    err = send_head(req, "captures", k_nav_settings);
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, sub);
    }
    if (err == ESP_OK && count == 0) {
        err = httpd_resp_sendstr_chunk(req, "<p class=\"e\">no captures on the board.</p>");
    }

    for (int i = 0; i < count && err == ESP_OK; i++) {
        const store_entry_t *e = &ents[i];
        /* The name is rebuilt from the sequence number rather than printed
         * from the directory entry: it keeps unescaped filesystem text out of
         * the page and guarantees the links match what parse_seq() accepts. */
        snprintf(name, sizeof(name), "cap%04d.wfl", e->seq);
        fmt_dur(e->duration_ms, dur, sizeof(dur));
        fmt_time(e->unix_start, when, sizeof(when));
        snprintf(row, sizeof(row),
                 "<div class=\"c\"><a class=\"d\" href=\"/f/%s\">%s</a>"
                 "<div class=\"m\">#%d | %" PRIu32 " KB | %s | %s</div>"
                 "<form method=\"post\" action=\"/rm/%s\">"
                 "<button type=\"submit\">delete</button></form></div>",
                 name, name, e->seq, (e->size + 1023) / 1024, dur, when, name);
        err = httpd_resp_sendstr_chunk(req, row);
    }

    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, k_page_tail);
    }
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, NULL);   /* terminating chunk */
    }
    free(ents);
    return err == ESP_OK ? ESP_OK : ESP_FAIL;
}

/* -------------------------------------------------------- HTTP: file dump */

static esp_err_t file_get(httpd_req_t *req)
{
    char path[64];
    /* Header values are referenced, not copied, so this has to stay alive
     * until the response is out - hence a function-scope buffer. */
    char disp[64];
    int seq = 0;

    if (!store_ready()) {
        return send_unavailable(req);
    }
    if (!parse_seq(req->uri, "/f/", &seq)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad capture name");
        return ESP_FAIL;
    }

    store_path(seq, path, sizeof(path));
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        /* Also the "deleted between listing and click" case. */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such capture");
        return ESP_FAIL;
    }
    char *buf = malloc(WEB_CHUNK);
    if (buf == NULL) {
        fclose(f);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_FAIL;
    }

    snprintf(disp, sizeof(disp), "attachment; filename=\"cap%04d.wfl\"", seq);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", disp);

    /* Chunked rather than one send: the file is megabytes and the heap is
     * not. The cost is that the browser gets no total size, so no progress
     * bar - the page already told the user how big the capture is. */
    esp_err_t err = ESP_OK;
    size_t n;
    while ((n = fread(buf, 1, WEB_CHUNK, f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) {
            ESP_LOGW(TAG, "%s: client went away", path);
            err = ESP_FAIL;
            break;
        }
    }
    if (err == ESP_OK && ferror(f)) {
        ESP_LOGE(TAG, "%s: read error", path);
        err = ESP_FAIL;
    }
    free(buf);
    fclose(f);

    if (err != ESP_OK) {
        /* No terminating chunk on a partial body: returning ESP_FAIL closes
         * the socket, which is what tells the client the file is truncated. */
        return ESP_FAIL;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* ----------------------------------------------------------- HTTP: delete */

/*
 * POST, not GET, even though a link would be one tag shorter: browsers and
 * scanners prefetch links, and a capture is not recoverable. A one-button
 * form is still plain HTML, works without scripting and taps well on a phone.
 */
static esp_err_t rm_post(httpd_req_t *req)
{
    int seq = 0;

    if (!store_ready()) {
        return send_unavailable(req);
    }
    if (!parse_seq(req->uri, "/rm/", &seq)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad capture name");
        return ESP_FAIL;
    }

    esp_err_t err = store_remove(seq);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "remove cap%04d: %s", seq, esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such capture");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "removed cap%04d.wfl", seq);

    /* 303 so the reload after the POST is a plain GET of the listing. */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, "", 0);
}

/* --------------------------------------------------------- HTTP: settings */

/*
 * The settings page edits the list of networks the Monitor may join. It is a
 * front end for wifi_store and keeps no state of its own, which is what makes
 * it agree with the console's `wifi add|list|del` by construction rather than
 * by both being kept up to date.
 *
 * Plain forms and no scripting, like the listing page: what has to work here
 * is whatever browser is on the rider's phone, in a car park, with the bike
 * switched off. Every POST answers 303 back to this page with a fixed code in
 * the query string, and the page turns that code into a sentence from a table
 * below. A code and not the message itself, because the alternative is a page
 * that prints back text a client sent it.
 */

/* Two fields, each at most three characters of encoding per byte: 32 for an
 * SSID and 63 for a passphrase come to a little over 300. */
#define WEB_FORM_MAX  512
/* An escaped SSID, worst case every character an "&quot;". */
#define WEB_ESC_MAX   (6 * WFOTA_SSID_MAX + 1)
/*
 * The escaped manifest URL. It needs a buffer of its own because
 * WFOTA_URL_MAX alone is longer than anything else settings_get() holds.
 * Nothing wfota_manifest_url() can build actually needs escaping - the
 * host is a literal and wfota_tag_ok() refuses every character esc_html()
 * expands - so twice the URL is room for an escape or two that should not
 * exist, and esc_html() truncates rather than overruns past that. Six
 * times, the true worst case, would be 1.5 KB of an 8 KB handler stack
 * spent guarding a URL this firmware has no way to construct.
 */
#define WEB_URL_ESC_MAX (2 * WFOTA_URL_MAX + 1)
/* A decoded field. Comfortably longer than either the SSID or the passphrase
 * wifi_store will take, so that something one character too long comes back as
 * a length wifi_store_add() complains about rather than as a form the server
 * could not read - the two are not the same mistake to have made. */
#define WEB_FIELD_MAX (WIFI_PASS_MAX + 2)

static const struct {
    const char *code;
    const char *text;
} k_msgs[] = {
    { "added",   "Network remembered." },
    { "gone",    "Network forgotten." },
    { "size",    "The name has to be 1 to 32 characters, and the passphrase "
                 "8 to 63 - or empty for an open network." },
    { "full",    "The list is full. Forget one to make room." },
    { "missing", "That network was not stored; it may already be gone." },
    { "form",    "That form could not be read." },
    { "chan",    "Update channel saved. The next update will read it." },
    { "badchan", "This firmware has no channel by that name. The channel "
                 "is unchanged." },
    { "flash",   "Writing to flash failed. The list is unchanged." },
};

static const char k_add_form[] =
    "<form class=\"c\" method=\"post\" action=\"/wifi/add\">"
    "<label>network name (SSID)"
    "<input name=\"ssid\" maxlength=\"32\" required autocomplete=\"off\" "
    "autocapitalize=\"none\" autocorrect=\"off\" spellcheck=\"false\"></label>"
    "<label>passphrase, or empty for an open network"
    "<input name=\"pass\" type=\"password\" minlength=\"8\" maxlength=\"63\" "
    "autocomplete=\"off\"></label>"
    "<button class=\"ok\" type=\"submit\">remember</button></form>";

static const char k_list_full[] =
    "<p class=\"e\">The list is full. Forget one to add another.</p>";

static const char k_settings_note[] =
    /* The manifest is read on the way into the mode, before this page
     * exists to be opened, so a channel saved here cannot be the one that
     * was just read. There is deliberately no "check now" button - the
     * install has to be confirmed on the device (ADR-0006), so a button
     * here could only start something the rider would then have to walk
     * back to the bike to answer. Saying so is the only way a rider learns
     * that saving a channel and walking away changed nothing yet. */
    "<p class=\"e\">A channel is read when this mode starts and looks for an "
    "update, which has already happened by the time this page is open. Save "
    "one here and it is read the next time the mode is entered.</p>"
    "<p class=\"e\">A passphrase is stored unencrypted and is never shown "
    "back, only counted - long enough to spot a typo by. Anyone who can reach "
    "this page can change this list: the access point's shared key, or the "
    "rider's own network, is the whole of what stands in front of it.</p>";

/* The banner above the list, or NULL for no banner. What arrives on the query
 * string is only ever looked up in the table, so nothing a client sends can
 * reach the page through here. */
static const char *msg_for(httpd_req_t *req)
{
    char query[40], code[12];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return NULL;
    }
    if (httpd_query_key_value(query, "m", code, sizeof(code)) != ESP_OK) {
        return NULL;
    }
    for (size_t i = 0; i < sizeof(k_msgs) / sizeof(k_msgs[0]); i++) {
        if (strcmp(code, k_msgs[i].code) == 0) {
            return k_msgs[i].text;
        }
    }
    return NULL;
}

/*
 * Escapes text for the page. An SSID is whatever a hotspot decided to call
 * itself, and here it lands in two places at once - as text, and as the value
 * of the hidden field the forget button posts back - so it has to survive
 * both without closing a tag or an attribute early. Callers pass WEB_ESC_MAX
 * bytes, which is the worst case for the longest SSID wifi_store will hold,
 * so the truncation below is a guard rather than a case that happens: a
 * shortened name would post back as a network nobody stored.
 */
static void esc_html(const char *in, char *out, size_t cap)
{
    size_t o = 0;

    for (; *in != '\0'; in++) {
        const char *rep = NULL;
        size_t      need;

        switch (*in) {
        case '&':  rep = "&amp;";  break;
        case '<':  rep = "&lt;";   break;
        case '>':  rep = "&gt;";   break;
        case '"':  rep = "&quot;"; break;
        case '\'': rep = "&#39;";  break;
        default:                   break;
        }
        need = (rep != NULL) ? strlen(rep) : 1;
        if (o + need + 1 > cap) {
            break;
        }
        if (rep != NULL) {
            memcpy(out + o, rep, need);
        } else {
            out[o] = *in;
        }
        o += need;
    }
    out[o] = '\0';
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

/*
 * Percent-decodes `len` bytes into `out`. False for a malformed escape, for a
 * result that does not fit, and for any control character - a passphrase with
 * a NUL in it would be stored truncated and then fail to join a network the
 * rider is certain they typed correctly. High bytes pass through: an SSID is
 * bytes, commonly UTF-8, and the store keeps whatever it is given.
 */
static bool url_decode(const char *in, size_t len, char *out, size_t cap)
{
    size_t o = 0;

    for (size_t i = 0; i < len; i++) {
        char c = in[i];

        if (c == '+') {
            c = ' ';
        } else if (c == '%') {
            int hi, lo;
            if (i + 2 >= len) {
                return false;
            }
            hi = hexval(in[i + 1]);
            lo = hexval(in[i + 2]);
            if (hi < 0 || lo < 0) {
                return false;
            }
            c = (char)((hi << 4) | lo);
            i += 2;
        }
        if ((unsigned char)c < 0x20 || (unsigned char)c == 0x7f) {
            return false;
        }
        if (o + 1 >= cap) {
            return false;
        }
        out[o++] = c;
    }
    out[o] = '\0';
    return true;
}

/*
 * Pulls one field out of an "ssid=the+cafe&pass=..." body. False when the
 * field is absent or will not decode; a field that is present and empty
 * decodes to "" and is true, which is how an open network arrives.
 */
static bool form_field(const char *body, const char *name, char *out, size_t cap)
{
    size_t      nlen = strlen(name);
    const char *p = body;

    while (*p != '\0') {
        if (strncmp(p, name, nlen) == 0 && p[nlen] == '=') {
            const char *val = p + nlen + 1;
            return url_decode(val, strcspn(val, "&"), out, cap);
        }
        p = strchr(p, '&');
        if (p == NULL) {
            break;
        }
        p++;
    }
    return false;
}

/* Reads a form body. Both forms here are tens of bytes, so a body that does
 * not fit did not come from a page this server served and is refused rather
 * than read in pieces. */
static bool read_body(httpd_req_t *req, char *buf, size_t cap)
{
    size_t len = (size_t)req->content_len;
    size_t got = 0;
    int    idle = 0;

    if (len == 0 || len >= cap) {
        return false;
    }
    while (got < len) {
        int r = httpd_req_recv(req, buf + got, len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            /* One retry. A client that has announced a body and then stops
             * sending it must not hold the single httpd task open. */
            if (++idle > 1) {
                return false;
            }
            continue;
        }
        if (r <= 0) {
            return false;
        }
        got += (size_t)r;
    }
    buf[got] = '\0';
    return true;
}

/* 303 so the reload after a POST is a plain GET, as on the listing page. The
 * Location string is referenced and not copied, so it has to outlive the
 * send - which it does, being this function's own buffer. */
static esp_err_t redirect_settings(httpd_req_t *req, const char *code)
{
    char loc[40];

    snprintf(loc, sizeof(loc), "/settings?m=%s", code);
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", loc);
    return httpd_resp_send(req, "", 0);
}

static esp_err_t settings_get(httpd_req_t *req)
{
    wifi_net_t  nets[WIFI_STORE_MAX];
    char        esc[WEB_ESC_MAX], sub[112], key[48], row[768];
    char        chan[WFOTA_CHANNEL_MAX + 1];
    char        url[WFOTA_URL_MAX + 1], urlesc[WEB_URL_ESC_MAX];
    const char *cur = WFOTA_CHANNEL_STABLE;
    const char *cname, *urltext;
    const char *msg = msg_for(req);
    esp_err_t   err;
    int         n;

    /* Deliberately not gated on store_ready(): the network list has nothing to
     * do with the capture store, and a board whose store will not mount is
     * exactly one that may need an update installed. */
    n = wifi_store_load(nets, WIFI_STORE_MAX);

    /* The name shown is the table's own entry and not the string that came
     * back out of flash. otaup_channel_get() already promises a name this
     * firmware knows; looking it up again means the page does not have to
     * take that on trust, and there is then no path by which a byte from NVS
     * reaches the markup. */
    otaup_channel_get(chan, sizeof(chan));
    for (int i = 0; (cname = wfota_channel_name(i)) != NULL; i++) {
        if (strcmp(cname, chan) == 0) {
            cur = cname;
            break;
        }
    }
    if (otaup_manifest_url(url, sizeof(url))) {
        /* Escaped although nothing wfota_manifest_url() can build needs it -
         * the host is a literal and wfota_tag_ok() gates the tail. The day a
         * tag reaches that URL from somewhere other than the channel table,
         * this line should not be the hole it goes through. */
        esc_html(url, urlesc, sizeof(urlesc));
        urltext = urlesc;
    } else {
        /* A phrase and not an empty string: a blank after "reads" would read
         * as "there is no update", which is a different and wrong answer. */
        urltext = "an address this build could not assemble";
    }

    err = send_head(req, "settings", k_nav_captures);
    if (err == ESP_OK) {
        snprintf(sub, sizeof(sub),
                 "<p class=\"sub\">%d of %d network(s) the Monitor may "
                 "join</p>", n, WIFI_STORE_MAX);
        err = httpd_resp_sendstr_chunk(req, sub);
    }
    if (err == ESP_OK && msg != NULL) {
        err = httpd_resp_sendstr_chunk(req, "<p class=\"msg\">");
        if (err == ESP_OK) {
            err = httpd_resp_sendstr_chunk(req, msg);
        }
        if (err == ESP_OK) {
            err = httpd_resp_sendstr_chunk(req, "</p>");
        }
    }
    /* Above the network list because "which stream is this bike on" is the
     * question a rider opens this page to settle, and the resolved URL under
     * the buttons is what answers it without a cable. Every name printed
     * comes out of wfota_channel_name(), so the page offers what this build
     * actually has rather than a pair somebody wrote down here once. */
    if (err == ESP_OK) {
        snprintf(row, sizeof(row),
                 "<form class=\"c\" method=\"post\" action=\"/ota/channel\">"
                 "<div class=\"n\">channel: %s</div>", cur);
        err = httpd_resp_sendstr_chunk(req, row);
    }
    for (int i = 0; err == ESP_OK &&
                    (cname = wfota_channel_name(i)) != NULL; i++) {
        snprintf(row, sizeof(row),
                 "<label class=\"r\"><input type=\"radio\" name=\"chan\" "
                 "value=\"%s\"%s>%s</label>",
                 cname, strcmp(cname, chan) == 0 ? " checked" : "", cname);
        err = httpd_resp_sendstr_chunk(req, row);
    }
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req,
            "<button class=\"ok\" type=\"submit\">use this channel</button>"
            "<div class=\"u\">the next update reads ");
    }
    /* Its own chunk: escaped, the URL is longer than `row` and every other
     * buffer in this function, and snprintf would answer that by silently
     * cutting it in half. */
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, urltext);
    }
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, "</div></form>");
    }

    if (err == ESP_OK && n == 0) {
        err = httpd_resp_sendstr_chunk(req,
            "<p class=\"e\">No networks stored, so there is nothing to "
            "join and the Monitor puts up this access point instead. Add one "
            "below and it will be joined from the next entry on - which is "
            "also what makes an update possible.</p>");
    }

    for (int i = 0; i < n && err == ESP_OK; i++) {
        size_t plen = strlen(nets[i].pass);

        esc_html(nets[i].ssid, esc, sizeof(esc));
        if (plen == 0) {
            snprintf(key, sizeof(key), "open network");
        } else {
            snprintf(key, sizeof(key), "passphrase of %u characters",
                     (unsigned)plen);
        }
        /* The SSID is the key wifi_store_del() takes, so the forget button
         * posts the name back rather than a position in a list that another
         * tab may have changed underneath it. */
        snprintf(row, sizeof(row),
                 "<div class=\"c\"><div class=\"n\">%s</div>"
                 "<div class=\"m\">%s</div>"
                 "<form method=\"post\" action=\"/wifi/del\">"
                 "<input type=\"hidden\" name=\"ssid\" value=\"%s\">"
                 "<button type=\"submit\">forget</button></form></div>",
                 esc, key, esc);
        err = httpd_resp_sendstr_chunk(req, row);
    }

    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req,
                  n >= WIFI_STORE_MAX ? k_list_full : k_add_form);
    }
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, k_settings_note);
    }
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, k_page_tail);
    }
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, NULL);   /* terminating chunk */
    }
    return err == ESP_OK ? ESP_OK : ESP_FAIL;
}

/*
 * Adding is also how a passphrase is rotated: wifi_store_add() replaces the
 * key of an SSID it already holds, so the page needs no separate edit, and
 * typing a stored network's name again is the way to change it.
 */
static esp_err_t wifi_add_post(httpd_req_t *req)
{
    char      body[WEB_FORM_MAX];
    char      ssid[WEB_FIELD_MAX];
    char      pass[WEB_FIELD_MAX];
    esp_err_t err;

    if (!read_body(req, body, sizeof(body)) ||
        !form_field(body, "ssid", ssid, sizeof(ssid))) {
        return redirect_settings(req, "form");
    }
    /* An absent field is an open network, the same as an empty one: a browser
     * that omits it and one that sends "pass=" mean the same thing. */
    if (!form_field(body, "pass", pass, sizeof(pass))) {
        pass[0] = '\0';
    }

    err = wifi_store_add(ssid, pass);
    if (err != ESP_OK) {
        /* The SSID is not logged: it names where the rider lives. */
        ESP_LOGW(TAG, "add network: %s", esp_err_to_name(err));
        return redirect_settings(req,
                   err == ESP_ERR_INVALID_SIZE ? "size" :
                   err == ESP_ERR_NO_MEM       ? "full" : "flash");
    }
    ESP_LOGI(TAG, "network list updated from the settings page");
    return redirect_settings(req, "added");
}

static esp_err_t wifi_del_post(httpd_req_t *req)
{
    char      body[WEB_FORM_MAX];
    char      ssid[WEB_FIELD_MAX];
    esp_err_t err;

    if (!read_body(req, body, sizeof(body)) ||
        !form_field(body, "ssid", ssid, sizeof(ssid))) {
        return redirect_settings(req, "form");
    }

    err = wifi_store_del(ssid);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "forget network: %s", esp_err_to_name(err));
        return redirect_settings(req,
                   err == ESP_ERR_NOT_FOUND ? "missing" : "flash");
    }
    ESP_LOGI(TAG, "network forgotten from the settings page");
    return redirect_settings(req, "gone");
}

/*
 * Chooses the release stream the next check will read. What crosses the wire
 * is a channel *name* and never a URL: otaup_channel_set() looks it up in the
 * compiled-in table, so the worst a client on this AP can do here is name a
 * channel that does not exist and be told so - it cannot point the Monitor at
 * an address of its own. The name is not echoed back either; "badchan" is a
 * code the page turns into a fixed sentence, like every other answer here.
 */
static esp_err_t ota_channel_post(httpd_req_t *req)
{
    char      body[WEB_FORM_MAX];
    /* WEB_FIELD_MAX rather than the channel's own length, for the reason the
     * wifi form uses it: a name one character too long should come back as a
     * channel nobody has, not as a form the server could not read. */
    char      chan[WEB_FIELD_MAX];
    esp_err_t err;

    if (!read_body(req, body, sizeof(body)) ||
        !form_field(body, "chan", chan, sizeof(chan))) {
        return redirect_settings(req, "form");
    }

    err = otaup_channel_set(chan);
    if (err != ESP_OK) {
        /* The refused name is not logged: it is client text, and the only
         * thing worth knowing here is that one was refused. */
        ESP_LOGW(TAG, "set channel: %s", esp_err_to_name(err));
        return redirect_settings(req,
                   err == ESP_ERR_INVALID_ARG ? "badchan" : "flash");
    }
    ESP_LOGI(TAG, "update channel set from the settings page");
    return redirect_settings(req, "chan");
}

static const httpd_uri_t k_uri_index = {
    .uri = "/", .method = HTTP_GET, .handler = index_get, .user_ctx = NULL,
};
static const httpd_uri_t k_uri_file = {
    .uri = "/f/*", .method = HTTP_GET, .handler = file_get, .user_ctx = NULL,
};
static const httpd_uri_t k_uri_rm = {
    .uri = "/rm/*", .method = HTTP_POST, .handler = rm_post, .user_ctx = NULL,
};
static const httpd_uri_t k_uri_settings = {
    .uri = "/settings", .method = HTTP_GET, .handler = settings_get,
    .user_ctx = NULL,
};
static const httpd_uri_t k_uri_wifi_add = {
    .uri = "/wifi/add", .method = HTTP_POST, .handler = wifi_add_post,
    .user_ctx = NULL,
};
static const httpd_uri_t k_uri_wifi_del = {
    .uri = "/wifi/del", .method = HTTP_POST, .handler = wifi_del_post,
    .user_ctx = NULL,
};
static const httpd_uri_t k_uri_ota_channel = {
    .uri = "/ota/channel", .method = HTTP_POST, .handler = ota_channel_post,
    .user_ctx = NULL,
};

/* ------------------------------------------------------- the fallback link */

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base != WIFI_EVENT || data == NULL) {
        return;
    }
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "station " MACSTR " joined (aid %d)", MAC2STR(e->mac), e->aid);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "station " MACSTR " left (aid %d)", MAC2STR(e->mac), e->aid);
    }
}

/* Wi-Fi keeps its calibration data in NVS. The app has normally initialised it
 * long before the mode is entered, so "already up" is the expected answer. */
static esp_err_t nvs_ready(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;
    }
    return err;
}

/* Undoes whatever web_ap_start() managed to bring up, in reverse order. Every
 * step is guarded, so this doubles as the failure path and as web_ap_stop().
 * It leaves the server alone: the httpd is not this link's to take down, and
 * the mode stops it first anyway. */
static void ap_teardown(void)
{
    if (s_wifi_evt != NULL) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_wifi_evt);
        s_wifi_evt = NULL;
    }
    if (s_wifi_inited) {
        esp_wifi_stop();
        esp_wifi_deinit();
        s_wifi_inited = false;
    }
    if (s_netif != NULL) {
        /* Takes the AP netif and its DHCP server down again. The default event
         * loop and esp_netif_init() stay up: they are process wide and other
         * components may be using them. */
        esp_netif_destroy_default_wifi(s_netif);
        s_netif = NULL;
    }
    s_ap_running = false;
}

esp_err_t web_ap_start(char *out_ssid, size_t ssid_cap, char *out_ip, size_t ip_cap)
{
    esp_err_t err = ESP_OK;
    uint8_t mac[6] = {0};
    char ssid[32] = {0};
    char ipbuf[16] = {0};
    const char *ipstr = WEB_FALLBACK_IP;
    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wcfg = {0};
    esp_netif_ip_info_t ip = {0};
    size_t slen = 0;

    if (s_ap_running) {
        return ESP_ERR_INVALID_STATE;
    }

    err = nvs_ready();
    if (err != ESP_OK) {
        goto fail;
    }
    err = esp_netif_init();
    if (err != ESP_OK) {
        goto fail;
    }
    err = esp_event_loop_create_default();
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;               /* the app already created it */
    }
    if (err != ESP_OK) {
        goto fail;
    }

    s_netif = esp_netif_create_default_wifi_ap();
    if (s_netif == NULL) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    err = esp_wifi_init(&icfg);
    if (err != ESP_OK) {
        goto fail;
    }
    s_wifi_inited = true;

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              wifi_event, NULL, &s_wifi_evt);
    if (err != ESP_OK) {
        goto fail;
    }

    /* The last three bytes of the station MAC make the SSID unique without a
     * setting to keep anywhere, and they are printed on the LCD so the rider
     * knows which of two boards to join. */
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(ssid, sizeof(ssid), WEB_SSID_PREFIX "%02x%02x%02x",
             mac[3], mac[4], mac[5]);

    slen = strlen(ssid);
    memcpy(wcfg.ap.ssid, ssid, slen);
    wcfg.ap.ssid_len = (uint8_t)slen;
    snprintf((char *)wcfg.ap.password, sizeof(wcfg.ap.password), "%s", WEB_PASSWORD);
    wcfg.ap.channel = WEB_CHANNEL;
    wcfg.ap.max_connection = WEB_MAX_CONN;
    wcfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wcfg.ap.pmf_cfg.required = false;

    /* The mode is transient; there is no reason to write the config to flash
     * and wear it out. */
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        goto fail;
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &wcfg);
    if (err != ESP_OK) {
        goto fail;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        goto fail;
    }

    /* Always 192.168.4.1 with the stock DHCP server, but read it back rather
     * than trust that, and fall back to the literal if the netif has none. */
    if (esp_netif_get_ip_info(s_netif, &ip) == ESP_OK && ip.ip.addr != 0) {
        esp_ip4addr_ntoa(&ip.ip, ipbuf, sizeof(ipbuf));
        ipstr = ipbuf;
    }

    if (out_ssid != NULL && ssid_cap > 0) {
        snprintf(out_ssid, ssid_cap, "%s", ssid);
    }
    if (out_ip != NULL && ip_cap > 0) {
        snprintf(out_ip, ip_cap, "%s", ipstr);
    }

    s_ap_running = true;
    ESP_LOGI(TAG, "AP \"%s\" up on %s, password \"%s\"", ssid, ipstr, WEB_PASSWORD);
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "access point failed: %s", esp_err_to_name(err));
    ap_teardown();
    if (out_ssid != NULL && ssid_cap > 0) {
        out_ssid[0] = '\0';
    }
    if (out_ip != NULL && ip_cap > 0) {
        out_ip[0] = '\0';
    }
    return err;
}

void web_ap_stop(void)
{
    if (!s_ap_running && !s_wifi_inited && s_netif == NULL) {
        return;                      /* never started, or stopped twice */
    }
    ESP_LOGI(TAG, "taking the access point down");
    ap_teardown();
}

bool web_ap_running(void)
{
    return s_ap_running;
}

/* ------------------------------------------------------------- the server */

esp_err_t web_serve_start(void)
{
    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    esp_err_t      err;

    if (s_server != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    hcfg.stack_size = 8192;          /* fread + the HTML formatting live here */
    hcfg.lru_purge_enable = true;    /* a phone that walks off must not wedge it */
    hcfg.max_open_sockets = 4;
    /* Seven routes today against a default of eight, which the channel form
     * was the seventh of. Said out loud so that the eighth is a number to
     * change here rather than a mode that will not start. */
    hcfg.max_uri_handlers = 10;
    /* the file and delete routes are registered as wildcard patterns */
    hcfg.uri_match_fn = httpd_uri_match_wildcard;
    err = httpd_start(&s_server, &hcfg);
    if (err != ESP_OK) {
        s_server = NULL;
        ESP_LOGE(TAG, "server failed: %s", esp_err_to_name(err));
        return err;
    }
    err = httpd_register_uri_handler(s_server, &k_uri_index);
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &k_uri_file);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &k_uri_rm);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &k_uri_settings);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &k_uri_wifi_add);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &k_uri_wifi_del);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &k_uri_ota_channel);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "routes failed: %s", esp_err_to_name(err));
        web_serve_stop();
        return err;
    }

    ESP_LOGI(TAG, "server up, %d capture(s)", store_ready() ? store_count() : -1);
    return ESP_OK;
}

void web_serve_stop(void)
{
    if (s_server == NULL) {
        return;                      /* never started, or stopped twice */
    }
    httpd_stop(s_server);
    s_server = NULL;
}

bool web_running(void)
{
    return s_server != NULL;
}

int web_clients(void)
{
    wifi_sta_list_t list = {0};

    /* Only the access point has stations of its own. Asked while the Monitor
     * is somebody else's station, esp_wifi_ap_get_sta_list() would answer for
     * an interface that is not up, so the question is refused here instead. */
    if (!s_ap_running) {
        return 0;
    }
    if (esp_wifi_ap_get_sta_list(&list) != ESP_OK) {
        return 0;
    }
    return list.num;
}
