/* ota_update - see ota_update.h. */
#include "ota_update.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wifi_store.h"

#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "otaup";

#define OTAUP_NS        "ota"
#define OTAUP_PIN_KEY   "pin"
#define OTAUP_CHAN_KEY  "chan"
#define OTAUP_TRIAL_KEY "trial"

/* More access points than a car park has, and the list is on the stack of the
 * task below; past this the extra ones are the faint ones anyway. */
#define SCAN_MAX        24
#define JOIN_TIMEOUT_MS 20000
#define HTTP_TIMEOUT_MS 15000
/*
 * Neither the check nor the install runs on the task that asked for it. The
 * TLS handshake wants several kilobytes of stack on top of the HTTP client's
 * own, and both callers - the button task at 4 KB and the console REPL - have
 * their stacks sized for something else entirely. A task with a stack this
 * file chooses is the only way that stays true when either caller is resized.
 */
#define CHECK_STACK     10240
#define CHECK_PRIO      3
/*
 * The receive buffer for the image, which is also where the response headers
 * land, and it is the headers that size it: github.com answers a release
 * download with a header block of about 5 KB, most of it one
 * Content-Security-Policy line of about 3.6 KB. Larger than the manifest
 * fetch's only because every buffer-full here is an esp_ota_write() call, and
 * there are a thousand of them.
 */
#define IMAGE_BUF       8192
/*
 * The transmit buffer, which is a different problem and was the one that
 * broke. The request line and every request header are assembled in this one
 * buffer before anything goes out, so it is sized by the URL and not by the
 * answer. ESP-IDF leaves it at 512 bytes unless it is asked, and 512 is
 * enough for the two github.com hops and not for the third: the release asset
 * redirects to a signed release-assets.githubusercontent.com URL of about
 * 1030 characters - fourteen query parameters, one of them a JWT - and "GET "
 * plus that plus " HTTP/1.1" plus Host, User-Agent, Accept and Connection
 * comes to roughly 1250. One byte over and esp_http_client logs "Out of
 * buffer" and gives up in prepare_first_line(), before a single byte of the
 * response is read: a certificate validated, then ESP_FAIL, which is exactly
 * what the board showed.
 *
 * 2048 is that 1250 with about 800 bytes to spare. Not the measured figure,
 * because the signature is GitHub's to lengthen and the failure when it is
 * one byte short is total; not more, because this is heap held while Wi-Fi
 * and TLS are both up. The cost over the default is 1.5 KB for the few
 * seconds the client is open, against the ~160 KB free at that moment.
 *
 * CONFIG_ESP_HTTP_CLIENT_STRICT_HEADER_BUFFER (default y, and y here) is
 * about this buffer too, despite reading like a response setting: with it a
 * request header that does not fit is ESP_ERR_HTTP_HEADER_TOO_LONG, and
 * without it the request goes out without its terminator and the server times
 * the connection out instead. It does not guard the request line, which is
 * where this one overflowed, so it would not have named the fault either way.
 */
#define REQ_BUF         2048

#define BIT_GOT_IP  BIT0
#define BIT_FAILED  BIT1

static esp_netif_t                 *s_netif;
static esp_event_handler_instance_t s_wifi_evt;
static esp_event_handler_instance_t s_ip_evt;
static EventGroupHandle_t           s_events;
static bool                         s_wifi_inited;
static bool                         s_running;
static char                         s_ip[16];
static uint8_t                      s_disc_reason;
/* Which network the link is on, kept here rather than only in the join's
 * result: the manifest check that follows may be a separate call minutes
 * later - the console's `ota check` over a link the menu brought up - and its
 * result has to describe the same link the first one did. */
static char                         s_ssid[WFOTA_SSID_MAX + 1];
static int                          s_rssi;

/* ------------------------------------------------------------------ the pin */

bool otaup_pin_get(char *out, size_t cap)
{
    nvs_handle_t h;

    if (out == NULL || cap == 0) {
        return false;
    }
    out[0] = '\0';
    if (nvs_open(OTAUP_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;               /* nothing has ever been pinned */
    }
    size_t len = cap;
    esp_err_t err = nvs_get_str(h, OTAUP_PIN_KEY, out, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        out[0] = '\0';
        return false;
    }
    /* Checked on the way out as well as on the way in: these bytes are pasted
     * into a URL, and flash is not where a URL should be trusted from. */
    if (!wfota_tag_ok(out)) {
        ESP_LOGW(TAG, "the stored pin is not a tag - ignoring it");
        out[0] = '\0';
        return false;
    }
    return true;
}

esp_err_t otaup_pin_set(const char *tag)
{
    nvs_handle_t h;
    esp_err_t    err;

    if (tag != NULL && tag[0] != '\0' && !wfota_tag_ok(tag)) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_open(OTAUP_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    if (tag == NULL || tag[0] == '\0') {
        err = nvs_erase_key(h, OTAUP_PIN_KEY);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;           /* clearing a pin nobody set */
        }
    } else {
        err = nvs_set_str(h, OTAUP_PIN_KEY, tag);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/* -------------------------------------------------------------- the channel */

void otaup_channel_get(char *out, size_t cap)
{
    nvs_handle_t h;
    char         name[WFOTA_CHANNEL_MAX + 1];

    if (out == NULL || cap == 0) {
        return;
    }
    /* Written first and left alone by every path out of here, so that there
     * is no way to leave this function without a channel the caller can use. */
    snprintf(out, cap, "%s", WFOTA_CHANNEL_STABLE);
    if (nvs_open(OTAUP_NS, NVS_READONLY, &h) != ESP_OK) {
        return;                     /* no channel has ever been chosen */
    }
    size_t    len = sizeof(name);
    esp_err_t err = nvs_get_str(h, OTAUP_CHAN_KEY, name, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        return;
    }
    /* Checked on the way out as well as on the way in, for the same reason as
     * the pin: this picks the URL the next check reads, and flash is not where
     * a URL should be trusted from. An empty value is not a name either,
     * whatever put it there. */
    if (name[0] == '\0' || wfota_channel_tag(name) == NULL) {
        ESP_LOGW(TAG, "the stored channel is not one this build has - stable");
        return;
    }
    snprintf(out, cap, "%s", name);
}

esp_err_t otaup_channel_set(const char *name)
{
    nvs_handle_t h;
    esp_err_t    err;
    /* stable is not stored as a value: it is what is left when the key is
     * gone, which is the whole reason a lost or unreadable setting is safe. */
    bool         clear = (name == NULL || name[0] == '\0' ||
                          strcmp(name, WFOTA_CHANNEL_STABLE) == 0);

    if (!clear && wfota_channel_tag(name) == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_open(OTAUP_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    if (clear) {
        err = nvs_erase_key(h, OTAUP_CHAN_KEY);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;           /* already on stable */
        }
    } else {
        err = nvs_set_str(h, OTAUP_CHAN_KEY, name);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/* ---------------------------------------------------------------- the trial */

bool otaup_trial_get(char *out, size_t cap)
{
    nvs_handle_t h;

    if (out == NULL || cap == 0) {
        return false;
    }
    out[0] = '\0';
    if (nvs_open(OTAUP_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;               /* nothing has ever been installed */
    }
    size_t    len = cap;
    esp_err_t err = nvs_get_str(h, OTAUP_TRIAL_KEY, out, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        out[0] = '\0';
        return false;
    }
    return out[0] != '\0';
}

/* NULL or "" erases, same convention as the pin. Static because the only
 * thing that writes a trial is the install below, which knows what it wrote
 * into the slot; everyone else only ever has one to clear. */
static esp_err_t trial_store(const char *chan)
{
    nvs_handle_t h;
    esp_err_t    err = nvs_open(OTAUP_NS, NVS_READWRITE, &h);

    if (err != ESP_OK) {
        return err;
    }
    if (chan == NULL || chan[0] == '\0') {
        err = nvs_erase_key(h, OTAUP_TRIAL_KEY);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;           /* no install was outstanding */
        }
    } else {
        err = nvs_set_str(h, OTAUP_TRIAL_KEY, chan);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

void otaup_trial_clear(void)
{
    esp_err_t err = trial_store(NULL);

    if (err != ESP_OK) {
        /* Nothing to do about it and nothing that breaks: a trial left behind
         * costs one needless revert to stable after a rollback that has not
         * happened yet, which is the harmless direction to fail in. */
        ESP_LOGW(TAG, "trial channel not cleared: %s", esp_err_to_name(err));
    }
}

bool otaup_manifest_url(char *out, size_t cap)
{
    char chan[WFOTA_CHANNEL_MAX + 1];
    char pin[WFOTA_VERSION_MAX + 1];

    otaup_channel_get(chan, sizeof(chan));   /* "stable" unless one is set */
    (void)otaup_pin_get(pin, sizeof(pin));   /* empty when nothing is pinned */
    return wfota_manifest_url(chan, pin, out, cap);
}

/* ------------------------------------------------------------------- radio */

static void net_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *e = data;
        /* No reconnect here, deliberately (ADR-0006): the rider is standing
         * next to the Monitor, and a retry loop would hide a hotspot that is
         * too weak rather than say so. */
        s_disc_reason = (e != NULL) ? e->reason : 0;
        xEventGroupSetBits(s_events, BIT_FAILED);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        if (e != NULL) {
            esp_ip4addr_ntoa(&e->ip_info.ip, s_ip, sizeof(s_ip));
        }
        xEventGroupSetBits(s_events, BIT_GOT_IP);
    }
}

/* Wi-Fi keeps its calibration data in NVS; the app has normally initialised it
 * long before the mode is entered, so "already up" is the expected answer.
 * Same shape as webdump.c, which owns the other half of this radio. */
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

/* Undoes whatever wifi_up() managed, in reverse order and every step guarded,
 * so this is both the failure path and otaup_stop(). */
static void teardown(void)
{
    if (s_wifi_inited) {
        esp_wifi_disconnect();
        esp_wifi_stop();
        esp_wifi_deinit();
        s_wifi_inited = false;
    }
    if (s_wifi_evt != NULL) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_wifi_evt);
        s_wifi_evt = NULL;
    }
    if (s_ip_evt != NULL) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_ip_evt);
        s_ip_evt = NULL;
    }
    if (s_netif != NULL) {
        esp_netif_destroy_default_wifi(s_netif);
        s_netif = NULL;
    }
    if (s_events != NULL) {
        vEventGroupDelete(s_events);
        s_events = NULL;
    }
    s_ip[0] = '\0';
    s_ssid[0] = '\0';
    s_rssi = 0;
    s_running = false;
}

static esp_err_t wifi_up(void)
{
    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t          err;

    s_events = xEventGroupCreate();
    if (s_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    err = nvs_ready();
    if (err != ESP_OK) {
        return err;
    }
    err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;               /* the app already created it */
    }
    if (err != ESP_OK) {
        return err;
    }

    s_netif = esp_netif_create_default_wifi_sta();
    if (s_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }
    err = esp_wifi_init(&icfg);
    if (err != ESP_OK) {
        return err;
    }
    s_wifi_inited = true;

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              net_event, NULL, &s_wifi_evt);
    if (err == ESP_OK) {
        err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                  net_event, NULL, &s_ip_evt);
    }
    if (err != ESP_OK) {
        return err;
    }

    /* The link is transient and the credentials already live in the `wifi`
     * namespace, so there is nothing to gain from letting the driver write a
     * second copy of them into flash on every join. */
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) {
        err = esp_wifi_start();
    }
    return err;
}

/* ------------------------------------------------------------- the manifest */

typedef struct {
    char  *buf;
    size_t cap;
    size_t len;
    bool   over;      /* the body outgrew the cap and was abandoned */
} body_t;

static esp_err_t http_event(esp_http_client_event_t *evt)
{
    body_t *b = (evt != NULL) ? evt->user_data : NULL;

    if (evt == NULL || evt->event_id != HTTP_EVENT_ON_DATA || b == NULL) {
        return ESP_OK;
    }
    /* GitHub answers the release URL with two redirects - to the newest tag's
     * own download URL, and from there to release-assets.githubusercontent.com
     * - and each of them has a short body of its own. Only the body of the 200
     * is the manifest, so the status decides what is collected. */
    if (esp_http_client_get_status_code(evt->client) != 200) {
        return ESP_OK;
    }
    if (b->len + (size_t)evt->data_len > b->cap) {
        /* Failing the transfer rather than truncating: a body this size is
         * not our manifest, and reading the rest of it would only spend the
         * heap to reach the same answer. */
        b->over = true;
        return ESP_FAIL;
    }
    memcpy(b->buf + b->len, evt->data, (size_t)evt->data_len);
    b->len += (size_t)evt->data_len;
    return ESP_OK;
}

static otaup_err_t fetch_manifest(otaup_result_t *r)
{
    char url[256];

    if (!otaup_manifest_url(url, sizeof(url))) {
        snprintf(r->detail, sizeof(r->detail), "bad url");
        return OTAUP_ERR_FETCH;
    }

    body_t b = {0};
    b.cap = WFOTA_BODY_MAX;
    b.buf = malloc(b.cap);
    if (b.buf == NULL) {
        snprintf(r->detail, sizeof(r->detail), "no memory");
        return OTAUP_ERR_FETCH;
    }

    esp_http_client_config_t cfg = {
        .url                   = url,
        .method                = HTTP_METHOD_GET,
        .timeout_ms            = HTTP_TIMEOUT_MS,
        /* ADR-0006: the bundle, never a pinned certificate. */
        .crt_bundle_attach     = esp_crt_bundle_attach,
        .event_handler         = http_event,
        .user_data             = &b,
        /* Sized for GitHub's headers, not for the manifest: the redirect from
         * github.com carries a Content-Security-Policy line of about 3.6 KB on
         * its own. The transmit buffer is sized by the signed URL of the last
         * hop and is the reason this fetch used to die before it read
         * anything; see REQ_BUF. Both are freed with the client, a few seconds
         * later. */
        .buffer_size           = 6144,
        .buffer_size_tx        = REQ_BUF,
        .keep_alive_enable     = false,
        /* Following the redirect is the point: the URL the Monitor knows is a
         * redirect to whichever release is newest. */
        .disable_auto_redirect = false,
    };

    ESP_LOGI(TAG, "GET %s", url);
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (c == NULL) {
        free(b.buf);
        snprintf(r->detail, sizeof(r->detail), "no client");
        return OTAUP_ERR_FETCH;
    }
    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);

    otaup_err_t res = OTAUP_OK;
    if (b.over) {
        /* Reported as a manifest problem rather than a transport one: the
         * transfer worked, what it carried is not a manifest. */
        snprintf(r->detail, sizeof(r->detail), "over %d bytes", WFOTA_BODY_MAX);
        res = OTAUP_ERR_MANIFEST;
    } else if (err != ESP_OK) {
        /* The esp_err_t goes to the log and the words go to the panel. A
         * rider standing next to the bike can do something about a hotspot
         * that stopped answering and nothing whatever about "ESP_FAIL"; the
         * name is only worth having where there is a keyboard to look it up
         * with. Every detail below is split the same way. */
        ESP_LOGE(TAG, "manifest: transport: %s", esp_err_to_name(err));
        snprintf(r->detail, sizeof(r->detail), "github did not answer");
        res = OTAUP_ERR_FETCH;
    } else if (status != 200) {
        /* 404 is the ordinary one: no release has been cut, or the pinned tag
         * does not exist. */
        snprintf(r->detail, sizeof(r->detail), "http %d", status);
        res = OTAUP_ERR_FETCH;
    } else {
        wfota_err_t perr = wfota_manifest_parse(b.buf, b.len, &r->manifest);
        if (perr != WFOTA_OK) {
            snprintf(r->detail, sizeof(r->detail), "%s", wfota_err_str(perr));
            res = OTAUP_ERR_MANIFEST;
        } else {
            r->have_manifest = true;
            /* ADR-0006: inequality, not order. Going back to an older build
             * is the same operation as going forward, which is what makes a
             * pin a way out of a bad release. */
            r->differs = (strcmp(r->manifest.version, r->running) != 0);
        }
    }

    free(b.buf);
    if (res != OTAUP_OK) {
        ESP_LOGE(TAG, "manifest: %s (%s)", otaup_err_str(res), r->detail);
    }
    return res;
}

/* --------------------------------------------------------------- the link */

static void say(otaup_progress_fn progress, const char *l1, const char *l2)
{
    if (progress != NULL) {
        progress(l1, l2);
    }
}

static otaup_err_t join(const wifi_net_t *net, otaup_result_t *r)
{
    wifi_config_t wc = {0};
    size_t        slen = strlen(net->ssid);

    /* wifi_config_t's ssid is 32 octets and is not a C string, so it is
     * copied by length; the passphrase field does have room for its NUL. */
    memcpy(wc.sta.ssid, net->ssid, slen > 32 ? 32 : slen);
    snprintf((char *)wc.sta.password, sizeof(wc.sta.password), "%s", net->pass);
    /* Whatever the access point offers, including open: the passphrase, or
     * the absence of one, is what decides whether the join works. */
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wc);
    if (err == ESP_OK) {
        err = esp_wifi_connect();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "join: %s", esp_err_to_name(err));
        snprintf(r->detail, sizeof(r->detail), "radio refused it");
        return OTAUP_ERR_JOIN;
    }

    EventBits_t bits = xEventGroupWaitBits(s_events, BIT_GOT_IP | BIT_FAILED,
                                           pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(JOIN_TIMEOUT_MS));
    if ((bits & BIT_GOT_IP) != 0) {
        snprintf(r->ip, sizeof(r->ip), "%s", s_ip);
        return OTAUP_OK;
    }
    if ((bits & BIT_FAILED) != 0) {
        /* The reason code is worth carrying: 15 is a wrong passphrase and 201
         * is an access point that went away, and those are different problems
         * standing in a car park. */
        snprintf(r->detail, sizeof(r->detail), "reason %u", s_disc_reason);
    } else {
        snprintf(r->detail, sizeof(r->detail), "no address");
    }
    return OTAUP_ERR_JOIN;
}

/* The running app's own version, which every result carries whether or not
 * anything else in it worked: it is half of the comparison the manifest is
 * read to make, and it is what the panel falls back to saying. */
static void fill_running(otaup_result_t *r)
{
    const esp_app_desc_t *desc = esp_app_get_description();

    snprintf(r->running, sizeof(r->running), "%s",
             (desc != NULL) ? desc->version : "");
}

/* What the link is, for a result that did not bring it up itself. */
static void fill_link(otaup_result_t *r)
{
    snprintf(r->ssid, sizeof(r->ssid), "%s", s_ssid);
    snprintf(r->ip, sizeof(r->ip), "%s", s_ip);
    r->rssi = s_rssi;
}

/*
 * Scans, picks and joins, and stops there. Everything up to a station with an
 * address, which is what the mode needs before it can serve anything - the
 * manifest is a separate question asked over the link this leaves up, because
 * a Monitor that joined a hotspot with no route to GitHub is still a Monitor
 * a rider can pull a Capture off (#41).
 */
static otaup_err_t run_join(otaup_result_t *r, otaup_progress_fn progress)
{
    wifi_net_t   nets[WIFI_STORE_MAX];
    const char  *known[WIFI_STORE_MAX];
    wfota_seen_t seen[SCAN_MAX];

    fill_running(r);

    int n_nets = wifi_store_load(nets, WIFI_STORE_MAX);
    if (n_nets == 0) {
        /* Nothing to scan for. Said before the radio comes up at all, because
         * the answer does not depend on what is in range - which is what
         * spares a fresh Monitor a scan on the way to the access point it was
         * always going to end up on. */
        return OTAUP_ERR_NO_NETS;
    }
    for (int i = 0; i < n_nets; i++) {
        known[i] = nets[i].ssid;
    }

    say(progress, "starting", NULL);
    esp_err_t err = wifi_up();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi up: %s", esp_err_to_name(err));
        snprintf(r->detail, sizeof(r->detail), "radio would not start");
        return OTAUP_ERR_WIFI;
    }
    s_running = true;

    say(progress, "scanning", NULL);
    wifi_scan_config_t sc = {0};
    /* Active, all channels: a phone hotspot sits on whichever channel the
     * phone picked, and there is nothing else for this radio to do. */
    err = esp_wifi_scan_start(&sc, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "scan: %s", esp_err_to_name(err));
        snprintf(r->detail, sizeof(r->detail), "scan would not start");
        return OTAUP_ERR_WIFI;
    }

    uint16_t found = 0;
    esp_wifi_scan_get_ap_num(&found);
    uint16_t take = (found > SCAN_MAX) ? SCAN_MAX : found;
    int n_seen = 0;
    if (take > 0) {
        wifi_ap_record_t *recs = calloc(take, sizeof(*recs));
        if (recs == NULL) {
            esp_wifi_clear_ap_list();
            snprintf(r->detail, sizeof(r->detail), "no memory");
            return OTAUP_ERR_WIFI;
        }
        /* Also frees the driver's own copy of the list, so it has to be
         * called even when only some of the records are wanted. */
        err = esp_wifi_scan_get_ap_records(&take, recs);
        if (err == ESP_OK) {
            for (uint16_t i = 0; i < take; i++) {
                snprintf(seen[n_seen].ssid, sizeof(seen[n_seen].ssid), "%s",
                         (const char *)recs[i].ssid);
                seen[n_seen].rssi = recs[i].rssi;
                n_seen++;
            }
        }
        free(recs);
    }
    ESP_LOGI(TAG, "scan: %u access point(s), %d known", found, n_nets);
    if (n_seen == 0) {
        return OTAUP_ERR_NO_SCAN;
    }

    int pick = wfota_pick_network(known, n_nets, seen, n_seen);
    if (pick < 0) {
        snprintf(r->detail, sizeof(r->detail), "%d seen", n_seen);
        return OTAUP_ERR_NO_KNOWN;
    }
    snprintf(r->ssid, sizeof(r->ssid), "%s", seen[pick].ssid);
    r->rssi = seen[pick].rssi;

    /* The scan result names the network; the stored entry carries its key. */
    const wifi_net_t *net = NULL;
    for (int i = 0; i < n_nets; i++) {
        if (strcmp(nets[i].ssid, r->ssid) == 0) {
            net = &nets[i];
            break;
        }
    }
    if (net == NULL) {
        return OTAUP_ERR_NO_KNOWN;   /* cannot happen: the pick came from it */
    }

    say(progress, "joining", r->ssid);
    ESP_LOGI(TAG, "joining \"%s\" at %d dBm", r->ssid, r->rssi);
    otaup_err_t uerr = join(net, r);
    if (uerr != OTAUP_OK) {
        return uerr;
    }
    snprintf(s_ssid, sizeof(s_ssid), "%s", r->ssid);
    s_rssi = r->rssi;
    ESP_LOGI(TAG, "joined \"%s\", address %s", r->ssid, r->ip);
    return OTAUP_OK;
}

/* ------------------------------------------------------- on our own task */

typedef struct {
    int             (*fn)(void *);
    void             *arg;
    int               rc;
    SemaphoreHandle_t done;
} job_t;

static void job_task(void *arg)
{
    job_t *j = arg;

    j->rc = j->fn(j->arg);
    xSemaphoreGive(j->done);
    vTaskDelete(NULL);
}

/* Runs `fn` on a task with the stack this file chose and waits for it. False
 * only when the task could not be started, which is the one failure the
 * caller has to have an answer of its own for. */
static bool run_on_own_task(const char *name, int (*fn)(void *), void *arg,
                            int *rc)
{
    job_t j = {.fn = fn, .arg = arg, .rc = 0, .done = NULL};

    j.done = xSemaphoreCreateBinary();
    if (j.done == NULL) {
        return false;
    }
    if (xTaskCreate(job_task, name, CHECK_STACK, &j, CHECK_PRIO, NULL) != pdPASS) {
        vSemaphoreDelete(j.done);
        return false;
    }
    /* No timeout: every step inside has one of its own, and `j` lives on this
     * stack, so returning while the task still holds it is not an option. */
    xSemaphoreTake(j.done, portMAX_DELAY);
    vSemaphoreDelete(j.done);
    *rc = j.rc;
    return true;
}

typedef struct {
    otaup_result_t   *result;
    otaup_progress_fn progress;
} check_ctx_t;

static int join_job(void *arg)
{
    check_ctx_t *ctx = arg;
    otaup_err_t  err = run_join(ctx->result, ctx->progress);

    if (err != OTAUP_OK) {
        /* Left failed, and left tidy: the radio goes back down so the failure
         * costs no heap while the caller brings the access point up over the
         * top of it. Nothing here retries, and nothing here falls back - which
         * of those to do is the mode's call, not the radio's. */
        teardown();
    }
    return (int)err;
}

otaup_err_t otaup_join(otaup_result_t *out, otaup_progress_fn progress)
{
    check_ctx_t ctx;
    int         rc = 0;

    if (out == NULL) {
        return OTAUP_ERR_STATE;
    }
    memset(out, 0, sizeof(*out));
    if (s_running) {
        return OTAUP_ERR_STATE;
    }

    ctx.result   = out;
    ctx.progress = progress;
    if (!run_on_own_task("otajoin", join_job, &ctx, &rc)) {
        return OTAUP_ERR_WIFI;
    }
    return (otaup_err_t)rc;
}

/*
 * The manifest over the link that is already up, and the one thing this file
 * does that does not tear the radio down when it fails. It cannot: the server
 * in webdump.c is listening on that link, and a hotspot that will not reach
 * GitHub is no reason to take the Capture listing away from a rider standing
 * in front of it. The mode is told, and the mode decides.
 */
static int manifest_job(void *arg)
{
    check_ctx_t *ctx = arg;

    return (int)fetch_manifest(ctx->result);
}

otaup_err_t otaup_manifest_check(otaup_result_t *out, otaup_progress_fn progress)
{
    check_ctx_t ctx;
    int         rc = 0;

    if (out == NULL) {
        return OTAUP_ERR_STATE;
    }
    memset(out, 0, sizeof(*out));
    if (!s_running) {
        /* No upstream. The access point the mode falls back to has no route
         * anywhere, so there is nothing to ask and no pretending otherwise. */
        snprintf(out->detail, sizeof(out->detail), "no network joined");
        return OTAUP_ERR_STATE;
    }
    fill_running(out);
    fill_link(out);

    /* The one line this step has to show, said here rather than from the
     * task: the fetch is a single blocking call with nothing to report part
     * way through, so `progress` is carried into the context only because
     * both jobs share it. */
    say(progress, "reading", out->ip);
    ctx.result   = out;
    ctx.progress = progress;
    if (!run_on_own_task("otamanifest", manifest_job, &ctx, &rc)) {
        snprintf(out->detail, sizeof(out->detail), "no task");
        return OTAUP_ERR_FETCH;
    }
    return (otaup_err_t)rc;
}

/* ---------------------------------------------------------- the install */

typedef struct {
    const wfota_manifest_t *m;
    otain_result_t         *r;
    otain_progress_fn       progress;
    esp_ota_handle_t        handle;
    bool                    open;       /* esp_ota_begin() succeeded */
    esp_err_t               write_err;  /* what esp_ota_write() said, if it did */
    wfota_dl_t              dl;
} install_ctx_t;

/*
 * Every piece of the body, in the order it arrives. Two things happen here
 * and their order is the point: the length is checked first, so the piece
 * that would take the slot past the size the manifest promised is refused
 * whole and never reaches flash, and only then is it written.
 *
 * Returning ESP_FAIL ends the transfer. Which failure it was is in the
 * context, because the client reports only that the handler said no.
 */
static esp_err_t image_event(esp_http_client_event_t *evt)
{
    install_ctx_t *ic = (evt != NULL) ? evt->user_data : NULL;

    if (evt == NULL || evt->event_id != HTTP_EVENT_ON_DATA || ic == NULL) {
        return ESP_OK;
    }
    /* Same rule as the manifest fetch: GitHub answers with redirects that end
     * at release-assets.githubusercontent.com, and each of them carries a body
     * of its own. Only the body of the 200 is the image. */
    if (esp_http_client_get_status_code(evt->client) != 200) {
        return ESP_OK;
    }
    if (wfota_dl_feed(&ic->dl, evt->data, (size_t)evt->data_len) != WFOTA_DL_OK) {
        return ESP_FAIL;
    }
    esp_err_t err = esp_ota_write(ic->handle, evt->data, (size_t)evt->data_len);
    if (err != ESP_OK) {
        ic->write_err = err;
        return ESP_FAIL;
    }

    int pct = wfota_dl_step(&ic->dl);
    if (pct >= 0 && ic->progress != NULL) {
        ic->progress(pct, ic->dl.got, ic->dl.want);
    }
    return ESP_OK;
}

static otain_err_t install(install_ctx_t *ic)
{
    const wfota_manifest_t *m = ic->m;
    otain_result_t         *r = ic->r;

    if (!s_running || m == NULL) {
        /* otaup_join() leaves the station joined precisely so that this can
         * run through it; without that link there is nothing to download. */
        return OTAIN_ERR_STATE;
    }

    /* The inactive slot, never the running one. This is the only call that
     * knows which of ota_0 and ota_1 this firmware is not in, which is why
     * the partition is asked for rather than named. */
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (part == NULL) {
        snprintf(r->detail, sizeof(r->detail), "no spare slot");
        return OTAIN_ERR_SLOT;
    }
    snprintf(r->slot, sizeof(r->slot), "%s", part->label);
    /* wfota_manifest_parse() already refused anything over WFOTA_IMAGE_MAX;
     * this asks the partition table on the board rather than the constant
     * compiled into the image, because the two are allowed to disagree - a
     * firmware built before a table change is exactly the firmware that would
     * be installing over the air. */
    if (m->size > part->size) {
        snprintf(r->detail, sizeof(r->detail), "%u > %u bytes",
                 (unsigned)m->size, (unsigned)part->size);
        return OTAIN_ERR_SLOT;
    }

    ESP_LOGI(TAG, "installing %s into %s at 0x%06" PRIx32 ", %u bytes",
             m->version, part->label, part->address, (unsigned)m->size);
    ESP_LOGI(TAG, "GET %s", m->url);

    /* The size is passed rather than OTA_SIZE_UNKNOWN so that exactly the
     * space the image needs is erased: the whole 1.875 MB slot would be most
     * of a minute of the rider's time, spent erasing flash nothing writes. */
    esp_err_t err = esp_ota_begin(part, m->size, &ic->handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota begin: %s", esp_err_to_name(err));
        snprintf(r->detail, sizeof(r->detail), "slot would not erase");
        return OTAIN_ERR_BEGIN;
    }
    ic->open = true;
    wfota_dl_begin(&ic->dl, m);

    esp_http_client_config_t cfg = {
        .url                   = m->url,
        .method                = HTTP_METHOD_GET,
        .timeout_ms            = HTTP_TIMEOUT_MS,
        /* ADR-0006: the bundle, never a pinned certificate. */
        .crt_bundle_attach     = esp_crt_bundle_attach,
        .event_handler         = image_event,
        .user_data             = ic,
        .buffer_size           = IMAGE_BUF,
        /* The same signed last hop the manifest fetch takes, so the same
         * transmit buffer: see REQ_BUF. */
        .buffer_size_tx        = REQ_BUF,
        .keep_alive_enable     = false,
        /* The manifest's url is a github.com release asset, which redirects
         * to the release-assets.githubusercontent.com URL the bytes actually
         * live behind. */
        .disable_auto_redirect = false,
    };

    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (c == NULL) {
        snprintf(r->detail, sizeof(r->detail), "no client");
        return OTAIN_ERR_FETCH;
    }
    err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);

    r->written = ic->dl.got;

    /* Read in the order that names the cause rather than the symptom: a
     * handler that said no makes perform() fail too, so what the handler was
     * unhappy about is asked first. */
    if (ic->write_err != ESP_OK) {
        ESP_LOGE(TAG, "ota write: %s", esp_err_to_name(ic->write_err));
        snprintf(r->detail, sizeof(r->detail), "flash refused a block");
        return OTAIN_ERR_WRITE;
    }
    if (ic->dl.err == WFOTA_DL_ERR_LONG) {
        snprintf(r->detail, sizeof(r->detail), "over %u bytes", (unsigned)m->size);
        return OTAIN_ERR_SIZE;
    }
    if (err != ESP_OK) {
        /* The dropped hotspot. Nothing here retries: otadata still points at
         * the running app, so the slot is dead weight and the rider is the
         * one who decides whether to try again (ADR-0006). */
        ESP_LOGE(TAG, "download: %s", esp_err_to_name(err));
        snprintf(r->detail, sizeof(r->detail), "stopped at %d%%",
                 wfota_dl_percent(&ic->dl));
        return OTAIN_ERR_FETCH;
    }
    if (status != 200) {
        snprintf(r->detail, sizeof(r->detail), "http %d", status);
        return OTAIN_ERR_FETCH;
    }

    /*
     * The digest, complete and compared, before anything makes this image
     * bootable. A body that got this far and does not hash to what the
     * manifest said is the one case a length check on its own would install.
     */
    wfota_dl_err_t derr = wfota_dl_end(&ic->dl);
    snprintf(r->sha256, sizeof(r->sha256), "%s", ic->dl.got_sha);
    if (derr == WFOTA_DL_ERR_SHORT) {
        snprintf(r->detail, sizeof(r->detail), "%u of %u bytes",
                 (unsigned)ic->dl.got, (unsigned)m->size);
        return OTAIN_ERR_SIZE;
    }
    if (derr != WFOTA_DL_OK) {
        ESP_LOGE(TAG, "sha256 %s, manifest says %s", ic->dl.got_sha, m->sha256);
        snprintf(r->detail, sizeof(r->detail), "got %.16s", ic->dl.got_sha);
        return OTAIN_ERR_SHA256;
    }

    /*
     * esp_ota_end() checks the image's own header and checksum, so a block
     * that reached flash wrong is caught here. That pairing is why the slot
     * is not read back and hashed a second time: the streamed digest proves
     * the bytes that arrived are the image, and this proves what is in the
     * slot is something the bootloader will load.
     */
    err = esp_ota_end(ic->handle);
    ic->open = false;               /* the handle is spent either way */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota end: %s", esp_err_to_name(err));
        /* The one failure here with an obvious next move, so the line is the
         * move and not the diagnosis: the bytes hashed right and the slot
         * still will not boot, which a second download is what to try. */
        snprintf(r->detail, sizeof(r->detail), "try the download again");
        return OTAIN_ERR_END;
    }

    /* The one irreversible line in this file, and the last one. */
    err = esp_ota_set_boot_partition(part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set boot partition: %s", esp_err_to_name(err));
        snprintf(r->detail, sizeof(r->detail), "otadata would not take it");
        return OTAIN_ERR_BOOT;
    }

    /*
     * Below the irreversible line on purpose: until otadata has taken the new
     * slot there is no trial to record, and after it there is one whether or
     * not this write works. So a failure here is logged and nothing more - the
     * install happened, and returning an error for it would be a lie that
     * costs the rider a reboot to find out about.
     */
    char      chan[WFOTA_CHANNEL_MAX + 1];
    esp_err_t terr;

    otaup_channel_get(chan, sizeof(chan));
    /* Only a channel there is something to come back from. Recording stable
     * too would cost nothing on the flash and a lie on the panel: a stable
     * update that rolls back would come up saying "back to stable" at a rider
     * who never left it, when what they need told is that the update failed.
     * Passing NULL also clears a trial an earlier debug install left behind. */
    terr = trial_store(strcmp(chan, WFOTA_CHANNEL_STABLE) == 0 ? NULL : chan);
    if (terr != ESP_OK) {
        ESP_LOGW(TAG, "trial channel not recorded: %s - a rollback will leave "
                      "the channel where it is", esp_err_to_name(terr));
    }

    ESP_LOGW(TAG, "%s installed into %s from the %s channel: the next boot "
                  "runs it on probation, and rolls back unless it passes the "
                  "health check",
             m->version, part->label, chan);
    return OTAIN_OK;
}

static int install_job(void *arg)
{
    install_ctx_t *ic  = arg;
    otain_err_t    err = install(ic);

    if (err != OTAIN_OK) {
        /* Abandoning the write is what leaves otadata pointing at the running
         * app: the slot keeps whatever half-image is in it, harmlessly, until
         * the next attempt erases it again. */
        if (ic->open) {
            esp_ota_abort(ic->handle);
            ic->open = false;
        }
        ESP_LOGE(TAG, "install: %s (%s)", otain_err_str(err),
                 ic->r->detail[0] ? ic->r->detail : "-");
        /* The link stays up, deliberately, where it used to come down with
         * the install (#41): the server is on it, and the rider whose
         * download was cut is the one most likely to want the settings page
         * next. Whether the mode keeps the link is the mode's to decide. */
    }
    return (int)err;
}

otain_err_t otaup_install(const wfota_manifest_t *m, otain_result_t *out,
                          otain_progress_fn progress)
{
    install_ctx_t ic;
    int           rc = 0;

    if (out == NULL || m == NULL) {
        return OTAIN_ERR_STATE;
    }
    memset(out, 0, sizeof(*out));
    memset(&ic, 0, sizeof(ic));
    ic.m         = m;
    ic.r         = out;
    ic.progress  = progress;
    ic.write_err = ESP_OK;

    if (!run_on_own_task("otainstall", install_job, &ic, &rc)) {
        snprintf(out->detail, sizeof(out->detail), "no task");
        return OTAIN_ERR_STATE;
    }
    return (otain_err_t)rc;
}

void otaup_stop(void)
{
    if (!s_running && !s_wifi_inited && s_netif == NULL && s_events == NULL) {
        return;                      /* never started, or stopped twice */
    }
    ESP_LOGI(TAG, "taking the station link down");
    teardown();
}

bool otaup_running(void)
{
    return s_running;
}

const char *otaup_err_str(otaup_err_t err)
{
    switch (err) {
    case OTAUP_OK:            return "ok";
    case OTAUP_ERR_STATE:     return "wrong link state for that";
    case OTAUP_ERR_WIFI:      return "wifi failed";
    case OTAUP_ERR_NO_NETS:   return "no networks stored";
    case OTAUP_ERR_NO_SCAN:   return "no access point in range";
    case OTAUP_ERR_NO_KNOWN:  return "no known network in range";
    case OTAUP_ERR_JOIN:      return "join failed";
    case OTAUP_ERR_FETCH:     return "fetch failed";
    case OTAUP_ERR_MANIFEST:  return "bad manifest";
    }
    return "unknown";
}

const char *otain_err_str(otain_err_t err)
{
    switch (err) {
    case OTAIN_OK:          return "ok";
    case OTAIN_ERR_STATE:   return "no link";
    case OTAIN_ERR_SLOT:    return "no slot for it";
    case OTAIN_ERR_BEGIN:   return "slot would not open";
    case OTAIN_ERR_FETCH:   return "download failed";
    case OTAIN_ERR_SIZE:    return "wrong size";
    case OTAIN_ERR_SHA256:  return "wrong sha256";
    case OTAIN_ERR_WRITE:   return "write refused";
    case OTAIN_ERR_END:     return "not a valid image";
    case OTAIN_ERR_BOOT:    return "otadata refused";
    }
    return "unknown";
}
