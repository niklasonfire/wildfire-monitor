/* ota_update - see ota_update.h. */
#include "ota_update.h"

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
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "otaup";

#define OTAUP_NS       "ota"
#define OTAUP_PIN_KEY  "pin"

/* More access points than a car park has, and the list is on the stack of the
 * task below; past this the extra ones are the faint ones anyway. */
#define SCAN_MAX        24
#define JOIN_TIMEOUT_MS 20000
#define HTTP_TIMEOUT_MS 15000
/*
 * The check runs on a task of its own rather than on whichever task asked for
 * it. The TLS handshake wants several kilobytes of stack on top of the HTTP
 * client's own, and both callers - the button task at 4 KB and the console
 * REPL - have their stacks sized for something else entirely. A task with a
 * stack this file chooses is the only way that stays true when either caller
 * is resized.
 */
#define CHECK_STACK     10240
#define CHECK_PRIO      3

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

bool otaup_manifest_url(char *out, size_t cap)
{
    char pin[WFOTA_VERSION_MAX + 1];

    (void)otaup_pin_get(pin, sizeof(pin));   /* empty when nothing is pinned */
    return wfota_manifest_url(pin, out, cap);
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
 * long before update mode, so "already up" is the expected answer. Same
 * shape as webdump.c, which is the other mode that owns this radio. */
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

    /* Update mode is transient and the credentials already live in the `wifi`
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
    /* GitHub answers the release URL with a redirect to its object store, and
     * that redirect has a short body of its own. Only the body of the 200 is
     * the manifest, so the status decides what is collected. */
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
        /* Sized for GitHub's headers, not for the manifest. The redirect from
         * github.com carries a Content-Security-Policy line of about 3.6 KB
         * on its own, and CONFIG_ESP_HTTP_CLIENT_STRICT_HEADER_BUFFER makes a
         * header that does not fit an error rather than a truncation. The
         * buffer is freed with the client, a few seconds later. */
        .buffer_size           = 6144,
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
        snprintf(r->detail, sizeof(r->detail), "%s", esp_err_to_name(err));
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

/* ----------------------------------------------------------- the whole check */

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
        snprintf(r->detail, sizeof(r->detail), "%s", esp_err_to_name(err));
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

static otaup_err_t run_check(otaup_result_t *r, otaup_progress_fn progress)
{
    wifi_net_t   nets[WIFI_STORE_MAX];
    const char  *known[WIFI_STORE_MAX];
    wfota_seen_t seen[SCAN_MAX];

    const esp_app_desc_t *desc = esp_app_get_description();
    snprintf(r->running, sizeof(r->running), "%s",
             (desc != NULL) ? desc->version : "");

    int n_nets = wifi_store_load(nets, WIFI_STORE_MAX);
    if (n_nets == 0) {
        /* Nothing to scan for. Said before the radio comes up at all, because
         * the answer does not depend on what is in range. */
        return OTAUP_ERR_NO_NETS;
    }
    for (int i = 0; i < n_nets; i++) {
        known[i] = nets[i].ssid;
    }

    say(progress, "starting", "wifi");
    esp_err_t err = wifi_up();
    if (err != ESP_OK) {
        snprintf(r->detail, sizeof(r->detail), "%s", esp_err_to_name(err));
        return OTAUP_ERR_WIFI;
    }
    s_running = true;

    say(progress, "scanning", NULL);
    wifi_scan_config_t sc = {0};
    /* Active, all channels: a phone hotspot sits on whichever channel the
     * phone picked, and there is nothing else for this radio to do. */
    err = esp_wifi_scan_start(&sc, true);
    if (err != ESP_OK) {
        snprintf(r->detail, sizeof(r->detail), "%s", esp_err_to_name(err));
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
    ESP_LOGI(TAG, "joined \"%s\", address %s", r->ssid, r->ip);

    say(progress, "reading", r->ip);
    return fetch_manifest(r);
}

typedef struct {
    otaup_result_t   *result;
    otaup_progress_fn progress;
    otaup_err_t       err;
    SemaphoreHandle_t done;
} check_ctx_t;

static void check_task(void *arg)
{
    check_ctx_t *ctx = arg;

    ctx->err = run_check(ctx->result, ctx->progress);
    if (ctx->err != OTAUP_OK) {
        /* Left failed, and left tidy: the radio goes back down so the failure
         * costs no heap while the rider reads the screen. */
        teardown();
    }
    xSemaphoreGive(ctx->done);
    vTaskDelete(NULL);
}

otaup_err_t otaup_check(otaup_result_t *out, otaup_progress_fn progress)
{
    check_ctx_t ctx = {0};

    if (out == NULL) {
        return OTAUP_ERR_STATE;
    }
    memset(out, 0, sizeof(*out));
    if (s_running) {
        return OTAUP_ERR_STATE;
    }

    ctx.result   = out;
    ctx.progress = progress;
    ctx.err      = OTAUP_ERR_STATE;
    ctx.done     = xSemaphoreCreateBinary();
    if (ctx.done == NULL) {
        return OTAUP_ERR_WIFI;
    }
    if (xTaskCreate(check_task, "otacheck", CHECK_STACK, &ctx, CHECK_PRIO,
                    NULL) != pdPASS) {
        vSemaphoreDelete(ctx.done);
        return OTAUP_ERR_WIFI;
    }
    /* No timeout: every step inside has one of its own, and ctx lives on this
     * stack, so returning while the task still holds it is not an option. */
    xSemaphoreTake(ctx.done, portMAX_DELAY);
    vSemaphoreDelete(ctx.done);
    return ctx.err;
}

void otaup_stop(void)
{
    if (!s_running && !s_wifi_inited && s_netif == NULL && s_events == NULL) {
        return;                      /* never started, or stopped twice */
    }
    ESP_LOGI(TAG, "leaving update mode");
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
    case OTAUP_ERR_STATE:     return "already running";
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
