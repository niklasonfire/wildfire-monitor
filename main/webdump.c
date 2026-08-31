/*
 * webdump - Wi-Fi readout mode: SoftAP plus a small HTTP file server.
 *
 * The console runs at 115200 baud, which is about four minutes per megabyte;
 * a ride is several megabytes. So instead of shipping the capture out over the
 * serial link the board becomes an access point and serves the files over
 * HTTP, where the same capture takes seconds. Nothing here is reachable from
 * anywhere but that AP, which is WPA2 protected and lives for one readout
 * session: there is no authentication beyond the pre-shared key.
 *
 * BLE is already down when web_start() runs - the caller does that with
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

static httpd_handle_t s_server;
static esp_netif_t *s_netif;
static esp_event_handler_instance_t s_wifi_evt;
static bool s_wifi_inited;
static bool s_running;

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

/* ------------------------------------------------------------- HTTP: index */

static const char k_page_head[] =
    "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>wildfire captures</title><style>"
    "body{font-family:system-ui,sans-serif;margin:0;padding:16px;"
    "background:#111;color:#eee}"
    "h1{font-size:20px;margin:0 0 4px}"
    ".sub{color:#999;font-size:13px;margin:0 0 16px}"
    ".c{border:1px solid #333;border-radius:8px;padding:12px;margin:0 0 12px}"
    "a.d{display:block;font-size:19px;color:#6cf;text-decoration:none;"
    "padding:8px 0}"
    ".m{color:#aaa;font-size:13px;margin:2px 0 10px}"
    "button{background:#822;color:#fff;border:0;border-radius:6px;"
    "padding:12px 18px;font-size:15px}"
    ".e{color:#999}"
    "</style></head><body><h1>wildfire captures</h1>";

static const char k_page_tail[] = "</body></html>";

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

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    err = httpd_resp_sendstr_chunk(req, k_page_head);
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

static const httpd_uri_t k_uri_index = {
    .uri = "/", .method = HTTP_GET, .handler = index_get, .user_ctx = NULL,
};
static const httpd_uri_t k_uri_file = {
    .uri = "/f/*", .method = HTTP_GET, .handler = file_get, .user_ctx = NULL,
};
static const httpd_uri_t k_uri_rm = {
    .uri = "/rm/*", .method = HTTP_POST, .handler = rm_post, .user_ctx = NULL,
};

/* ------------------------------------------------------------------- wifi */

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
 * long before readout mode, so "already up" is the expected answer here. */
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

/* Undoes whatever web_start() managed to bring up, in reverse order. Every
 * step is guarded, so this doubles as the failure path and as web_stop(). */
static void web_teardown(void)
{
    if (s_server != NULL) {
        httpd_stop(s_server);
        s_server = NULL;
    }
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
    s_running = false;
}

/* -------------------------------------------------------------------- api */

esp_err_t web_start(char *out_ssid, size_t ssid_cap, char *out_ip, size_t ip_cap)
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
    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();

    if (s_running) {
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

    /* Readout mode is transient; there is no reason to write the config to
     * flash and wear it out. */
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

    hcfg.stack_size = 8192;          /* fread + the HTML formatting live here */
    hcfg.lru_purge_enable = true;    /* a phone that walks off must not wedge it */
    hcfg.max_open_sockets = 4;
    /* the file and delete routes are registered as wildcard patterns */
    hcfg.uri_match_fn = httpd_uri_match_wildcard;
    err = httpd_start(&s_server, &hcfg);
    if (err != ESP_OK) {
        s_server = NULL;
        goto fail;
    }
    err = httpd_register_uri_handler(s_server, &k_uri_index);
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &k_uri_file);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &k_uri_rm);
    }
    if (err != ESP_OK) {
        goto fail;
    }

    if (out_ssid != NULL && ssid_cap > 0) {
        snprintf(out_ssid, ssid_cap, "%s", ssid);
    }
    if (out_ip != NULL && ip_cap > 0) {
        snprintf(out_ip, ip_cap, "%s", ipstr);
    }

    s_running = true;
    ESP_LOGI(TAG, "AP \"%s\" up on %s, password \"%s\", %d capture(s)",
             ssid, ipstr, WEB_PASSWORD, store_ready() ? store_count() : -1);
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "start failed: %s", esp_err_to_name(err));
    web_teardown();
    if (out_ssid != NULL && ssid_cap > 0) {
        out_ssid[0] = '\0';
    }
    if (out_ip != NULL && ip_cap > 0) {
        out_ip[0] = '\0';
    }
    return err;
}

void web_stop(void)
{
    if (!s_running && s_server == NULL && !s_wifi_inited && s_netif == NULL) {
        return;                      /* never started, or stopped twice */
    }
    ESP_LOGI(TAG, "stopping readout mode");
    web_teardown();
}

bool web_running(void)
{
    return s_running;
}

int web_clients(void)
{
    wifi_sta_list_t list = {0};

    if (!s_running) {
        return 0;
    }
    if (esp_wifi_ap_get_sta_list(&list) != ESP_OK) {
        return 0;
    }
    return list.num;
}
