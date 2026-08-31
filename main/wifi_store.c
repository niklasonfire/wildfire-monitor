/* wifi_store - see wifi_store.h. */
#include "wifi_store.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_store";

#define WIFI_NS   "wifi"
#define WIFI_KEY  "nets"

/*
 * One blob holding the whole list rather than a key per network: add and
 * delete are then a read, an edit and a write, with no way to leave a hole in
 * the middle or two keys disagreeing about how many there are. The count is
 * the blob's length divided by the entry size - wifi_net_t is nothing but
 * char arrays, so it has no padding and no alignment to argue about - and a
 * length that is not a whole number of entries is a blob this file did not
 * write, which is treated as no list at all.
 */
static int load(wifi_net_t *nets)
{
    nvs_handle_t h;
    size_t       len = WIFI_STORE_MAX * sizeof(wifi_net_t);

    memset(nets, 0, len);
    if (nvs_open(WIFI_NS, NVS_READONLY, &h) != ESP_OK) {
        return 0;                   /* nothing has ever been added */
    }
    esp_err_t err = nvs_get_blob(h, WIFI_KEY, nets, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        return 0;
    }
    if (len == 0 || (len % sizeof(wifi_net_t)) != 0) {
        ESP_LOGW(TAG, "the stored list is %u bytes, which is not a list of "
                      "networks - ignoring it", (unsigned)len);
        return 0;
    }

    int n = (int)(len / sizeof(wifi_net_t));
    /* Both strings come back out of flash and go straight into an SSID and a
     * passphrase field, so terminate them here rather than trust the blob. */
    for (int i = 0; i < n; i++) {
        nets[i].ssid[WFOTA_SSID_MAX] = '\0';
        nets[i].pass[WIFI_PASS_MAX] = '\0';
    }
    return n;
}

static esp_err_t save(const wifi_net_t *nets, int n)
{
    nvs_handle_t h;
    esp_err_t    err = nvs_open(WIFI_NS, NVS_READWRITE, &h);

    if (err != ESP_OK) {
        return err;
    }
    if (n == 0) {
        /* The last network deleted leaves no key, so the next load() takes
         * the "never written" path instead of reading a zero-length blob. */
        err = nvs_erase_key(h, WIFI_KEY);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
    } else {
        err = nvs_set_blob(h, WIFI_KEY, nets, (size_t)n * sizeof(wifi_net_t));
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

int wifi_store_load(wifi_net_t *out, int cap)
{
    wifi_net_t nets[WIFI_STORE_MAX];

    if (out == NULL || cap <= 0) {
        return 0;
    }
    int n = load(nets);
    if (n > cap) {
        n = cap;
    }
    memcpy(out, nets, (size_t)n * sizeof(nets[0]));
    return n;
}

esp_err_t wifi_store_add(const char *ssid, const char *pass)
{
    wifi_net_t nets[WIFI_STORE_MAX];

    if (ssid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pass == NULL) {
        pass = "";
    }
    size_t slen = strlen(ssid);
    size_t plen = strlen(pass);
    if (slen == 0 || slen > WFOTA_SSID_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    /* WPA2 will not take a shorter key, so a typo is worth catching here
     * rather than at a hotspot in a car park. */
    if (plen != 0 && (plen < WIFI_PASS_MIN || plen > WIFI_PASS_MAX)) {
        return ESP_ERR_INVALID_SIZE;
    }

    int n = load(nets);
    for (int i = 0; i < n; i++) {
        if (strcmp(nets[i].ssid, ssid) == 0) {
            /* Same network, new key: the rotation the ADR expects a phone to
             * do, and the one case where adding is not a new entry. */
            snprintf(nets[i].pass, sizeof(nets[i].pass), "%s", pass);
            ESP_LOGI(TAG, "replaced the passphrase for \"%s\"", ssid);
            return save(nets, n);
        }
    }
    if (n >= WIFI_STORE_MAX) {
        return ESP_ERR_NO_MEM;
    }
    memset(&nets[n], 0, sizeof(nets[n]));
    snprintf(nets[n].ssid, sizeof(nets[n].ssid), "%s", ssid);
    snprintf(nets[n].pass, sizeof(nets[n].pass), "%s", pass);
    ESP_LOGI(TAG, "added \"%s\" (%d of %d)", ssid, n + 1, WIFI_STORE_MAX);
    return save(nets, n + 1);
}

esp_err_t wifi_store_del(const char *ssid)
{
    wifi_net_t nets[WIFI_STORE_MAX];

    if (ssid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    int n = load(nets);
    for (int i = 0; i < n; i++) {
        if (strcmp(nets[i].ssid, ssid) != 0) {
            continue;
        }
        for (int j = i; j + 1 < n; j++) {
            nets[j] = nets[j + 1];
        }
        ESP_LOGI(TAG, "forgot \"%s\"", ssid);
        return save(nets, n - 1);
    }
    return ESP_ERR_NOT_FOUND;
}
