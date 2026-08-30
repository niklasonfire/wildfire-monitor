/* est_store - see est_store.h. The NVS half of the estimation seam. */
#include "est_store.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "est_store";

#define EST_NS   "wfest"
#define EST_KEY  "state"

bool est_store_load(wf_est_persist_t *out)
{
    if (out == NULL) {
        return false;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(EST_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        /* No namespace yet is the normal state of a Monitor that has never
         * finished a capture. */
        return false;
    }

    uint8_t blob[WF_EST_PERSIST_BYTES];
    size_t len = sizeof(blob);
    err = nvs_get_blob(h, EST_KEY, blob, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        return false;
    }

    /* The estimator decides whether these bytes are its own: magic, version
     * and CRC are its business, not this file's. */
    if (!wf_est_persist_decode(blob, len, out)) {
        ESP_LOGW(TAG, "saved state is not ours, starting cold");
        return false;
    }
    ESP_LOGI(TAG, "restored %.2f Ah / %.0f Wh", (double)out->coulomb_ah,
             (double)out->remaining_wh);
    return true;
}

esp_err_t est_store_save(const wf_est_persist_t *p)
{
    if (p == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t blob[WF_EST_PERSIST_BYTES];
    if (!wf_est_persist_encode(p, blob, sizeof(blob))) {
        return ESP_ERR_INVALID_SIZE;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(EST_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_blob(h, EST_KEY, blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "save: %s", esp_err_to_name(err));
    }
    return err;
}
