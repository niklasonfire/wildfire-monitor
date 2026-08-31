/*
 * ota_update - update mode's radio half: join a hotspot, read the manifest.
 *
 * The Monitor stops listening, brings Wi-Fi up as a station, joins the
 * strongest of the networks it knows, fetches the four-field manifest from
 * the repository's newest release over HTTPS, and compares its `version` to
 * the running app's for inequality. Nothing is downloaded and nothing is
 * written to flash beyond the Wi-Fi list and the pin: installing the image is
 * issue #28.
 *
 * The shape is readout mode's (ADR-0006). Wi-Fi and NimBLE do not fit in this
 * chip's RAM together, so the caller takes BLE down first and the only way
 * back to capturing is a reboot. A failure is left failed: it produces a
 * message and a return to the menu, and nothing retries on its own, because
 * the rider is standing next to the Monitor and an automatic retry would only
 * hide a hotspot that is too weak to finish.
 *
 * Trust is the ESP-IDF certificate bundle and not a pinned certificate:
 * GitHub rotates its certificates, and a pinned one would eventually stop
 * every update with no way back except the cable this design exists to avoid.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#include "wfota.h"

/* Each one is a different thing to tell the rider, which is why they are not
 * one "it did not work". */
typedef enum {
    OTAUP_OK = 0,
    OTAUP_ERR_STATE,      /* update mode is already up */
    OTAUP_ERR_WIFI,       /* the radio would not start */
    OTAUP_ERR_NO_NETS,    /* nothing has ever been added with `wifi add` */
    OTAUP_ERR_NO_SCAN,    /* the scan saw no access point at all */
    OTAUP_ERR_NO_KNOWN,   /* it saw some, and none of them is one of ours */
    OTAUP_ERR_JOIN,       /* association or DHCP failed */
    OTAUP_ERR_FETCH,      /* TLS, HTTP, or a status that was not 200 */
    OTAUP_ERR_MANIFEST,   /* what came back is not the four fields */
} otaup_err_t;

typedef struct {
    char             ssid[WFOTA_SSID_MAX + 1];   /* the network joined */
    int              rssi;
    char             ip[16];                     /* the address it was given */
    char             running[WFOTA_VERSION_MAX + 1];  /* the app's own version */
    wfota_manifest_t manifest;
    bool             have_manifest;
    bool             differs;    /* the manifest names another version */
    char             detail[40]; /* second line of whatever went wrong */
} otaup_result_t;

/* Called as the check moves on, so the rider sees a screen change rather than
 * a frozen one. Runs on the check's own task; either line may be NULL. */
typedef void (*otaup_progress_fn)(const char *line1, const char *line2);

/*
 * Runs the whole check and blocks until it is over, about ten seconds when
 * everything works. On OTAUP_OK the station is left joined - the address in
 * `out` is live, and #28 will download through it - so the caller must call
 * otaup_stop() when the rider is done with the screen. Every failure has
 * already torn the radio back down.
 */
otaup_err_t otaup_check(otaup_result_t *out, otaup_progress_fn progress);

/* Takes the station back down. Safe to call when nothing came up. */
void otaup_stop(void);
bool otaup_running(void);

/* A word for the console and the log, not a sentence for the panel. */
const char *otaup_err_str(otaup_err_t err);

/* ---- the pin ------------------------------------------------------------
 *
 * Unpinned, the Monitor reads `releases/latest/download/manifest.json`, the
 * redirect GitHub keeps pointing at the newest release. Pinned, it reads that
 * one tag's manifest instead - which, because versions are compared for
 * inequality and never for order, is how a suspect release is backed out
 * without publishing anything.
 */
esp_err_t otaup_pin_set(const char *tag);   /* NULL or "" clears it */
/* False when nothing is pinned, and `out` is then an empty string. */
bool      otaup_pin_get(char *out, size_t cap);
/* The URL the next check will read, pin included. For `ota` on the console. */
bool      otaup_manifest_url(char *out, size_t cap);
