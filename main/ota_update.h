/*
 * ota_update - update mode's radio half: join a hotspot, read the manifest,
 * install the image.
 *
 * The Monitor stops listening, brings Wi-Fi up as a station, joins the
 * strongest of the networks it knows, fetches the four-field manifest from
 * the repository's newest release over HTTPS, and compares its `version` to
 * the running app's for inequality. If the rider then says so, it streams the
 * image the manifest names into the inactive app slot, checks its length and
 * its SHA-256 against the manifest, and only then points the bootloader at
 * it. Whether it stays pointed there is main/ota_health.c's question.
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
    /* The second line of whatever went wrong, and it goes straight onto the
     * message screen, so it is words a rider can act on and never an
     * esp_err_t name: "ESP_FAIL" under a headlamp is a dead end. The precise
     * error is logged where there is a keyboard to look it up with. */
    char             detail[40];
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

/* ---- the install --------------------------------------------------------
 *
 * The half that cannot be undone from a handlebar, so the order it does
 * things in is the whole design:
 *
 *   the inactive slot is chosen, never the running one;
 *   the body is length-checked as it arrives, so nothing past the end of the
 *     image is ever handed to esp_ota_write();
 *   the SHA-256 is complete and compared before esp_ota_end(), and the boot
 *     partition is switched only after both agree.
 *
 * Every failure before that last step leaves otadata alone, which is what
 * makes a dropped hotspot harmless: the half-written slot is dead weight, the
 * running app is still what boots, and the next attempt starts from zero.
 * Nothing retries by itself (ADR-0006).
 */
typedef enum {
    OTAIN_OK = 0,
    OTAIN_ERR_STATE,      /* no link up, or no manifest to install */
    OTAIN_ERR_SLOT,       /* no second app slot, or the image will not fit it */
    OTAIN_ERR_BEGIN,      /* esp_ota_begin() refused - the slot would not erase */
    OTAIN_ERR_FETCH,      /* TLS, an HTTP status, or the hotspot went away */
    OTAIN_ERR_SIZE,       /* the body was not the length the manifest promised */
    OTAIN_ERR_SHA256,     /* it was, and it is not the image named */
    OTAIN_ERR_WRITE,      /* esp_ota_write() refused a block */
    OTAIN_ERR_END,        /* esp_ota_end() refused: the slot holds no image */
    OTAIN_ERR_BOOT,       /* otadata would not take the new slot */
} otain_err_t;

typedef struct {
    char     slot[17];    /* the partition written - esp_partition_t's label */
    uint32_t written;
    char     sha256[WFOTA_SHA256_HEX + 1];  /* of what actually arrived */
    char     detail[40];   /* the rider's words, as above - never an esp_err_t */
} otain_result_t;

/* Called once per whole percent, on the install's own task. */
typedef void (*otain_progress_fn)(int percent, uint32_t got, uint32_t want);

/*
 * Downloads and installs the image `m` names, and blocks for the minute or so
 * that takes. Requires the station otaup_check() left joined.
 *
 * On OTAIN_OK the bootloader has been pointed at the new slot and nothing
 * else has happened: the caller reboots, and the new image comes up on
 * probation with main/ota_health.c's gates in front of it. It is deliberately
 * not this function that restarts, because the rider should see what it did
 * before the screen goes away.
 */
otain_err_t otaup_install(const wfota_manifest_t *m, otain_result_t *out,
                          otain_progress_fn progress);

const char *otain_err_str(otain_err_t err);

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
/* The URL the next check will read, channel and pin included. For `ota` on
 * the console, and for the settings page, where it is the thing that makes
 * "which stream is this Monitor on" answerable without a cable. */
bool      otaup_manifest_url(char *out, size_t cap);

/* ---- the channel --------------------------------------------------------
 *
 * Which stream of releases the Monitor follows, stored in the same namespace
 * as the pin and under the same rule: stable is the *absence* of the key, not
 * a value that spells it. That is what makes an erased NVS, an entry that
 * will not read back and a firmware built before channels existed all land on
 * the one stream every Monitor can always reach. A pin still outranks it,
 * because a pin is the deliberate one-off.
 */
/* NULL, "" or "stable" clears it. ESP_ERR_INVALID_ARG for a name that is not
 * in this build's table, so a channel with no URL behind it can never be
 * stored. */
esp_err_t otaup_channel_set(const char *name);
/* Always writes a usable name: "stable" when nothing is stored, when the
 * stored value will not read back, and when it is not one this firmware
 * knows. `cap` wants to be WFOTA_CHANNEL_MAX + 1. */
void      otaup_channel_get(char *out, size_t cap);

/* ---- the trial ----------------------------------------------------------
 *
 * The channel an image was installed from, written the moment the bootloader
 * is pointed at it and outstanding until that image earns its place. It is
 * how a boot after a rollback knows the failed update came from somewhere
 * other than stable, and it is erased either way - by ota_health.c when the
 * image is confirmed, and by the revert itself when it is not - so the
 * revert happens once and a rider who picks a channel again afterwards keeps
 * it.
 */
/* False when no install is outstanding, and `out` is then an empty string. */
bool otaup_trial_get(char *out, size_t cap);
/* Ends the trial. Idempotent, and a failure to write is only logged: an
 * uncleared trial costs one needless revert to stable and nothing else. */
void otaup_trial_clear(void);
