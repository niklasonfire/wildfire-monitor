/*
 * webdump - the pages the Monitor serves, and the access point it falls back
 * to when it cannot join anything.
 *
 * The console runs at 115200 baud, which is about four minutes per megabyte,
 * and a ride is several megabytes; so a Capture leaves the board over HTTP
 * instead, where the same file takes seconds. The same server carries the
 * settings page - the networks the Monitor may join, the update channel, and
 * the update itself - because a phone already holding the Capture listing is
 * the one place on this bike a rider can type.
 *
 * This file is two halves that used to be one function, and separating them
 * is the whole of what let the two old Wi-Fi modes become Service Mode (#41):
 * the pages are now served over a hotspot the Monitor joined just as readily
 * as over an access point of its own.
 *
 *   web_ap_start()    owns a link: the SoftAP, its netif and its DHCP server.
 *   web_serve_start() owns the httpd and its handlers, and knows nothing
 *                     whatever about the link underneath it.
 *
 * main.c puts the two together, because which link to be on is the mode's
 * decision and not the server's. Over the access point the server answers on
 * 192.168.4.1 behind the WPA2 key below, and there is no authentication
 * beyond it; over a joined network it answers on whatever address DHCP gave
 * the Monitor, where anyone else on that network can reach it too. Neither is
 * guarded further. What is served is a Capture of a bike and a list of SSIDs -
 * passphrases go into the settings page and are never rendered back out, so
 * what being in range buys is the ability to change that list rather than to
 * read it - and, since the update moved onto the page, the ability to install
 * a release. That last one is bounded rather than guarded: the image is named
 * by a manifest fetched over TLS from a public GitHub release and checked by
 * length and SHA-256 before the bootloader is pointed at it, so what reaching
 * this server buys is forcing a *published* build, forward or back, and not
 * running one of somebody's own. ADR-0006's amendment says what that costs.
 *
 * BLE is already down before either half starts - the mode does that with
 * cap_ble_shutdown() - because NimBLE and Wi-Fi do not fit in RAM together on
 * this chip. That is also why the handlers keep almost nothing static and
 * borrow their buffers from the heap only for the length of a response.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#include "wfota.h"      /* WFOTA_VERSION_MAX, for the update status below */

#define WEB_SSID_PREFIX "wildfire-"
#define WEB_PASSWORD    "wildfire"   /* WPA2 needs 8 characters */

/* ---- the fallback link --------------------------------------------------
 *
 * Brings up the SoftAP and nothing else. The caller must have taken BLE down
 * (cap_ble_shutdown()) first, and must not already hold the radio as a
 * station - one link at a time, which is main.c's rule to keep. Fills in the
 * SSID and the address the rider has to read off the LCD.
 */
esp_err_t web_ap_start(char *out_ssid, size_t ssid_cap, char *out_ip, size_t ip_cap);
void      web_ap_stop(void);
bool      web_ap_running(void);

/* ---- the server ---------------------------------------------------------
 *
 * Starts the httpd over whichever link is already up. It binds to every
 * address the Monitor has, so this is the same call for the access point and
 * for a joined network; the mode is what knows which of those it is.
 */
esp_err_t web_serve_start(void);
void      web_serve_stop(void);
bool      web_running(void);

/* Stations associated to our own access point, for the console. Zero when the
 * Monitor is a station itself: nothing is associated *to* it then, and the
 * question a rider would be asking - is anything talking to me - is one the
 * hotspot's owner has to answer, not this. */
int       web_clients(void);

/* ---- the update, as the page sees it ------------------------------------
 *
 * The settings page carries the update now - a button that reads the manifest
 * and a second one that installs what it found - and the mode no longer
 * checks on its own way in. What the server does *not* carry is any of the
 * work. Two things forbid that, and both of them hang rather than fail:
 *
 *   the install stops the httpd before the transfer, to leave TLS, the image
 *     buffer and the OTA write the heap they need, and a handler that stops
 *     the server is a handler stopping the task it is running on;
 *   the handler stack is 8 KB, sized for fread and HTML, and a manifest read
 *     is a TLS handshake plus an HTTP client on top of that.
 *
 * So the two routes below post a request and answer at once. The mode owns a
 * worker that does the work and leaves its answer here, and the page reads
 * that answer on the next GET - which is why a check is two page loads and
 * not one. Nothing is registered until Service Mode is up, and until then the
 * page says there is no update to be had rather than showing a button that
 * leads nowhere.
 */
typedef enum {
    WEB_UPD_IDLE = 0,    /* nothing has been asked since the mode came up */
    WEB_UPD_BUSY,        /* the worker has the manifest question in hand */
    WEB_UPD_CURRENT,     /* it answered: the running version is the published one */
    WEB_UPD_OFFER,       /* it answered: another version is on offer */
    WEB_UPD_INSTALLING,  /* the image is coming down and the server is going away */
    WEB_UPD_FAILED,      /* the check or the install did not finish */
} web_upd_state_t;

typedef struct {
    web_upd_state_t state;
    /* The version this is about, or the words for what went wrong - the same
     * rider's words the panel used to carry. `text` can be a version a
     * manifest sent, so the page escapes it before printing it; `detail` is
     * the second line the check or the install collected, and is empty when
     * there was none. */
    char            text[WFOTA_VERSION_MAX + 1];
    char            detail[40];
} web_upd_status_t;

/*
 * What the mode lends the server. `check` and `install` return false when the
 * worker is already busy or could not be started - the page says so rather
 * than queueing a second request behind the first - and `install` also
 * refuses when no check has put anything on offer. `status` fills `out` for
 * whatever the last request left behind.
 */
typedef struct {
    bool (*check)(void);
    bool (*install)(void);
    void (*status)(web_upd_status_t *out);
} web_update_ops_t;

/* Registered by the mode on its way up and never taken back: the ops point at
 * statics that outlive every request. NULL puts the page back to offering
 * nothing, which is also where it starts. */
void web_update_ops(const web_update_ops_t *ops);
