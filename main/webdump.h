/*
 * webdump - the pages the Monitor serves, and the access point it falls back
 * to when it cannot join anything.
 *
 * The console runs at 115200 baud, which is about four minutes per megabyte,
 * and a ride is several megabytes; so a Capture leaves the board over HTTP
 * instead, where the same file takes seconds. The same server carries the
 * settings page - the networks the Monitor may join, the update channel, the
 * pin - because a phone already holding the Capture listing is the one place
 * on this bike a rider can type.
 *
 * This file is two halves that used to be one function, and separating them
 * is the whole of what let the two Wi-Fi modes become one (#41): the pages are
 * now served over a hotspot the Monitor joined just as readily as over an
 * access point of its own.
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
 * guarded further, because what is served is a Capture of a bike and a list
 * of SSIDs: passphrases go into the settings page and are never rendered back
 * out, so what being in range buys is the ability to change that list rather
 * than to read it.
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
