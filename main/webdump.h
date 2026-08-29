/*
 * webdump - Wi-Fi readout mode.
 *
 * Pulling a capture off the board over the 115200 baud console takes minutes
 * per megabyte, so the board can instead put up its own access point and serve
 * the capture files over HTTP: connect a phone or a laptop, open the page,
 * download. BLE has to be down before this starts - the two do not fit in RAM
 * together - so the only way back to capturing is a reboot.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#define WEB_SSID_PREFIX "wildfire-"
#define WEB_PASSWORD    "wildfire"   /* WPA2 needs 8 characters */

/* Brings up the AP and the HTTP server. The caller must have taken BLE down
 * (cap_ble_shutdown()) first. Fills in what the UI has to show. */
esp_err_t web_start(char *out_ssid, size_t ssid_cap, char *out_ip, size_t ip_cap);
void      web_stop(void);
bool      web_running(void);
/* Number of stations currently associated, for the readout screen. */
int       web_clients(void);
