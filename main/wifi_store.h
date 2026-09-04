/*
 * wifi_store - the networks the Monitor is allowed to join.
 *
 * A list from the first commit, and not a single network (ADR-0006). A rider
 * with a second phone must not be a data-model change, and this comment used
 * to say that a provisioning screen would one day have to find a way in here
 * rather than start a second store. It did: the settings page webdump.c
 * serves writes through wifi_store_add() and wifi_store_del() and holds
 * nothing of its own, so it and the console agree by construction.
 *
 * That page is now the way in that always exists (#41). The Monitor has one
 * Wi-Fi mode, it serves the page over the access point it falls back to just
 * as readily as over a network it joined, and so a list that has gone
 * wrong - a rotated key, an SSID typed with a capital in the wrong place - is
 * edited from the failure it caused rather than over the cable ADR-0006
 * exists to avoid.
 *
 * The passphrases sit in NVS unencrypted, deliberately: encryption on this
 * board would only move the key next to the ciphertext, and what is stored is
 * a phone hotspot key that can be rotated on the phone. Nothing else in the
 * firmware may put a credential here, and nothing in the repository may carry
 * one - it is public, and these arrive from the console at runtime.
 */
#pragma once

#include "esp_err.h"

#include "wfota.h"

/* Four is the ADR's number: home, the phone, a spare and one to be wrong
 * about. It also keeps the whole list on the stack of the task that scans. */
#define WIFI_STORE_MAX  4
/* A WPA2 passphrase, which is 8..63 characters; an empty one means an open
 * network, which a phone hotspot never is but a workshop's Wi-Fi might be. */
#define WIFI_PASS_MIN   8
#define WIFI_PASS_MAX   63

typedef struct {
    char ssid[WFOTA_SSID_MAX + 1];
    char pass[WIFI_PASS_MAX + 1];
} wifi_net_t;

/* Reads the list, newest last. Returns how many were written into `out`, or
 * 0 when the namespace has never been written - which is the normal state of
 * a Monitor nobody has run `wifi add` on. */
int wifi_store_load(wifi_net_t *out, int cap);

/* Adds a network, or replaces the passphrase of one already stored under the
 * same SSID. ESP_ERR_INVALID_SIZE for an SSID or passphrase that cannot be
 * used, ESP_ERR_NO_MEM when the list is full. */
esp_err_t wifi_store_add(const char *ssid, const char *pass);

/* Forgets one network. ESP_ERR_NOT_FOUND if it was not stored. */
esp_err_t wifi_store_del(const char *ssid);
