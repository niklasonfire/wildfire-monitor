/*
 * wifi_store - the networks update mode is allowed to join.
 *
 * A list from the first commit, and not a single network (ADR-0006). Only the
 * console writes to it today, but a rider with a second phone must not be a
 * data-model change, and a provisioning screen later should find a way in
 * here rather than start a second store.
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
