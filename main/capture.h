/*
 * capture - the standalone, button-driven capture on the bike.
 *
 * Runs the whole session without a host: scan until both the Fardriver
 * controller and the Daly BMS have been seen, wait for the user to arm it,
 * hold both links at once, subscribe to their notify characteristics and
 * write every frame into capture_store with a timestamp.
 *
 * It talks to NimBLE directly rather than through ble_explorer, because the
 * explorer is a single-connection, host-driven tool and this needs two links
 * kept up across the dropouts of a ride. The two are mutually exclusive: the
 * capture refuses to start while the explorer holds a connection or a scan.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    CAP_IDLE = 0,     /* radio quiet, nothing to do */
    CAP_SCANNING,     /* looking for the two devices */
    CAP_ARMED,        /* both seen, waiting for the button */
    CAP_CONNECTING,   /* connecting and subscribing */
    CAP_RECORDING,    /* frames are going to flash */
    CAP_STOPPING,     /* closing the file */
    CAP_DONE,         /* last capture finished, summary on screen */
    CAP_ERROR,
} cap_state_t;

/* Which of the two peers a status block describes. */
typedef enum { CAP_LINK_MCU = 0, CAP_LINK_BMS = 1, CAP_LINK_COUNT } cap_link_t;

typedef struct {
    bool     seen;          /* found by the scan */
    bool     connected;
    bool     subscribed;    /* CCCD written, frames expected */
    char     name[24];
    char     addr[18];
    int8_t   rssi;
    uint32_t frames;
    uint32_t bytes;
    uint32_t reconnects;
    uint32_t last_frame_ms; /* capture time of the most recent frame */
} cap_link_status_t;

typedef struct {
    cap_state_t       state;
    cap_link_status_t link[CAP_LINK_COUNT];
    uint32_t          elapsed_ms;   /* since recording started */
    uint32_t          frames;       /* both links */
    uint32_t          dropped;
    uint64_t          bytes;        /* written to the capture file */
    int               seq;          /* capture number, -1 when none */
    char              file[32];
    char              err[48];      /* last error, empty when fine */
} cap_status_t;

/* Brings up the capture task. Does not start scanning. */
esp_err_t cap_init(void);

esp_err_t cap_scan_start(void);
esp_err_t cap_scan_stop(void);
/* Only legal in CAP_ARMED: connects both links and opens the capture file. */
esp_err_t cap_record_start(void);
/* Closes the file and drops both links. Safe to call in any state. */
esp_err_t cap_record_stop(void);

/* Writes a marker into the running capture. This is how a ride becomes
 * interpretable: the rider does a defined manoeuvre - full throttle, coast,
 * brake - and marks it, so offline analysis has labelled sections instead of
 * an undifferentiated stream. No-op unless recording. */
esp_err_t   cap_marker(const char *text);

void        cap_status(cap_status_t *out);
cap_state_t cap_state(void);
const char *cap_state_str(cap_state_t s);

/* Stops everything and takes NimBLE and the BT controller down so the Wi-Fi
 * readout mode has the RAM. The only way back to capturing is a reboot. */
esp_err_t cap_ble_shutdown(void);
bool      cap_ble_down(void);
