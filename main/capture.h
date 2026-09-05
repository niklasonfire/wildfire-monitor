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
#include "wfdecode/wfdecode.h"
#include "wfest/wfest.h"

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
    /* How long this link has been silent, in milliseconds of uptime, and
     * whether it has ever spoken at all. Both are tracked whether or not a
     * capture is running, because what they exist for is telling a live
     * reading from a remembered one: every field wfdecode keeps latches its
     * validity flag the first time the frame type carrying it arrives and
     * never clears it, so a Controller that has gone off the air leaves the
     * last current it reported standing on the screen looking exactly like
     * the current flowing now. `quiet_ms` is what says otherwise. It is zero
     * while `ever_rx` is false, which is the "nothing has arrived" case and
     * not a fresh one. */
    bool     ever_rx;
    uint32_t quiet_ms;
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

/* Snapshot of the Controller fields wfdecode keeps up to date; see
 * wf_ctrl_live_t. Decoded from every MCU-link frame regardless of whether a
 * capture is running - the live screen has to work while just riding, not only
 * mid-capture. */
void cap_live_get(wf_ctrl_live_t *out);

/* Snapshot of what main/wfest makes of those fields: Remaining Energy, the
 * Coulomb Count and how it stands against its Anchor. Fed from the same two
 * notification streams as cap_live_get(), capture running or not, and every
 * figure in it is computed in the estimator - callers format it and do no
 * arithmetic of their own. */
void cap_est_get(wf_est_out_t *out);

/* ------------------------------------------------------------------- tuning
 *
 * The handful of numbers the link handling is built on, readable and writable
 * at runtime.
 *
 * Issue #34: the BMS answers exactly 27 polls per connection and then goes
 * silent, on a cycle that repeats every ~40.7 s. Four explanations are still
 * standing - the peer's answer path wedging, our receive path choking on the
 * fragmented answers, our requests never leaving the radio at all, or a timer
 * inside the peer - and the Captures taken so far separate none of them. What
 * separates them is a matrix of runs that differ in exactly one thing each,
 * and reflashing between runs would change more than that one thing.
 *
 * So these are knobs on the bench rather than constants in the binary, and
 * every default reproduces today's behaviour exactly: nothing in here changes
 * what a ride does until somebody at the console asks it to. The effective
 * tune is written into the Capture when it starts, because a file that does
 * not say which run it is cannot be compared against another one.
 */
typedef struct {
    uint32_t poll_ms;    /* poll period of a polled link */
    uint16_t poll_regs;  /* 0 = the link keeps its own width logic, else this */
    uint32_t stale_ms;   /* silence after which a subscribed link is rebuilt */
    uint8_t  miss_probe; /* unanswered polls before the stall probe, 0 = never */
    bool     probe;      /* run the stall probe at all */
    bool     mcu_off;    /* leave the Controller link down for the whole run */
    uint16_t itvl;       /* 0 = accept the peer's request, else force this (1.25 ms units) */
} cap_tune_t;

void cap_tune_get(cap_tune_t *out);

/* Refused with ESP_ERR_INVALID_STATE unless the capture is idle. A knob moved
 * mid-run would change what the numbers already in the open file mean, and a
 * Capture that cannot be read back as one experiment is worth less than no
 * Capture at all. poll_regs is clamped into 1..WF_BMS_MAX_REGS; a period or a
 * timeout the link handling could not act on is refused with
 * ESP_ERR_INVALID_ARG rather than quietly rounded. */
esp_err_t cap_tune_set(const cap_tune_t *in);

/* Stops everything and takes NimBLE and the BT controller down so the Wi-Fi
 * readout mode has the RAM. The only way back to capturing is a reboot. */
esp_err_t cap_ble_shutdown(void);
bool      cap_ble_down(void);
