/*
 * ota_health - whether the running firmware has earned its place.
 *
 * With the bootloader's rollback enabled (ADR-0006) an image written into the
 * spare app slot boots on probation: unless it tells the bootloader it is
 * well, the next reset goes back to the slot that worked. This is the thing
 * that decides, and what it asks for is deliberately cheap:
 *
 *   NVS opens, the capture store mounts, the display draws one frame, and
 *   sixty seconds of uptime pass.
 *
 * No Controller and no BMS link is required, and that is not an oversight.
 * Updates happen with the bike switched off, so a healthy firmware that rolled
 * itself back for want of a Pack would be worse than no rollback at all.
 *
 * An image that is not on probation - one flashed over USB, one already
 * confirmed - still runs the whole check and still reports it. Keeping the two
 * cases on one path is what makes the check verifiable on the bench, where the
 * only images that exist arrived down a cable.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* The three things that have to happen, each reported once by whichever
 * subsystem is the only one that can know it did. */
typedef enum {
    OTA_GATE_NVS     = 1u << 0,   /* nvs_flash_init() returned ESP_OK */
    OTA_GATE_STORE   = 1u << 1,   /* the capture store mounted */
    OTA_GATE_DISPLAY = 1u << 2,   /* the redraw task painted a frame */
} ota_gate_t;

#define OTA_GATES_ALL       (OTA_GATE_NVS | OTA_GATE_STORE | OTA_GATE_DISPLAY)
/* The fourth condition, which is not a gate anybody passes: it is just time.
 * Long enough for a boot loop to have shown itself, short enough that the
 * rider is still standing next to the Monitor when it is over. */
#define OTA_HEALTH_UPTIME_S 60

/* Samples the rollback state of the running image and starts the task that
 * watches the gates. Call once, early; gates reported before it still count. */
void ota_health_start(void);

/* Reports a gate. Idempotent, and safe to call from any task. */
void ota_health_pass(ota_gate_t gate);

/* ---- what the console asks ---------------------------------------------- */

/* Labels of the app partition running now and of the one the bootloader would
 * pick on the next reset. Valid strings before ota_health_start(), too. */
const char *ota_running_label(void);
const char *ota_boot_label(void);
/* The running image was still on probation when it came up. */
bool     ota_health_on_probation(void);
/*
 * The other app slot holds an image the bootloader gave up on, which means
 * the last thing installed never reached the gates above and this firmware is
 * what it went back to. A rollback that nobody is told about is a rollback
 * that looks like an update which quietly did nothing, so this is what the
 * screen and the console say it with. It stays true until something is
 * written into that slot again.
 */
bool        ota_health_rolled_back(void);
const char *ota_rollback_label(void);   /* the abandoned slot, or "none" */
/* Gates passed so far, as a mask of ota_gate_t. */
uint32_t ota_health_gates(void);
/* The check has completed: rollback cancelled, or nothing to cancel. */
bool     ota_health_confirmed(void);
