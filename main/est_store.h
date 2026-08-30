/*
 * est_store - the estimator's persisted state, in NVS.
 *
 * This is the whole of what main/wfest does not know about. The estimator
 * hands out a plain POD struct and encodes it to a fixed byte string; this
 * file is the only thing that knows those bytes end up in a flash partition
 * under a key, and it is firmware, not pure C99, precisely so that the seam
 * stays pure. Swap NVS for a file or a host stub and nothing in main/wfest
 * changes.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "wfest/wfest.h"

/* Reads the saved state. False when nothing is saved, or what is saved is not
 * ours - a blob from an older build, a half-written record, a flipped bit.
 * A false here is not an error: cap_init() then starts the estimator cold and
 * the first BMS answer acquires an Anchor within a second. */
bool est_store_load(wf_est_persist_t *out);

/* Writes the saved state, replacing whatever was there. Called at the end of a
 * capture and on shutdown, not on a timer: NVS is flash, and the value this
 * bridges - the seconds between power-on and the first BMS answer - is not
 * worth a write per second of riding. */
esp_err_t est_store_save(const wf_est_persist_t *p);
