/*
 * ui - what the LCD shows while the capture runs.
 *
 * A redraw task polls cap_status() a few times a second and paints the screen
 * for the current state. Everything the rider needs at a glance: which state
 * the capture is in, whether both links are up, how long it has been running
 * and how much battery and flash is left.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t ui_init(void);      /* display up, splash screen */
esp_err_t ui_start(void);     /* start the redraw task */

/* Puts a full-screen message up until ui_message_clear(): used for the Wi-Fi
 * readout screen and for fatal errors. Either line may be NULL. */
void ui_message(const char *title, const char *l1, const char *l2,
                const char *l3, const char *l4);
void ui_message_clear(void);
/* Forces a full repaint, e.g. after the backlight came back on. */
void ui_redraw(void);
