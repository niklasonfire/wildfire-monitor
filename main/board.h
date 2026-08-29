/*
 * Board support for the M5StickC PLUS2 (ESP32-PICO-V3-02).
 *
 * The standalone capture runs off the internal battery with nobody watching,
 * so this layer owns the three things that decide whether it survives: the
 * power latch, the buttons that drive the state machine and the battery
 * voltage that tells the capture when to close its file.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"

#define PIN_POWER_HOLD GPIO_NUM_4
#define PIN_LED        GPIO_NUM_19
#define PIN_BTN_A      GPIO_NUM_37
#define PIN_BTN_B      GPIO_NUM_39
#define PIN_BTN_PWR    GPIO_NUM_35
#define PIN_BAT_ADC    GPIO_NUM_38

void board_init(void);
void board_led_set(bool on);
bool board_led_get(void);
void board_led_blink(int times, int on_ms, int off_ms);

/* Buttons read active-low in hardware; these return true while pressed. */
bool board_btn_a(void);
bool board_btn_b(void);
bool board_btn_pwr(void);

/* ---- button events ------------------------------------------------------
 *
 * The capture is driven by presses, not by levels, so a poll task debounces
 * the three inputs and posts one event per press. A press is reported as LONG
 * as soon as it has been held for BOARD_LONG_PRESS_MS - while still held, so
 * the user gets feedback without having to let go - and as SHORT on release.
 */
#define BOARD_LONG_PRESS_MS 800

typedef enum { BOARD_BTN_A_ID = 0, BOARD_BTN_B_ID, BOARD_BTN_PWR_ID } board_btn_id_t;
typedef enum { BOARD_PRESS_SHORT = 0, BOARD_PRESS_LONG } board_press_t;

typedef struct {
    board_btn_id_t btn;
    board_press_t  press;
} board_btn_evt_t;

/* Starts the poll task and the event queue. Call once, after board_init(). */
void board_buttons_start(void);
/* Waits for the next press. timeout_ms < 0 blocks forever. */
bool board_btn_wait(board_btn_evt_t *out, int timeout_ms);
void board_btn_flush(void);

/* ---- battery ------------------------------------------------------------ */

/* Battery voltage in millivolts, or -1 if the ADC is not up yet. GPIO38 sits
 * behind a 1:2 divider, so the raw reading is doubled. Averaged over several
 * samples because the BLE radio makes a single one jump around. */
int board_battery_mv(void);
/* Rough state of charge in percent for a 1S LiPo (4.2 V full, 3.3 V empty). */
int board_battery_pct(void);

/* Releases the power latch: the board switches off unless USB is plugged in.
 * Never returns when running on battery. */
void board_power_off(void);
