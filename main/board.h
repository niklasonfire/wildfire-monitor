/*
 * Board support for the M5StickC PLUS2 (ESP32-PICO-V3-02).
 *
 * Only the pieces the scanner needs: keeping the power latch closed, the red
 * LED as an activity indicator and the three buttons.
 */
#pragma once

#include <stdbool.h>

#include "driver/gpio.h"

#define PIN_POWER_HOLD GPIO_NUM_4
#define PIN_LED        GPIO_NUM_19
#define PIN_BTN_A      GPIO_NUM_37
#define PIN_BTN_B      GPIO_NUM_39
#define PIN_BTN_PWR    GPIO_NUM_35

void board_init(void);
void board_led_set(bool on);
bool board_led_get(void);
void board_led_blink(int times, int on_ms, int off_ms);

/* Buttons read active-low in hardware; these return true while pressed. */
bool board_btn_a(void);
bool board_btn_b(void);
bool board_btn_pwr(void);
