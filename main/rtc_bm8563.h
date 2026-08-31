/*
 * BM8563 real time clock, on the M5StickC PLUS2's internal I2C bus
 * (GPIO21 SDA / GPIO22 SCL, address 0x51).
 *
 * The point of it is that a capture taken on the bike, hours after the last
 * USB connection, still carries a wall clock start time. The RTC is backed by
 * the board's own battery, so it keeps running across a power cycle; the
 * seconds register has a voltage-low flag that says whether it ever lost time.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "driver/gpio.h"
#include "esp_err.h"

#define RTC_I2C_SDA GPIO_NUM_21
#define RTC_I2C_SCL GPIO_NUM_22
#define RTC_I2C_ADDR 0x51

esp_err_t bm8563_init(void);
bool      bm8563_present(void);
/* False while the voltage-low flag is set, i.e. the time was never set or the
 * backup battery went flat. A capture then stores unix_start = 0. */
bool      bm8563_valid(void);

esp_err_t bm8563_get(struct tm *out);
esp_err_t bm8563_set(const struct tm *in);
/* Seconds since the epoch (UTC), or -1 when the clock is not trustworthy. */
int64_t   bm8563_unix(void);
/* Copies the RTC into the system clock so localtime()/time() work. */
esp_err_t bm8563_sync_system_time(void);
