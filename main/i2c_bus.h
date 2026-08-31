/*
 * The internal I2C bus of the M5StickC PLUS2 (GPIO21 SDA / GPIO22 SCL).
 *
 * Two chips hang off it - the BM8563 real time clock and the MPU6886 IMU - and
 * the v6 I2C driver lets exactly one owner create a port. So the bus is
 * created here once and both drivers just add their device to it.
 */
#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#define I2C_BUS_SDA  GPIO_NUM_21
#define I2C_BUS_SCL  GPIO_NUM_22
#define I2C_BUS_HZ   100000

/* Idempotent: the first caller creates the bus, later callers just get it. */
esp_err_t i2c_bus_init(void);
/* NULL until i2c_bus_init() has succeeded. */
i2c_master_bus_handle_t i2c_bus(void);
