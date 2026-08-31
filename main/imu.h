/*
 * MPU6886 six-axis IMU, on the internal I2C bus at address 0x68.
 *
 * The point of logging it is that the BLE frames alone are uninterpretable:
 * knowing that payload bytes 10..11 rose does not say whether the bike was
 * accelerating, braking or standing still. The IMU gives an independent
 * movement signal, recorded into the same capture file with the same clock,
 * so a candidate field can be correlated against what the bike was doing.
 *
 * Full scales are fixed at +-8 g and +-2000 dps, which is what a motorbike
 * needs and what the host decoder assumes: 4096 LSB/g and 16.4 LSB/dps.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define IMU_I2C_ADDR   0x68
#define IMU_WHO_AM_I   0x19    /* the MPU6886's identity byte */
#define IMU_ACCEL_LSB_PER_G   4096
#define IMU_GYRO_LSB_PER_DPS  164   /* tenths: 16.4 LSB/dps */
#define IMU_LOG_HZ     20

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} imu_sample_t;

esp_err_t imu_init(void);      /* needs i2c_bus_init() first */
bool      imu_present(void);
esp_err_t imu_read(imu_sample_t *out);

/* Starts a task that writes one WFREC_IMU record per sample at IMU_LOG_HZ
 * into the open capture. t0_us is the capture's zero point, the same one the
 * frame timestamps are relative to. */
esp_err_t imu_log_start(int64_t t0_us);
void      imu_log_stop(void);
bool      imu_logging(void);
