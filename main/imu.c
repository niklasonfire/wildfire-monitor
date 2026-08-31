/*
 * MPU6886 driver and the IMU logging task.
 *
 * The bus belongs to i2c_bus.c because the BM8563 clock shares it, so this
 * driver only adds and removes its own device handle. Everything the host
 * decoder needs is raw counts at a fixed full scale, so the configuration is
 * written once at init and never touched again - a capture whose sensitivity
 * changed halfway through would be worse than no capture at all.
 *
 * The logging task exists to make the undecoded BLE frames interpretable: a
 * candidate payload field only means something once it can be lined up against
 * an independent "the bike was accelerating here" signal on the same clock.
 * That makes the sample interval, not the sample itself, the critical part -
 * see imu_task().
 */
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "imu.h"

#include "capture_store.h"
#include "i2c_bus.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "imu";

#define IMU_I2C_TIMEOUT_MS 100

/* Register map, MPU6886 datasheet section 8. */
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_INT_ENABLE    0x38
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

/* Accel X/Y/Z, temperature, gyro X/Y/Z: seven big-endian 16 bit words that sit
 * back to back from REG_ACCEL_XOUT_H, so one transaction gets a coherent set
 * of axes rather than three reads straddling an internal sample boundary. */
#define IMU_BURST_LEN 14

/* Task */
#define IMU_TASK_STACK 3072
#define IMU_TASK_PRIO  4
/* Long enough to cover one period plus a stalled I2C transaction, short enough
 * that a wedged task does not hold up closing the capture file. */
#define IMU_STOP_WAIT_MS 2000

static i2c_master_dev_handle_t s_dev;
static SemaphoreHandle_t       s_mtx;      /* imu_read() is called from the UI
                                            * and the console as well as the
                                            * logging task */
static bool                    s_present;

static TaskHandle_t      s_task;
static SemaphoreHandle_t s_task_done;  /* given by the task just before it exits */
static volatile bool     s_run;
static int64_t           s_t0_us;
static uint32_t          s_samples;
static uint32_t          s_drops;      /* store_write() refusals, not fatal */

/* --- raw register access ------------------------------------------------- */

static esp_err_t imu_write_reg(uint8_t reg, uint8_t val)
{
    if (!s_present || s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[2] = {reg, val};
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    esp_err_t err = i2c_master_transmit(s_dev, buf, sizeof(buf), IMU_I2C_TIMEOUT_MS);
    xSemaphoreGive(s_mtx);
    return err;
}

static esp_err_t imu_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, data, len,
                                                IMU_I2C_TIMEOUT_MS);
    xSemaphoreGive(s_mtx);
    return err;
}

/* The sensor puts the high byte first and the ESP32 is little endian, so the
 * word is assembled by hand; a memcpy of the burst into an int16_t array would
 * byte-swap every axis. */
static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* --- init ---------------------------------------------------------------- */

esp_err_t imu_init(void)
{
    if (s_present) {
        return ESP_OK; /* idempotent, the capture path may call it twice */
    }

    if (s_mtx == NULL) {
        s_mtx = xSemaphoreCreateMutex();
        if (s_mtx == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    /* i2c_bus_init() is idempotent, so it does not matter whether the IMU or
     * the clock gets here first. The handle stays local: the bus outlives this
     * driver and must never be deleted from here. */
    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "shared I2C bus unavailable: %s", esp_err_to_name(err));
        return err;
    }

    i2c_master_bus_handle_t bus = i2c_bus();
    if (bus == NULL) {
        ESP_LOGW(TAG, "shared I2C bus not up");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_dev == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = IMU_I2C_ADDR,
            .scl_speed_hz = I2C_BUS_HZ,
        };
        err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "add device failed: %s", esp_err_to_name(err));
            s_dev = NULL;
            return err;
        }
    }

    /* Identity check rather than a bare probe: an address that acknowledges is
     * not proof it is the part this register map belongs to. */
    uint8_t who = 0;
    err = imu_read_regs(REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK || who != IMU_WHO_AM_I) {
        if (err == ESP_OK) {
            ESP_LOGW(TAG, "WHO_AM_I 0x%02x, expected 0x%02x", who, IMU_WHO_AM_I);
        } else {
            ESP_LOGW(TAG, "no MPU6886 at 0x%02x: %s", IMU_I2C_ADDR,
                     esp_err_to_name(err));
        }
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    /* imu_write_reg() refuses while s_present is false; the chip has answered
     * for itself by now, so open the gate before configuring it. */
    s_present = true;

    /* Reset first. A warm boot after a crash leaves the sensor configured from
     * the previous run, and the sample rate divider in particular would then
     * be whatever that run wanted. */
    err = imu_write_reg(REG_PWR_MGMT_1, 0x80);
    if (err == ESP_OK) {
        /* The datasheet asks for a settling delay after DEVICE_RESET; the bit
         * clears itself and the part is not addressable in between. */
        vTaskDelay(pdMS_TO_TICKS(10));
        err = imu_write_reg(REG_PWR_MGMT_1, 0x00);  /* wake, internal osc */
    }
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* reg, value, and why the value is what it is. */
    static const struct {
        uint8_t reg;
        uint8_t val;
    } cfg[] = {
        /* CLKSEL = 1: follow the gyro PLL when it is up, fall back to the
         * internal oscillator otherwise. Steadier than CLKSEL 0, and the
         * steadiness is what the sample interval rides on. */
        {REG_PWR_MGMT_1,    0x01},
        /* ACCEL_FS_SEL = 2 in bits 4:3 -> +-8 g, 4096 LSB/g. A motorbike
         * hitting a pothole clips +-4 g. */
        {REG_ACCEL_CONFIG,  0x10},
        /* FS_SEL = 3 in bits 4:3 -> +-2000 dps, 16.4 LSB/dps. */
        {REG_GYRO_CONFIG,   0x18},
        /* DLPF_CFG = 1: gyro bandwidth 176 Hz. Frame rate is 36 Hz and this
         * logs at 20 Hz, so the point is only to keep engine and road buzz
         * from aliasing down into the band that carries the riding. */
        {REG_CONFIG,        0x01},
        /* A_DLPF_CFG = 1 with ACCEL_FCHOICE_B clear: the matching 218 Hz
         * accelerometer filter. */
        {REG_ACCEL_CONFIG2, 0x01},
        /* 1 kHz / (1 + 4) = 200 Hz internal sampling, ten times the logging
         * rate, so every logged sample is fresh to well under a millisecond. */
        {REG_SMPLRT_DIV,    0x04},
        /* No interrupts: the INT line is shared on this board and nothing
         * here is interrupt driven, the task polls. */
        {REG_INT_ENABLE,    0x00},
    };

    for (size_t i = 0; i < sizeof(cfg) / sizeof(cfg[0]) && err == ESP_OK; i++) {
        err = imu_write_reg(cfg[i].reg, cfg[i].val);
        /* The part needs a moment between configuration writes; this runs once
         * at startup, so the milliseconds are free. */
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "config write failed: %s", esp_err_to_name(err));
        s_present = false;
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }

    ESP_LOGI(TAG, "MPU6886 present, +-8 g / +-2000 dps");
    return ESP_OK;
}

bool imu_present(void)
{
    return s_present;
}

esp_err_t imu_read(imu_sample_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_present || s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t r[IMU_BURST_LEN] = {0};
    esp_err_t err = imu_read_regs(REG_ACCEL_XOUT_H, r, sizeof(r));
    if (err != ESP_OK) {
        return err;
    }

    out->ax   = be16(&r[0]);
    out->ay   = be16(&r[2]);
    out->az   = be16(&r[4]);
    out->temp = be16(&r[6]);   /* die temperature, not logged, see imu_task() */
    out->gx   = be16(&r[8]);
    out->gy   = be16(&r[10]);
    out->gz   = be16(&r[12]);

    return ESP_OK;
}

/* --- logging task -------------------------------------------------------- */

static void imu_task(void *arg)
{
    (void)arg;

    /* A fixed period from a wake time the scheduler advances for us. The I2C
     * burst takes a variable ~2 ms and a FAT flush elsewhere can steal more,
     * and vTaskDelay() would add every one of those to the interval - the
     * sample clock would then drift away from the frame timestamps it exists
     * to be correlated against. xTaskDelayUntil() absorbs the read time
     * instead, so 20 Hz stays 20 Hz over a whole ride. */
    const TickType_t period = pdMS_TO_TICKS(1000 / IMU_LOG_HZ) > 0
                                  ? pdMS_TO_TICKS(1000 / IMU_LOG_HZ)
                                  : 1;
    TickType_t wake = xTaskGetTickCount();

    while (s_run) {
        imu_sample_t s;
        esp_err_t err = imu_read(&s);

        /* Timestamped after the transaction, on the same esp_timer clock and
         * the same t0 as the frame records. */
        int64_t now_us = esp_timer_get_time();
        int64_t rel_us = now_us - s_t0_us;
        if (rel_us < 0) {
            rel_us = 0;
        }
        uint32_t t_ms = (uint32_t)(rel_us / 1000);

        if (err == ESP_OK) {
            /* Six axes only. Temperature is in the burst because it sits
             * between the accelerometer and the gyro and cannot be skipped,
             * but it says nothing about how the bike is moving and would cost
             * two bytes on every one of 20 records a second. */
            wflog_imu_t rec = {
                .ax = s.ax, .ay = s.ay, .az = s.az,
                .gx = s.gx, .gy = s.gy, .gz = s.gz,
            };
            if (store_write(WFREC_IMU, t_ms, &rec, sizeof(rec))) {
                s_samples++;
            } else {
                /* The ring was full. The store counts its own drops and the
                 * next sample is 50 ms away; losing one is not worth aborting
                 * a capture over. */
                s_drops++;
            }
        } else {
            s_drops++;
        }

        xTaskDelayUntil(&wake, period);
    }

    ESP_LOGI(TAG, "imu logging stopped, %" PRIu32 " samples, %" PRIu32 " lost",
             s_samples, s_drops);

    /* Hand the flag over before deleting ourselves: once this is given, no
     * further store_write() can come from here, which is exactly what
     * imu_log_stop() waits for before the capture file is closed. */
    s_task = NULL;
    xSemaphoreGive(s_task_done);
    vTaskDelete(NULL);
}

esp_err_t imu_log_start(int64_t t0_us)
{
    if (!s_present) {
        return ESP_ERR_NOT_FOUND;
    }
    if (s_task != NULL || s_run) {
        /* Deliberately an error rather than a silent no-op: a second start
         * carries a second t0_us, and quietly keeping the first one would
         * shift every later timestamp against the frames. */
        ESP_LOGW(TAG, "already logging");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task_done == NULL) {
        s_task_done = xSemaphoreCreateBinary();
        if (s_task_done == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    /* Drain a leftover token from a previous stop that timed out, so this
     * run's stop waits for this run's task instead of returning at once. */
    (void)xSemaphoreTake(s_task_done, 0);

    s_t0_us = t0_us;
    s_samples = 0;
    s_drops = 0;
    s_run = true;

    if (xTaskCreate(imu_task, "imu", IMU_TASK_STACK, NULL, IMU_TASK_PRIO,
                    &s_task) != pdPASS) {
        ESP_LOGE(TAG, "imu task create failed");
        s_run = false;
        s_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "imu logging at %d Hz", IMU_LOG_HZ);
    return ESP_OK;
}

void imu_log_stop(void)
{
    if (s_task == NULL && !s_run) {
        return; /* never started, or already stopped */
    }

    s_run = false;

    /* Wait for the task to have actually left its loop. Returning early would
     * let a sample land in store_write() after store_end() has closed the
     * capture. */
    if (s_task_done != NULL &&
        xSemaphoreTake(s_task_done, pdMS_TO_TICKS(IMU_STOP_WAIT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "imu task did not exit within %d ms", IMU_STOP_WAIT_MS);
    }

    s_task = NULL;
}

bool imu_logging(void)
{
    return s_run && s_task != NULL;
}
