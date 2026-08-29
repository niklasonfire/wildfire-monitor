/*
 * BM8563 driver for the M5StickC PLUS2.
 *
 * Everything here goes over the new i2c_master driver; the legacy driver/i2c.h
 * API is gone in IDF v6. The bus itself belongs to i2c_bus.c because the IMU
 * shares it, so this driver only adds and removes its own device handle. The
 * chip is tiny: one burst read of registers 0x02..0x08 is the whole time, and
 * the only state worth caring about is the voltage-low flag in bit 7 of the
 * seconds register.
 */
#include <inttypes.h>
#include <string.h>
#include <sys/time.h>

/* rtc_bm8563.h names GPIO_NUM_21/22 but does not pull in the GPIO header. */
#include "driver/gpio.h"
#include "rtc_bm8563.h"

#include "i2c_bus.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "rtc";

#define RTC_I2C_TIMEOUT_MS 100

/* Register map. The time registers are BCD with a few flag bits stuck on top,
 * so every field needs its own mask before it is decoded. */
#define REG_CONTROL1 0x00
#define REG_CONTROL2 0x01
#define REG_SECONDS  0x02 /* bit 7 = VL, voltage low: the time is meaningless */
#define REG_MINUTES  0x03 /* bits 0..6 */
#define REG_HOURS    0x04 /* bits 0..5 */
#define REG_DAY      0x05 /* bits 0..5 */
#define REG_WEEKDAY  0x06 /* bits 0..2 */
#define REG_MONTH    0x07 /* bits 0..4, bit 7 = century */
#define REG_YEAR     0x08 /* full byte, 00..99 BCD */

#define MASK_VL      0x80
#define MASK_CENTURY 0x80

/* Registers 0x02..0x08 inclusive: the whole clock in one transaction. */
#define TIME_REGS 7

static i2c_master_dev_handle_t s_dev;
static SemaphoreHandle_t       s_mtx;
static bool                    s_present;
/* Cached copy of the VL flag, refreshed by every register read that sees it. */
static bool                    s_vl = true;

static inline uint8_t bcd2dec(uint8_t v)
{
    return (uint8_t)(((v >> 4) & 0x0F) * 10 + (v & 0x0F));
}

static inline uint8_t dec2bcd(uint8_t v)
{
    return (uint8_t)(((v / 10) << 4) | (v % 10));
}

/* --- raw register access ------------------------------------------------- */

static esp_err_t rtc_write(uint8_t reg, const uint8_t *data, size_t len)
{
    if (!s_present || s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > TIME_REGS) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t buf[1 + TIME_REGS];
    buf[0] = reg;
    if (len > 0) {
        memcpy(&buf[1], data, len);
    }

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    esp_err_t err = i2c_master_transmit(s_dev, buf, len + 1, RTC_I2C_TIMEOUT_MS);
    xSemaphoreGive(s_mtx);
    return err;
}

static esp_err_t rtc_read(uint8_t reg, uint8_t *data, size_t len)
{
    if (!s_present || s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, data, len,
                                                RTC_I2C_TIMEOUT_MS);
    xSemaphoreGive(s_mtx);
    return err;
}

/* --- init ---------------------------------------------------------------- */

esp_err_t bm8563_init(void)
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

    err = i2c_master_probe(bus, RTC_I2C_ADDR, RTC_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no BM8563 at 0x%02x: %s", RTC_I2C_ADDR, esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RTC_I2C_ADDR,
        .scl_speed_hz = I2C_BUS_HZ,
    };
    err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "add device failed: %s", esp_err_to_name(err));
        s_dev = NULL;
        return err;
    }

    s_present = true;

    /* Both control registers to zero: no alarm, no timer, no interrupt driving
     * the shared INT line, which is all this firmware wants from the chip. */
    uint8_t ctrl[2] = {0x00, 0x00};
    err = rtc_write(REG_CONTROL1, ctrl, sizeof(ctrl));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "control register write failed: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        s_present = false;
        return err;
    }

    /* Prime the cached VL flag so bm8563_valid() is meaningful before any read. */
    uint8_t sec = 0;
    if (rtc_read(REG_SECONDS, &sec, 1) == ESP_OK) {
        s_vl = (sec & MASK_VL) != 0;
    }

    ESP_LOGI(TAG, "BM8563 present, time %s", s_vl ? "not set" : "valid");
    return ESP_OK;
}

bool bm8563_present(void)
{
    return s_present;
}

bool bm8563_valid(void)
{
    uint8_t sec = 0;
    if (rtc_read(REG_SECONDS, &sec, 1) != ESP_OK) {
        return false;
    }
    s_vl = (sec & MASK_VL) != 0;
    return !s_vl;
}

/* --- time ---------------------------------------------------------------- */

esp_err_t bm8563_get(struct tm *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* One burst, so seconds cannot roll over between the fields and hand back
     * a time that never existed. */
    uint8_t r[TIME_REGS] = {0};
    esp_err_t err = rtc_read(REG_SECONDS, r, sizeof(r));
    if (err != ESP_OK) {
        return err;
    }

    s_vl = (r[0] & MASK_VL) != 0;

    memset(out, 0, sizeof(*out));
    out->tm_sec  = bcd2dec(r[0] & 0x7F);
    out->tm_min  = bcd2dec(r[1] & 0x7F);
    out->tm_hour = bcd2dec(r[2] & 0x3F);
    out->tm_mday = bcd2dec(r[3] & 0x3F);
    out->tm_wday = r[4] & 0x07;
    out->tm_mon  = bcd2dec(r[5] & 0x1F) - 1; /* struct tm counts months from 0 */

    /* Datasheet century bit: set means the 1900s, clear means the 2000s. This
     * firmware only ever writes it clear and stores 2000 + the BCD year, so
     * the branch exists purely to read back a clock somebody else set. */
    int year = bcd2dec(r[6]);
    out->tm_year = ((r[5] & MASK_CENTURY) ? 1900 : 2000) + year - 1900;
    out->tm_isdst = 0;

    return ESP_OK;
}

esp_err_t bm8563_set(const struct tm *in)
{
    if (in == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int full_year = in->tm_year + 1900;
    if (full_year < 1900 || full_year > 2099 ||
        in->tm_mon < 0 || in->tm_mon > 11 ||
        in->tm_mday < 1 || in->tm_mday > 31 ||
        in->tm_hour < 0 || in->tm_hour > 23 ||
        in->tm_min < 0 || in->tm_min > 59 ||
        in->tm_sec < 0 || in->tm_sec > 59) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t century = 0;
    int yy = full_year - 2000;
    if (full_year < 2000) {
        century = MASK_CENTURY;
        yy = full_year - 1900;
    }

    int wday = in->tm_wday;
    if (wday < 0 || wday > 6) {
        wday = 0;
    }

    uint8_t r[TIME_REGS];
    /* Writing the seconds register with bit 7 clear is what clears VL, so a
     * successful set is also what makes the clock trustworthy again. */
    r[0] = dec2bcd((uint8_t)in->tm_sec) & 0x7F;
    r[1] = dec2bcd((uint8_t)in->tm_min) & 0x7F;
    r[2] = dec2bcd((uint8_t)in->tm_hour) & 0x3F;
    r[3] = dec2bcd((uint8_t)in->tm_mday) & 0x3F;
    r[4] = (uint8_t)wday & 0x07;
    r[5] = (dec2bcd((uint8_t)(in->tm_mon + 1)) & 0x1F) | century;
    r[6] = dec2bcd((uint8_t)yy);

    esp_err_t err = rtc_write(REG_SECONDS, r, sizeof(r));
    if (err == ESP_OK) {
        s_vl = false;
    }
    return err;
}

int64_t bm8563_unix(void)
{
    struct tm tm_now;
    if (bm8563_get(&tm_now) != ESP_OK) {
        return -1;
    }
    /* bm8563_get() refreshed the flag from the same burst as the time, so this is
     * bm8563_valid() without a second transaction over the wire. */
    if (s_vl) {
        return -1;
    }

    time_t t = timegm(&tm_now); /* the RTC is kept in UTC */
    if (t == (time_t)-1) {
        return -1;
    }
    return (int64_t)t;
}

esp_err_t bm8563_sync_system_time(void)
{
    int64_t unix_s = bm8563_unix();
    if (unix_s < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    struct timeval tv = {
        .tv_sec = (time_t)unix_s,
        .tv_usec = 0,
    };
    if (settimeofday(&tv, NULL) != 0) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "system clock set from RTC, unix %" PRId64, unix_s);
    return ESP_OK;
}
