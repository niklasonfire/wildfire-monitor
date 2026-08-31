#include "board.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "board";

static bool s_led_on;

/* ---- battery ------------------------------------------------------------
 *
 * GPIO38 is ADC1 channel 2 on the ESP32 and sits behind a 1:2 divider, so a
 * full 4.2 V pack reads as ~2.1 V at the pin. 12 dB attenuation puts that
 * comfortably inside the usable input range.
 */
#define BAT_ADC_UNIT    ADC_UNIT_1
#define BAT_ADC_CHANNEL ADC_CHANNEL_2
#define BAT_SAMPLES     8

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_adc_cali;
static bool s_adc_ready;

static void battery_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BAT_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC1 unit init failed: %s", esp_err_to_name(err));
        s_adc = NULL;
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc, BAT_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC1 channel config failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        return;
    }

    /* Line fitting is the only calibration scheme the ESP32 has. Without the
     * eFuse characterisation the driver falls back to a nominal Vref, and if
     * the scheme cannot be created at all board_battery_mv() uses a plain
     * raw-to-millivolt ratio instead - good to maybe 5 %, enough to decide
     * when to close a capture file. */
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = BAT_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .default_vref = 1100,
    };
    err = adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration unavailable: %s", esp_err_to_name(err));
        s_adc_cali = NULL;
    }
#endif

    s_adc_ready = true;
}

/* ---- button events ------------------------------------------------------ */

#define BTN_POLL_MS        10
#define BTN_DEBOUNCE_MS    30
#define BTN_DEBOUNCE_TICKS (BTN_DEBOUNCE_MS / BTN_POLL_MS)
#define BTN_QUEUE_LEN      8
#define BTN_COUNT          3

struct btn_state {
    board_btn_id_t id;
    gpio_num_t     pin;
    bool           raw;        /* last raw sample, true while pressed */
    bool           stable;     /* debounced level */
    int            settle;     /* poll ticks the raw level has been steady */
    int            held_ms;    /* time since the press was accepted */
    bool           long_fired; /* LONG already reported, suppress SHORT */
};

static QueueHandle_t s_btn_q;
static TaskHandle_t  s_btn_task;

static void btn_post(board_btn_id_t btn, board_press_t press)
{
    board_btn_evt_t evt = {.btn = btn, .press = press};
    /* Zero timeout: the poll task never blocks on anything but vTaskDelay, so
     * a press is dropped rather than delayed if nobody drains the queue. No
     * log here either - ESP_LOGx takes the stdout lock. */
    (void)xQueueSend(s_btn_q, &evt, 0);
}

static void btn_task(void *arg)
{
    (void)arg;

    struct btn_state btns[BTN_COUNT] = {
        {.id = BOARD_BTN_A_ID,   .pin = PIN_BTN_A},
        {.id = BOARD_BTN_B_ID,   .pin = PIN_BTN_B},
        {.id = BOARD_BTN_PWR_ID, .pin = PIN_BTN_PWR},
    };

    while (true) {
        for (int i = 0; i < BTN_COUNT; i++) {
            struct btn_state *b = &btns[i];

            /* Active low: the pins have no internal pulls, the board pulls
             * them up externally and the switch shorts them to ground. */
            bool raw = (gpio_get_level(b->pin) == 0);

            if (raw != b->raw) {
                b->raw = raw;
                b->settle = 0;
            } else if (b->settle < BTN_DEBOUNCE_TICKS) {
                b->settle++;
            }

            bool debounced = (b->settle >= BTN_DEBOUNCE_TICKS) ? b->raw : b->stable;

            if (debounced && !b->stable) {
                /* Press accepted. */
                b->held_ms = 0;
                b->long_fired = false;
            } else if (!debounced && b->stable) {
                /* Release. A press that already reported LONG stays silent. */
                if (!b->long_fired) {
                    btn_post(b->id, BOARD_PRESS_SHORT);
                }
            } else if (debounced) {
                b->held_ms += BTN_POLL_MS;
                if (!b->long_fired && b->held_ms >= BOARD_LONG_PRESS_MS) {
                    /* Fire while the button is still down so the user sees the
                     * board react without having to let go. */
                    b->long_fired = true;
                    btn_post(b->id, BOARD_PRESS_LONG);
                }
            }

            b->stable = debounced;
        }

        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
    }
}

/* ---- public API --------------------------------------------------------- */

void board_init(void)
{
    gpio_config_t out_cfg = {
        .pin_bit_mask = BIT64(PIN_POWER_HOLD) | BIT64(PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    /* Latch power on so the board survives running off the internal battery. */
    ESP_ERROR_CHECK(gpio_set_level(PIN_POWER_HOLD, 1));
    ESP_ERROR_CHECK(gpio_set_level(PIN_LED, 0));

    /* GPIO34-39 are input only and have no internal pull resistors. */
    gpio_config_t in_cfg = {
        .pin_bit_mask = BIT64(PIN_BTN_A) | BIT64(PIN_BTN_B) | BIT64(PIN_BTN_PWR),
        .mode = GPIO_MODE_INPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&in_cfg));

    /* Bring the ADC up here rather than lazily: the first battery reading can
     * come from any task and adc_oneshot_new_unit() is not reentrant. */
    battery_adc_init();
}

void board_led_set(bool on)
{
    s_led_on = on;
    gpio_set_level(PIN_LED, on ? 1 : 0);
}

bool board_led_get(void)
{
    return s_led_on;
}

void board_led_blink(int times, int on_ms, int off_ms)
{
    for (int i = 0; i < times; i++) {
        board_led_set(true);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        board_led_set(false);
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

bool board_btn_a(void)   { return !gpio_get_level(PIN_BTN_A); }
bool board_btn_b(void)   { return !gpio_get_level(PIN_BTN_B); }
bool board_btn_pwr(void) { return !gpio_get_level(PIN_BTN_PWR); }

void board_buttons_start(void)
{
    if (s_btn_task != NULL) {
        return; /* already running */
    }
    if (s_btn_q == NULL) {
        s_btn_q = xQueueCreate(BTN_QUEUE_LEN, sizeof(board_btn_evt_t));
        if (s_btn_q == NULL) {
            ESP_LOGE(TAG, "button queue alloc failed");
            return;
        }
    }
    if (xTaskCreate(btn_task, "btn", 3072, NULL, 5, &s_btn_task) != pdPASS) {
        ESP_LOGE(TAG, "button task create failed");
        s_btn_task = NULL;
    }
}

bool board_btn_wait(board_btn_evt_t *out, int timeout_ms)
{
    if (out == NULL || s_btn_q == NULL) {
        return false;
    }
    TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive(s_btn_q, out, ticks) == pdTRUE;
}

void board_btn_flush(void)
{
    if (s_btn_q != NULL) {
        xQueueReset(s_btn_q);
    }
}

int board_battery_mv(void)
{
    if (!s_adc_ready) {
        return -1;
    }

    /* One sample moves by tens of millivolts while the BLE radio bursts, so
     * average a handful before anyone acts on the number. */
    int sum = 0;
    int used = 0;
    for (int i = 0; i < BAT_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, BAT_ADC_CHANNEL, &raw) != ESP_OK) {
            continue;
        }
        int mv = 0;
        if (s_adc_cali == NULL || adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) != ESP_OK) {
            /* Uncalibrated approximation: 12 bit width, ~3.3 V full scale at
             * 12 dB attenuation. */
            mv = raw * 3300 / 4095;
        }
        sum += mv;
        used++;
    }
    if (used == 0) {
        return -1;
    }

    return (sum / used) * 2; /* undo the 1:2 divider */
}

int board_battery_pct(void)
{
    int mv = board_battery_mv();
    if (mv < 0) {
        return -1;
    }

    /* Straight line between 3.30 V and 4.20 V. A 1S LiPo discharge curve is
     * flat through the middle, so this overstates the state of charge between
     * roughly 40 % and 80 %; it is only meant as a coarse gauge and as the
     * trigger for closing a capture near the empty end, where the curve does
     * drop steeply and the line is close to right. */
    if (mv >= 4200) {
        return 100;
    }
    if (mv <= 3300) {
        return 0;
    }
    return (mv - 3300) * 100 / (4200 - 3300);
}

void board_power_off(void)
{
    board_led_set(false);

    /* Releasing the latch cuts the regulator when running on the internal
     * battery. On USB power nothing happens, so park here forever rather than
     * returning into code that assumes the board is gone. */
    gpio_set_level(PIN_POWER_HOLD, 0);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
