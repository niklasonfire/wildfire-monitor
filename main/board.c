#include "board.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"

static bool s_led_on;

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
