/*
 * wildfire_monitor - BLE scanner for the Blacktea Wildfire electric motorbike.
 *
 * Stage one of the project: find the Fardriver motor controller and the Daly
 * BMS, connect to both and pull out everything that a later, permanent
 * connection will need - handles, MTU, connection parameters, which
 * characteristics stream data on their own and which have to be polled.
 *
 * The whole thing is driven from a console on UART0 (115200 8N1) so a script
 * on the host can run a capture unattended. See cmd_ble.c for the commands.
 */
#include <stdio.h>
#include <string.h>

#include "ble_explorer.h"
#include "board.h"

#include "esp_chip_info.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void cmd_ble_register(void);

static const char *TAG = "main";

static volatile bool s_heartbeat;

static int cmd_led(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: led on|off|toggle\n");
        return 1;
    }
    if (strcmp(argv[1], "on") == 0) {
        board_led_set(true);
    } else if (strcmp(argv[1], "off") == 0) {
        board_led_set(false);
    } else if (strcmp(argv[1], "toggle") == 0) {
        board_led_set(!board_led_get());
    } else {
        printf("unknown argument: %s\n", argv[1]);
        return 1;
    }
    printf("led=%s\n", board_led_get() ? "on" : "off");
    return 0;
}

static int cmd_ping(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("pong uptime_ms=%lld\n", esp_timer_get_time() / 1000);
    return 0;
}

static int cmd_btn(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("btnA=%d btnB=%d btnPWR=%d\n", board_btn_a(), board_btn_b(), board_btn_pwr());
    return 0;
}

static int cmd_hb(int argc, char **argv)
{
    if (argc == 2) {
        s_heartbeat = (strcmp(argv[1], "on") == 0);
    }
    printf("heartbeat=%s\n", s_heartbeat ? "on" : "off");
    return 0;
}

static int cmd_info(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    esp_chip_info_t chip;
    esp_chip_info(&chip);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    printf("board      M5StickC PLUS2\n");
    printf("chip       %s rev %d.%d, %d core(s)\n",
           CONFIG_IDF_TARGET, chip.revision / 100, chip.revision % 100, chip.cores);
    printf("flash      %lu MB\n", (unsigned long)(flash_size / (1024 * 1024)));
    printf("mac(sta)   %02x:%02x:%02x:%02x:%02x:%02x\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("idf        %s\n", esp_get_idf_version());
    printf("ble        %s\n", blex_ready() ? "ready" : "not ready");
    printf("free heap  %lu bytes\n", (unsigned long)esp_get_free_heap_size());
    printf("min heap   %lu bytes\n", (unsigned long)esp_get_minimum_free_heap_size());
    printf("uptime_ms  %lld\n", esp_timer_get_time() / 1000);
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        {.command = "led", .help = "Control the red LED: led on|off|toggle", .func = cmd_led},
        {.command = "ping", .help = "Reply with pong and uptime", .func = cmd_ping},
        {.command = "btn", .help = "Read button states (A / B / power)", .func = cmd_btn},
        {.command = "hb", .help = "Heartbeat log on/off: hb on|off", .func = cmd_hb},
        {.command = "info", .help = "Print chip, flash, MAC, BLE and heap info", .func = cmd_info},
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}

static void heartbeat_task(void *arg)
{
    (void)arg;
    uint32_t n = 0;
    while (true) {
        if (s_heartbeat) {
            ESP_LOGI(TAG, "heartbeat %lu, free heap %lu",
                     (unsigned long)++n, (unsigned long)esp_get_free_heap_size());
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    board_init();
    board_led_blink(2, 120, 120);

    ESP_LOGI(TAG, "wildfire_monitor BLE scanner, IDF %s", esp_get_idf_version());

    esp_err_t err = blex_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed: %s", esp_err_to_name(err));
    }

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "wf> ";
    repl_cfg.max_cmdline_length = 256;
    /* 'probe' nests several commands and every GATT callback runs on the
     * NimBLE host task, but the console task still carries the printf paths. */
    repl_cfg.task_stack_size = 8192;

    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl));

    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_commands();
    cmd_ble_register();

    xTaskCreate(heartbeat_task, "heartbeat", 3072, NULL, 3, NULL);

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
