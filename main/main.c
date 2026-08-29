/*
 * wildfire_monitor - BLE scanner for the Blacktea Wildfire electric motorbike.
 *
 * Stage one of the project: find the Fardriver motor controller and the Daly
 * BMS, connect to both and pull out everything that a later, permanent
 * connection will need - handles, MTU, connection parameters, which
 * characteristics stream data on their own and which have to be polled.
 *
 * Two ways to drive it. Tethered, a console on UART0 (115200 8N1) lets a
 * script on the host run a capture; see cmd_ble.c. Untethered - which is the
 * only way to capture while the bike is actually moving - the two buttons and
 * the LCD drive it and the frames go to flash; see capture.c, ui.c and the
 * button task below.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ble_explorer.h"
#include "board.h"
#include "capture.h"
#include "capture_store.h"
#include "display.h"
#include "i2c_bus.h"
#include "imu.h"
#include "rtc_bm8563.h"
#include "ui.h"
#include "webdump.h"

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
#include "nvs_flash.h"

void cmd_ble_register(void);
void cmd_cap_register(void);

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

/* A scripted session sometimes has to wait for something the board does on
 * its own - a scan filling up, a capture running - and every fresh ./wf.sh
 * invocation would reset the board and throw that state away. */
static int cmd_sleep(int argc, char **argv)
{
    int secs = (argc >= 2) ? atoi(argv[1]) : 1;
    if (secs < 0 || secs > 600) {
        printf("usage: sleep <0..600>\n");
        return 1;
    }
    vTaskDelay(pdMS_TO_TICKS(secs * 1000));
    printf("slept %d s\n", secs);
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
        {.command = "sleep", .help = "Wait, so a script can let the board work: sleep <secs>", .func = cmd_sleep},
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


/* ---- readout mode -------------------------------------------------------
 *
 * Wi-Fi and NimBLE do not fit in this chip's RAM together, so entering readout
 * mode is a one-way door: BLE goes down for good and the way back to capturing
 * is a reboot. Both the B button and the "wifi on" command land here.
 */
bool app_readout_enter(void)
{
    if (web_running()) {
        return true;
    }
    if (cap_state() == CAP_RECORDING || cap_state() == CAP_CONNECTING) {
        ESP_LOGW(TAG, "refusing readout mode while a capture is running");
        return false;
    }

    ui_message("WIFI", "starting", NULL, NULL, NULL);
    esp_err_t err = cap_ble_shutdown();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE shutdown: %s", esp_err_to_name(err));
    }

    char ssid[40] = "", ip[24] = "";
    err = web_start(ssid, sizeof(ssid), ip, sizeof(ip));
    if (err != ESP_OK) {
        ui_message("WIFI", "failed", esp_err_to_name(err), NULL, "A: REBOOT");
        return false;
    }

    char line_ip[32];
    snprintf(line_ip, sizeof(line_ip), "http://%s", ip);
    ui_message("READOUT", ssid, "pw " WEB_PASSWORD, line_ip, "A: REBOOT");
    ESP_LOGI(TAG, "readout mode: ssid=%s pass=%s url=http://%s", ssid,
             WEB_PASSWORD, ip);
    return true;
}

/* ---- buttons ------------------------------------------------------------
 *
 * A is the whole capture workflow - scan, start, stop - because it is the one
 * button a rider can find with a glove on. B is the display and, held, the
 * readout mode.
 */
static uint32_t s_markers;

static void button_task(void *arg)
{
    (void)arg;

    board_btn_evt_t evt;
    while (true) {
        if (!board_btn_wait(&evt, -1)) {
            continue;
        }

        if (web_running()) {
            /* Nothing to capture any more; the only useful action left is to
             * get the firmware back into a state that can. */
            if (evt.btn == BOARD_BTN_A_ID) {
                esp_restart();
            }
            if (evt.btn == BOARD_BTN_B_ID) {
                disp_backlight(!disp_backlight_get());
            }
            continue;
        }

        switch (evt.btn) {
        case BOARD_BTN_A_ID:
            if (evt.press != BOARD_PRESS_SHORT) {
                break;
            }
            switch (cap_state()) {
            case CAP_ARMED:
                cap_record_start();
                break;
            case CAP_RECORDING:
            case CAP_CONNECTING:
                /* Aborting a half-built link is the same operation as stopping
                 * a running capture: drop both links, close whatever is open. */
                cap_record_stop();
                break;
            case CAP_SCANNING:
                cap_scan_stop();
                break;
            case CAP_STOPPING:
                /* The file is being closed; a button press here would only
                 * risk truncating it. The screen says so. */
                break;
            default:
                cap_scan_start();
                break;
            }
            break;

        case BOARD_BTN_B_ID:
            if (evt.press == BOARD_PRESS_LONG) {
                app_readout_enter();
            } else if (cap_state() == CAP_RECORDING) {
                /* While recording, B is the marker: the rider rides a defined
                 * manoeuvre and stamps it, which is what turns a stream of
                 * undecoded bytes into labelled sections. The backlight is not
                 * worth the button here - a marker is. */
                char text[32];
                snprintf(text, sizeof(text), "marker %" PRIu32, ++s_markers);
                cap_marker(text);
            } else {
                /* The backlight is the biggest single draw on this board, so
                 * turning it off is what makes a long ride possible. */
                disp_backlight(!disp_backlight_get());
                ui_redraw();
            }
            break;

        case BOARD_BTN_PWR_ID:
            if (evt.press == BOARD_PRESS_LONG) {
                cap_record_stop();
                board_power_off();
            }
            break;
        }
    }
}

void app_main(void)
{
    board_init();
    board_led_blink(2, 120, 120);

    ESP_LOGI(TAG, "wildfire_monitor, IDF %s", esp_get_idf_version());

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* The screen comes up first: everything after this can report its own
     * failure to the rider instead of only to a serial log nobody is reading. */
    if (ui_init() != ESP_OK) {
        ESP_LOGE(TAG, "display init failed");
    }

    /* The RTC and the IMU share the internal I2C bus, so it is created once
     * here before either of them tries to claim the port. */
    if (i2c_bus_init() != ESP_OK) {
        ESP_LOGE(TAG, "internal I2C bus unavailable - no RTC, no IMU");
    }

    /* The RTC is what stamps a capture with a wall clock. It keeps time on its
     * own backup cell, so a ride hours after the last USB session is still
     * dated - unless it was never set, which bm8563_valid() reports. */
    if (bm8563_init() == ESP_OK && bm8563_valid()) {
        bm8563_sync_system_time();
    } else {
        ESP_LOGW(TAG, "RTC unset or absent - captures will be stamped unknown");
    }

    /* Without the IMU a capture is still valid, just harder to interpret: the
     * frames then have no independent movement signal to be correlated with. */
    if (imu_init() != ESP_OK) {
        ESP_LOGW(TAG, "IMU absent - captures will carry no movement reference");
    }

    err = store_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "capture store: %s", esp_err_to_name(err));
    }

    err = blex_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed: %s", esp_err_to_name(err));
    }

    err = cap_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "capture init failed: %s", esp_err_to_name(err));
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
    cmd_cap_register();

    xTaskCreate(heartbeat_task, "heartbeat", 3072, NULL, 3, NULL);

    board_buttons_start();
    xTaskCreate(button_task, "buttons", 4096, NULL, 4, NULL);
    ui_start();

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
