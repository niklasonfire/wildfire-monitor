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
#include "ota_health.h"
#include "ota_update.h"
#include "rtc_bm8563.h"
#include "ui.h"
#include "webdump.h"

#include "esp_app_desc.h"
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
    printf("app slot   %s%s\n", ota_running_label(),
           ota_health_on_probation() ? " (on probation)" : "");
    printf("idf        %s\n", esp_get_idf_version());
    printf("ble        %s\n", blex_ready() ? "ready" : "not ready");
    printf("free heap  %lu bytes\n", (unsigned long)esp_get_free_heap_size());
    printf("min heap   %lu bytes\n", (unsigned long)esp_get_minimum_free_heap_size());
    printf("uptime_ms  %lld\n", esp_timer_get_time() / 1000);
    return 0;
}

/* Update mode from the console. Takes BLE down and blocks for as long as the
 * check runs, exactly as "wifi on" does, because it is the same one-way door;
 * main.c owns the sequence because the menu goes through it too. */
static bool app_update_enter(void);

/*
 * The health check runs whether or not anything is on probation (see
 * ota_health.h), so plain `ota` is what makes it observable on a board that
 * has only ever been flashed down a cable: watch the gates fill in, watch
 * "confirmed" flip a minute in. It also prints the URL the next check will
 * read, which is the one thing the pin changes.
 *
 * `ota pin <tag>` points the Monitor at one release instead of at `latest`,
 * and `ota pin` with no argument clears it. Because versions are compared for
 * inequality and never for order (ADR-0006), pinning an older tag is how a
 * suspect release is backed out without publishing anything.
 */
static int cmd_ota(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "pin") == 0) {
        const char *tag = (argc >= 3) ? argv[2] : NULL;
        esp_err_t   err = otaup_pin_set(tag);

        if (err != ESP_OK) {
            printf("OTA error=%s\n", esp_err_to_name(err));
            return 1;
        }
    } else if (argc >= 2 && strcmp(argv[1], "check") == 0) {
        if (!app_update_enter()) {
            printf("OTA error=check failed\n");
            return 1;
        }
        return 0;
    } else if (argc >= 2) {
        printf("usage: ota [pin [<tag>]|check]\n");
        return 1;
    }

    uint32_t gates = ota_health_gates();
    char     pin[40] = "";
    char     url[256] = "";

    printf("running    %s\n", ota_running_label());
    printf("version    %s\n", esp_app_get_description()->version);
    printf("next boot  %s\n", ota_boot_label());
    printf("probation  %s\n", ota_health_on_probation() ? "yes" : "no");
    printf("gates      nvs=%d store=%d display=%d\n",
           (gates & OTA_GATE_NVS) != 0, (gates & OTA_GATE_STORE) != 0,
           (gates & OTA_GATE_DISPLAY) != 0);
    printf("uptime_s   %lld of %d\n", esp_timer_get_time() / 1000000,
           OTA_HEALTH_UPTIME_S);
    printf("confirmed  %s\n", ota_health_confirmed() ? "yes" : "no");
    printf("pin        %s\n", otaup_pin_get(pin, sizeof(pin)) ? pin : "(latest)");
    printf("manifest   %s\n", otaup_manifest_url(url, sizeof(url)) ? url : "-");
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
        {.command = "ota",
         .help = "Slot, rollback health, and update mode: ota [pin [<tag>]|check]",
         .func = cmd_ota},
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


/* ---- the modes behind the menu ------------------------------------------
 *
 * Readout mode and update mode share a shape: one radio at a time, entered
 * deliberately by the rider, left by a reboot. Both take the radio the capture
 * needs, so both refuse while one is running - and so does the menu that
 * offers them, which is the earliest point the rider can be told. One
 * predicate for all three, so they cannot drift apart.
 */
static bool capture_busy(const char *what)
{
    cap_state_t st = cap_state();

    if (st != CAP_RECORDING && st != CAP_CONNECTING) {
        return false;
    }
    ESP_LOGW(TAG, "refusing %s while a capture is running", what);
    return true;
}

/* Set once either mode has taken BLE down for the radio. It is what stops
 * button A from offering a capture that can no longer be started. */
static bool s_ble_spent;

/* Wi-Fi and NimBLE do not fit in this chip's RAM together, so entering readout
 * mode is a one-way door: BLE goes down for good and the way back to capturing
 * is a reboot. Both the menu and the "wifi on" command land here.
 */
bool app_readout_enter(void)
{
    if (web_running()) {
        return true;
    }
    if (capture_busy("readout mode")) {
        return false;
    }

    ui_message("WIFI", "starting", NULL, NULL, NULL);
    esp_err_t err = cap_ble_shutdown();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE shutdown: %s", esp_err_to_name(err));
    }
    s_ble_spent = true;

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

/* Menu timings. Up here because the update screen below borrows the idle one:
 * a screen the rider walked away from should fall out of the way on the same
 * schedule wherever it came from. */
#define MENU_IDLE_MS    10000   /* long enough to read two entries in a helmet */
#define MENU_REFUSE_MS  2500    /* the refusal is a sentence, not a screen */
#define MENU_LABEL_MAX  12      /* "> " plus the longest label, plus the NUL */

/* ---- update mode --------------------------------------------------------
 *
 * It shares readout mode's shape (ADR-0006): BLE goes down so the radio is
 * free, the Monitor joins the strongest hotspot it knows, and the only way
 * back to capturing is a reboot. What it does with the link is one question -
 * is there something newer than this - and it answers it and stops. The
 * download and the install are issue #28.
 *
 * The capture check is repeated rather than left to the menu because ADR-0006
 * asks update mode itself to refuse, exactly as readout mode does, and because
 * `ota check` on the console comes here without passing the menu at all.
 *
 * Unlike readout mode this is not a screen the rider is stranded on: a failure
 * is left failed, says which failure it was, and hands the panel back. Nothing
 * retries by itself - the rider is standing next to the Monitor, and a retry
 * loop would only hide a hotspot too weak to finish.
 */
/* Runs on the check's own task. ui_message() copies its lines under the UI
 * mutex, so this is safe from there, and it is the only reason the rider sees
 * "scanning" and "joining" rather than ten seconds of a frozen panel. */
static void update_stage(const char *l1, const char *l2)
{
    ui_message("UPDATE", l1, l2, NULL, NULL);
}

/* One line per failure, in the words a rider can act on: which of them it was
 * decides whether they move the bike, open the hotspot on their phone, or go
 * and look at what was published. The second line is whatever detail the check
 * collected - a disconnect reason, an HTTP status, why the JSON was refused. */
static const char *update_fail_line(otaup_err_t err)
{
    switch (err) {
    case OTAUP_ERR_STATE:    return "already on";
    case OTAUP_ERR_WIFI:     return "radio failed";
    case OTAUP_ERR_NO_NETS:  return "no networks";
    case OTAUP_ERR_NO_SCAN:  return "no hotspot";
    case OTAUP_ERR_NO_KNOWN: return "none known";
    case OTAUP_ERR_JOIN:     return "join failed";
    case OTAUP_ERR_FETCH:    return "no manifest";
    case OTAUP_ERR_MANIFEST: return "bad manifest";
    case OTAUP_OK:           break;
    }
    return "failed";
}

static bool app_update_enter(void)
{
    otaup_result_t res;

    if (capture_busy("update mode")) {
        return false;
    }

    ui_message("UPDATE", "starting", NULL, NULL, NULL);
    esp_err_t err = cap_ble_shutdown();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE shutdown: %s", esp_err_to_name(err));
    }
    s_ble_spent = true;

    otaup_err_t uerr = otaup_check(&res, update_stage);
    if (uerr != OTAUP_OK) {
        ESP_LOGE(TAG, "update mode: %s (%s)", otaup_err_str(uerr),
                 res.detail[0] ? res.detail : "-");
        ui_message("UPDATE", update_fail_line(uerr),
                   res.detail[0] ? res.detail : NULL,
                   (uerr == OTAUP_ERR_NO_NETS) ? "use wifi add" : NULL,
                   "any: BACK");
    } else if (!res.differs) {
        ESP_LOGI(TAG, "update mode: %s is current, from %s at %s",
                 res.running, res.ssid, res.ip);
        ui_message("UPDATE", "UP TO DATE", res.running, res.ip, "any: BACK");
    } else {
        /* Named, not judged: versions are compared for inequality, so the tag
         * on offer may be older than the one running and installing it is the
         * same operation either way. */
        ESP_LOGI(TAG, "update mode: %s on offer, running %s",
                 res.manifest.version, res.running);
        ui_message("UPDATE", "ON OFFER", res.manifest.version, res.ip,
                   "any: BACK");
    }

    board_btn_evt_t evt;
    (void)board_btn_wait(&evt, MENU_IDLE_MS);
    ui_message_clear();
    /* The link goes down with the screen. #28 is where it stays up, because
     * that is the ticket with something to download through it. */
    otaup_stop();
    return uerr == OTAUP_OK;
}

/* ---- the B menu ---------------------------------------------------------
 *
 * Two buttons cannot carry a gesture per mode, so held-B stopped being the
 * readout shortcut and became the way in to a list (ADR-0006). Readout mode
 * costs one keypress more than it did; that keypress is what buys somewhere
 * to put update mode, and whatever follows it, without inventing a third
 * gesture nobody would remember.
 *
 * The menu runs inside the button task rather than as a screen the task posts
 * to: it is a loop that reads the same queue with a timeout instead of
 * forever, so which button means what while it is up needs no mode flag that
 * the rest of the task then has to test. The timeout is the whole trick - a
 * menu opened by a pocket press must fall out of the way on its own, because
 * a rider who has just noticed a menu they did not ask for is looking at the
 * road, not at the panel.
 *
 * B cycles and A picks: B is the button that opened the menu, so the thumb is
 * already there, and A - the button that means "do the thing" everywhere else
 * in this firmware - keeps meaning it here.
 */
typedef struct {
    const char *label;
    bool      (*enter)(void);   /* true once the panel is somebody else's */
} menu_entry_t;

static const menu_entry_t s_menu[] = {
    {"READOUT", app_readout_enter},
    {"UPDATE",  app_update_enter},
};
#define MENU_COUNT ((int)(sizeof(s_menu) / sizeof(s_menu[0])))

/* The message screen takes four lines, and the fourth is the button hint, so
 * three entries is the ceiling before this needs a scrolling window. The
 * two-character cursor prefix also keeps every row at the same text scale:
 * fit_scale() in ui.c drops from 3 to 2 above seven characters, so a short
 * label and a long one would otherwise be drawn at different sizes. */
_Static_assert(MENU_COUNT <= 3, "the message screen has three lines for entries");

static void menu_draw(int sel)
{
    char        row[3][MENU_LABEL_MAX];
    const char *line[3] = {NULL, NULL, NULL};

    for (int i = 0; i < MENU_COUNT; i++) {
        snprintf(row[i], sizeof(row[i]), "%c %s", i == sel ? '>' : ' ',
                 s_menu[i].label);
        line[i] = row[i];
    }
    ui_message("MENU", line[0], line[1], line[2], "B: NEXT  A: PICK");
}

static void menu_run(void)
{
    if (capture_busy("the menu")) {
        /* On screen and not only in the log: the rider pressed something and
         * has to be told why nothing happened. Dismissable, because the
         * recording screen is the one they actually want back. */
        ui_message("BUSY", "capture", "running", NULL, "any: BACK");
        board_btn_evt_t busy_evt;
        (void)board_btn_wait(&busy_evt, MENU_REFUSE_MS);
        ui_message_clear();
        return;
    }

    int sel = 0;
    menu_draw(sel);

    while (true) {
        board_btn_evt_t evt;

        if (!board_btn_wait(&evt, MENU_IDLE_MS)) {
            break;              /* idle: leave, rather than trap the rider */
        }

        switch (evt.btn) {
        case BOARD_BTN_B_ID:
            if (evt.press == BOARD_PRESS_SHORT) {
                sel = (sel + 1) % MENU_COUNT;
                menu_draw(sel);
            } else {
                /* Held B is the gesture that opened this, so holding it again
                 * closes it. The press that got here fired its LONG while
                 * still down and stays silent on release, so this cannot see
                 * the opening gesture and shut the menu straight again. */
                goto leave;
            }
            break;

        case BOARD_BTN_A_ID:
            if (evt.press == BOARD_PRESS_SHORT) {
                /* Whichever entry this is, the menu is over: the mode it
                 * starts owns the panel from here - readout mode never gives
                 * it back, and a failure has its own screen to show. */
                (void)s_menu[sel].enter();
                return;
            }
            break;

        case BOARD_BTN_PWR_ID:
            if (evt.press == BOARD_PRESS_LONG) {
                /* Power off has to work from every screen, menu included. */
                cap_record_stop();
                board_power_off();
            }
            goto leave;         /* short PWR is the cancel */
        }
    }

leave:
    ui_message_clear();
}

/* ---- buttons ------------------------------------------------------------
 *
 * A is the whole capture workflow - scan, start, stop - because it is the one
 * button a rider can find with a glove on. B is the display and, held, the
 * menu. PWR short is otherwise idle time on a button already under the thumb
 * for power-off, so it toggles the live telemetry screen.
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

        if (s_ble_spent && evt.btn == BOARD_BTN_A_ID) {
            /* Update mode has been through here, so BLE is gone and A has no
             * capture left to start. Rebooting is the only thing that gets one
             * back, and it is what A already means on the readout screen. B
             * still opens the menu, so update mode can be run again. */
            if (evt.press == BOARD_PRESS_SHORT) {
                esp_restart();
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
                menu_run();
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
            } else {
                /* Short PWR was otherwise unbound, and it is the one button
                 * free for a screen the capture workflow doesn't own: live
                 * Fardriver telemetry, toggled over whatever A's state screen
                 * is currently showing. */
                ui_live_toggle();
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

    /* First, so the log opens with which slot this is and whether it is on
     * probation, and so the gates reported below have somewhere to land. */
    ota_health_start();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ota_health_pass(OTA_GATE_NVS);

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
    } else {
        ota_health_pass(OTA_GATE_STORE);
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
