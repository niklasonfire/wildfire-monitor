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

/* Service Mode, and the update that runs over it. Declared up here
 * because the console reaches both without passing the menu, and main.c owns
 * the sequence either way - taking BLE down is a one-way door wherever the
 * request came from. See "Service Mode" below. `install_now` is what
 * replaces the button press the console cannot give. */
bool        app_service_up(void);
static bool app_service_update(bool install_now);

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
 * suspect release is backed out without publishing anything - and `ota
 * install` is then what puts it on, which is the same operation as going
 * forward.
 *
 * `ota channel <name>` moves the Monitor onto another stream of releases and
 * `ota channel` with no argument puts it back on stable. The rider-facing way
 * to do this is the settings page, which needs nothing but a phone; this one
 * is the bench surface, and it is the only way back that still works if a
 * debug image boots healthy and serves a broken page.
 *
 * `ota check` and `ota install` bring Service Mode up if it is not already
 * and then ask the manifest over it, so they still mean what they always did
 * even though there is now one mode rather than two (#41). Both refuse when
 * the mode fell back to the access point: there is no upstream behind it, and
 * the honest answer to "check" is that there was nothing to ask. Run `wifi
 * on` first and the mode is already up, so these are only the question.
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
    } else if (argc >= 2 && strcmp(argv[1], "channel") == 0) {
        const char *name = (argc >= 3) ? argv[2] : NULL;
        esp_err_t   err  = otaup_channel_set(name);

        if (err != ESP_OK) {
            printf("OTA error=%s\n", esp_err_to_name(err));
            return 1;
        }
    } else if (argc >= 2 && strcmp(argv[1], "check") == 0) {
        if (!app_service_up() || !app_service_update(false)) {
            printf("OTA error=check failed\n");
            return 1;
        }
        return 0;
    } else if (argc >= 2 && strcmp(argv[1], "install") == 0) {
        /* The menu's run without the button press nobody is there to give.
         * This is how the update gets exercised on the bench, where the
         * Monitor is on a cable rather than on a handlebar - and it is the
         * only path that ends in a reboot rather than in a return. */
        if (!app_service_up() || !app_service_update(true)) {
            printf("OTA error=install failed\n");
            return 1;
        }
        return 0;
    } else if (argc >= 2) {
        printf("usage: ota [pin [<tag>]|channel [<name>]|check|install]\n");
        return 1;
    }

    uint32_t gates = ota_health_gates();
    char     chan[WFOTA_CHANNEL_MAX + 1] = "";
    char     pin[40] = "";
    char     url[256] = "";

    otaup_channel_get(chan, sizeof(chan));

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
    printf("rollback   %s\n", ota_health_rolled_back()
           ? ota_rollback_label() : "no");
    printf("channel    %s\n", chan);
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
         .help = "Slot, rollback health, and the update: "
                 "ota [pin [<tag>]|channel [<name>]|check|install]",
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


/* ---- Service Mode -------------------------------------------------------
 *
 * There is one, and entering it is a one-way door. Wi-Fi and NimBLE do not fit
 * in this chip's RAM together, so BLE goes down for good and the way back to
 * capturing is a reboot; and because the radio is the one a Capture is using,
 * the mode refuses while one is running - as does the menu that offers it,
 * which is the earliest point the rider can be told. One predicate for both,
 * so they cannot drift apart.
 *
 * There used to be two of these (ADR-0006). Readout mode put up an access
 * point and served the Captures and the settings page; update mode joined a
 * hotspot, read the manifest and served nothing. A rider had to know which of
 * the two held the settings page, and a rider whose hotspot key had been
 * rotated had to reboot into the other mode to repair it. So they are one, and
 * the link is chosen rather than picked off a menu (#41). Knowing a network,
 * the Monitor scans and joins the strongest of the ones it knows; knowing
 * none, it goes to the access point without scanning at all, because that
 * answer does not depend on what is in range and a fresh Monitor should not
 * wait to be told what it already knows; and failing to join for any reason
 * whatever, it goes to the access point too.
 *
 * The server runs over whichever of the two came up, and that is the point.
 * A rotated key, or an SSID typed with a capital in the wrong place, must not
 * cost a rider the USB lead and an unclipped Monitor: the page that repairs
 * the fault has to be reachable from the fault. The bootstrap falls out of the
 * same rule rather than needing a mechanism of its own - a Monitor that knows
 * no networks lands in its own access point, the rider adds their hotspot on
 * the page that is already there, and every entry after that joins it.
 *
 * What a station link buys on top of that is an upstream, so the update is
 * offered over it and not over the access point, where there is nowhere to
 * fetch from. Nothing about the install itself changes: the rider confirms it
 * on the device, the image is checked against the manifest before the
 * bootloader is pointed at it, and it comes up on probation with
 * ota_health.c's gates in front of it.
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

/* Set once the mode has taken BLE down for the radio. It is what stops button
 * A from offering a capture that can no longer be started. */
static bool s_ble_spent;

/* Menu timings. Up here because the mode's own screens borrow them: a screen
 * the rider walked away from should fall out of the way on the same schedule
 * wherever it came from. */
#define MENU_IDLE_MS    10000   /* long enough to read the list in a helmet */
#define MENU_REFUSE_MS  2500    /* the refusal is a sentence, not a screen */
#define MENU_LABEL_MAX  12      /* "> " plus the longest label, plus the NUL */

/*
 * The three lines the mode leaves on the panel, kept rather than rebuilt
 * because more than one thing paints over them: the update borrows the screen
 * on the way in, and the console can borrow it again minutes later, and each
 * of those has to be able to put the address back afterwards.
 *
 * Lines and not a role and an address, because the two links do not want the
 * same facts in the same order, or even the same three facts. Joined to a
 * hotspot: which network, the address, and how strong the signal is. Being
 * one: the SSID and the key, which have to be read before anything else can
 * happen, and then the address - which leaves the access point no third line
 * for a link state, and rightly so. The state worth knowing there is whether
 * a phone has joined, and a rider who cannot read the key has no way to make
 * one; `wifi` on the console counts them for whoever is debugging it.
 *
 * They are written once, when the mode comes up, and not repainted after. A
 * station link that drops later leaves a dead address on the panel, which is
 * a known cost: watching the link would mean a task, and the rider is
 * standing in front of a Monitor whose page has stopped answering, which
 * says the same thing.
 *
 * Sized for the longest thing that can land in one and not for the panel: an
 * SSID is 32 octets and its NUL, and "http://" with a dotted quad after it is
 * 23. The panel fits 22 characters at scale 1 and clips the rest, which is
 * the right place for that to happen - a line cut short here would be one the
 * rider had no way of telling had been cut.
 */
#define SERVICE_LINE_MAX 40
static char s_service_line[3][SERVICE_LINE_MAX];

static void service_screen(void)
{
    ui_message("SERVICE", s_service_line[0], s_service_line[1],
               s_service_line[2], "A: REBOOT");
}

/* Both run on the radio's own task. ui_message() copies its lines under the UI
 * mutex, so that is safe, and it is the only reason the rider sees "scanning"
 * and "joining" rather than twenty seconds of a frozen panel. */
static void service_stage(const char *l1, const char *l2)
{
    ui_message("SERVICE", l1, l2, NULL, NULL);
}

static void update_stage(const char *l1, const char *l2)
{
    ui_message("UPDATE", l1, l2, NULL, NULL);
}

/* Whichever link the mode landed on. It is one or the other and never both,
 * so asking is cheaper than calling both and letting each decide it has
 * nothing to do. */
static void service_link_stop(void)
{
    if (otaup_running()) {
        otaup_stop();
    } else {
        web_ap_stop();
    }
}

/*
 * Brings the mode up: a link, then the server on it, then the screen the
 * address is read off. Idempotent, because the console can arrive here after
 * the menu already has.
 *
 * `force_ap` skips the station attempt outright. A scan plus a join that times
 * out is twenty-odd seconds, and a rider who already knows there is no hotspot
 * in range would otherwise wait all of it to be told so.
 */
static bool service_enter(bool force_ap)
{
    char      ssid[WFOTA_SSID_MAX + 1] = "", ip[16] = "";
    bool      station = false;
    esp_err_t err;

    if (web_running()) {
        return true;
    }
    if (capture_busy("service mode")) {
        return false;
    }

    ui_message("SERVICE", "starting", NULL, NULL, NULL);
    err = cap_ble_shutdown();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE shutdown: %s", esp_err_to_name(err));
    }
    s_ble_spent = true;

    if (!force_ap) {
        /* Around 500 bytes on the button task's 4 KB stack, and only for the
         * length of this call: what the mode keeps of it is three short
         * lines. otaup_join() answers OTAUP_ERR_NO_NETS without starting the
         * radio, so an empty store costs no scan. */
        otaup_result_t link;
        otaup_err_t    uerr = otaup_join(&link, service_stage);

        if (uerr == OTAUP_OK) {
            station = true;
            snprintf(s_service_line[0], SERVICE_LINE_MAX, "%s", link.ssid);
            snprintf(s_service_line[1], SERVICE_LINE_MAX, "http://%s", link.ip);
            snprintf(s_service_line[2], SERVICE_LINE_MAX, "%d dBm", link.rssi);
        } else {
            /* Logged as a fallback and not as a failure, because that is what
             * it is: every one of these is a reason the access point exists,
             * and none of them is worth a screen the rider has to dismiss on
             * the way to the page that fixes it. */
            ESP_LOGW(TAG, "no network joined (%s, %s), putting up the access "
                          "point instead", otaup_err_str(uerr),
                     link.detail[0] ? link.detail : "-");
            /* Whatever it left behind goes, so the driver is certainly free
             * before the access point asks for it. Every failure inside
             * otaup_join() has already done this; OTAUP_ERR_STATE is the one
             * that has not, and it is the one where something else is holding
             * the radio - which is exactly the case where guessing would put
             * a stale address on the panel. */
            otaup_stop();
        }
    }

    if (!station) {
        err = web_ap_start(ssid, sizeof(ssid), ip, sizeof(ip));
        if (err != ESP_OK) {
            /* Both links refused, so there is nothing further to try and
             * nothing to serve. BLE is already spent, which is why the hint
             * is the only thing left that works. */
            ESP_LOGE(TAG, "no link at all: %s", esp_err_to_name(err));
            ui_message("SERVICE", "no wifi", esp_err_to_name(err), NULL,
                       "A: REBOOT");
            return false;
        }
        snprintf(s_service_line[0], SERVICE_LINE_MAX, "%s", ssid);
        snprintf(s_service_line[1], SERVICE_LINE_MAX, "pw " WEB_PASSWORD);
        snprintf(s_service_line[2], SERVICE_LINE_MAX, "http://%s", ip);
    }

    err = web_serve_start();
    if (err != ESP_OK) {
        /* A link with nothing listening on it is not this mode, so it goes
         * back down rather than sitting there holding heap the rider cannot
         * reach. The update is not offered over it either: an install onto a
         * board that has just failed to allocate a server is not a repair. */
        service_link_stop();
        ui_message("SERVICE", "no server", esp_err_to_name(err), NULL,
                   "A: REBOOT");
        return false;
    }

    service_screen();
    ESP_LOGI(TAG, "service mode up as %s: %s / %s / %s",
             otaup_running() ? "a station" : "an access point",
             s_service_line[0], s_service_line[1], s_service_line[2]);
    return true;
}

/* Takes the mode down again: the server first, then the link under it. Only
 * the console calls this - from the menu the mode is a one-way door and the
 * way out is the reboot the screen offers. */
void app_service_stop(void)
{
    web_serve_stop();
    service_link_stop();
    for (int i = 0; i < 3; i++) {
        s_service_line[i][0] = '\0';
    }
    ui_message_clear();
}

/* `wifi on`: the mode, and nothing beyond it. The update is `ota check`,
 * which is what that command has always been called and still means. One door
 * now, and which of the two names opens it decides only what happens once it
 * is open. */
bool app_service_up(void)
{
    return service_enter(false);
}

/* One line per failure, in the words a rider can act on: which of them it was
 * decides whether they move the bike, open the hotspot on their phone, or go
 * and look at what was published. The second line is whatever detail the check
 * collected - an HTTP status, why the JSON was refused - and it too is words
 * rather than an esp_err_t name, which ota_update.c logs instead. The console
 * is for whoever is debugging this; the panel is for whoever is standing next
 * to the bike. */
static const char *update_fail_line(otaup_err_t err)
{
    switch (err) {
    /* The only way OTAUP_ERR_STATE reaches a panel is the manifest check
     * asked over an access point, so "no upstream" is what it means here.
     * otaup_join()'s other reading of the same code - a link already up -
     * never gets this far, because service_enter() takes that path as a
     * fallback and logs it. */
    case OTAUP_ERR_STATE:    return "no upstream";
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

/* Runs on the install's own task, once per whole percent. Two lines because
 * the percentage alone cannot tell a rider whether a stalled number means a
 * slow hotspot or a stopped one; the byte count keeps moving until it does. */
static void install_stage(int percent, uint32_t got, uint32_t want)
{
    char pct[8];
    char of[24];

    snprintf(pct, sizeof(pct), "%d%%", percent);
    snprintf(of, sizeof(of), "%uk of %uk", (unsigned)(got / 1024),
             (unsigned)(want / 1024));
    ui_message("INSTALL", pct, of, NULL, NULL);
}

/* As with the check: which failure it was decides what the rider does next -
 * move closer to the phone, look at what was published, or reach for the
 * cable. The detail line carries the number that goes with it, and the
 * esp_err_t behind it stays in the log. */
static const char *install_fail_line(otain_err_t err)
{
    switch (err) {
    case OTAIN_ERR_STATE:  return "link gone";
    case OTAIN_ERR_SLOT:   return "no slot";
    case OTAIN_ERR_BEGIN:  return "slot failed";
    case OTAIN_ERR_FETCH:  return "download cut";
    case OTAIN_ERR_SIZE:   return "wrong size";
    case OTAIN_ERR_SHA256: return "bad sha256";
    case OTAIN_ERR_WRITE:  return "write failed";
    case OTAIN_ERR_END:    return "bad image";
    case OTAIN_ERR_BOOT:   return "boot refused";
    case OTAIN_OK:         break;
    }
    return "failed";
}

/*
 * Downloads and installs, and on success does not come back: the Monitor
 * restarts into the new image, which then has sixty seconds and three gates
 * to earn its place before the bootloader takes it away again (ota_health.c).
 *
 * A failure returns false with the message already up, and nothing retries:
 * otadata still points at the running app, so the half-written slot is dead
 * weight, and the rider - who is standing right here - decides whether the
 * hotspot is worth another try (ADR-0006). The link and the server are both
 * still up underneath that message, which is the difference this mode makes:
 * a cut download leaves the settings page in reach rather than taking it
 * away.
 */
static bool app_update_install(const wfota_manifest_t *m)
{
    otain_result_t res;

    ui_message("INSTALL", "0%", NULL, NULL, NULL);
    /* The server comes down for the length of the transfer and goes back up
     * if the transfer did not end in a reboot. Two reasons, and the first is
     * the one that matters: TLS, the image buffer and the OTA write already
     * spend most of the heap that is left with Wi-Fi up, and the httpd's
     * 8 KB task plus four sockets was never part of that budget - under the
     * two old modes the two could not be up at once and nobody had to think
     * about it. The second is the radio: a phone pulling a four-megabyte
     * Capture through the same link as the image is a download made slower
     * for no reason, at the one moment a hotspot too weak to finish costs a
     * whole attempt. */
    web_serve_stop();
    otain_err_t err = otaup_install(m, &res, install_stage);
    if (err != OTAIN_OK) {
        ESP_LOGE(TAG, "install %s: %s (%s)", m->version, otain_err_str(err),
                 res.detail[0] ? res.detail : "-");
        /* Back up over the link the install left alone, so the settings page
         * is in reach of the rider whose download was cut. Failing that, the
         * mode is a link with nothing on it and the reboot the panel already
         * offers is the way out. */
        esp_err_t werr = web_serve_start();
        if (werr != ESP_OK) {
            ESP_LOGE(TAG, "server did not come back: %s",
                     esp_err_to_name(werr));
        }
        ui_message("INSTALL", install_fail_line(err),
                   res.detail[0] ? res.detail : NULL, "not installed",
                   "any: BACK");
        return false;
    }

    ESP_LOGW(TAG, "installed %s into %s, %" PRIu32 " bytes, sha256 %s",
             m->version, res.slot, res.written, res.sha256);
    /* The reboot is the last thing rather than the thing that interrupts the
     * message: the rider gets to read what was installed before the panel
     * goes away, and a Monitor that restarts with no explanation is
     * indistinguishable from one that crashed. */
    ui_message("INSTALL", "DONE", m->version, "rebooting", NULL);
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return true;                /* not reached */
}

/* Nothing installs without this. ADR-0006: never silent and never automatic -
 * the rider chose the mode, and now chooses the image. A screen nobody answers
 * falls away rather than installing by default, which is the same rule the
 * menu itself follows. */
static bool update_confirmed(void)
{
    board_btn_evt_t evt;

    if (!board_btn_wait(&evt, MENU_IDLE_MS)) {
        return false;
    }
    return evt.btn == BOARD_BTN_A_ID && evt.press == BOARD_PRESS_SHORT;
}

/*
 * The update, over the link the mode came up on. It is its own function and
 * not part of service_enter() because it answers a different question - is the
 * published version a different one from this - and because it can be asked
 * again over a link that is already up, which is what `ota check` is for.
 *
 * `install_now` replaces the button press the console cannot give.
 *
 * Every path here ends by putting the address screen back, because the mode
 * outlives the update: whatever the answer was, the rider is left where they
 * were, on a page they can still reach. False means the question could not be
 * answered - no upstream, no manifest - or that an accepted install did not
 * happen. A version that matches is a true: the check worked, and the answer
 * was no.
 */
static bool app_service_update(bool install_now)
{
    otaup_result_t  res;
    board_btn_evt_t evt;

    if (!web_running() || !otaup_running()) {
        /* The access point has no route anywhere, so there is nothing to ask.
         * Only the console gets here - app_service_enter() offers the update
         * only when there is a link to offer it over - so it is a log line
         * and not a screen. A message screen here would paint over the
         * address the rider is reading and then hold the console for ten
         * seconds waiting for a button nobody is standing next to. */
        ESP_LOGW(TAG, "no upstream to check: the Monitor is its own "
                      "access point");
        return false;
    }

    otaup_err_t uerr = otaup_manifest_check(&res, update_stage);
    if (uerr != OTAUP_OK) {
        /* The link is still up and the pages are still served: only the
         * update is off the table, so this is a message and not the end of
         * the mode. */
        ESP_LOGE(TAG, "update: %s (%s)", otaup_err_str(uerr),
                 res.detail[0] ? res.detail : "-");
        ui_message("UPDATE", update_fail_line(uerr),
                   res.detail[0] ? res.detail : NULL, NULL, "any: BACK");
        (void)board_btn_wait(&evt, MENU_IDLE_MS);
        service_screen();
        return false;
    }

    if (!res.differs) {
        ESP_LOGI(TAG, "update: %s is current, from %s at %s", res.running,
                 res.ssid, res.ip);
        /* A beat rather than a screen. There is nothing to decide, and the
         * address the rider came for is waiting underneath it - but the
         * question was worth asking out loud, or a Monitor that looked and a
         * Monitor that did not would look the same. */
        ui_message("UPDATE", "UP TO DATE", res.running, NULL, "any: BACK");
        (void)board_btn_wait(&evt, MENU_REFUSE_MS);
        service_screen();
        return true;
    }

    /* Named, not judged: versions are compared for inequality, so the tag on
     * offer may be older than the one running and installing it is the same
     * operation either way. */
    ESP_LOGI(TAG, "update: %s on offer, running %s", res.manifest.version,
             res.running);
    ui_message("UPDATE", "ON OFFER", res.manifest.version,
               install_now ? "installing" : NULL,
               install_now ? NULL : "A: INSTALL  B: BACK");

    if (!install_now && !update_confirmed()) {
        ESP_LOGI(TAG, "update: %s was offered and not installed",
                 res.manifest.version);
        service_screen();
        return true;
    }

    /* Downloaded through the link the join deliberately left up. This returns
     * only if the install did not happen; otherwise the Monitor is already
     * restarting into the new image. */
    if (app_update_install(&res.manifest)) {
        return true;                /* not reached */
    }
    /* The message on the panel is the install's own. */
    (void)board_btn_wait(&evt, MENU_IDLE_MS);
    service_screen();
    return false;
}

/*
 * What the menu calls, and it does both halves: the rider standing at the bike
 * has no second command to type, so the mode comes up and, if it came up on
 * somebody's network, the update is offered before the screen settles on the
 * address.
 *
 * B still down when A picked the entry forces the access point. In practice
 * that is the opening long-press never released: SERVICE is the first entry,
 * so a rider who holds B and presses A lands on it with the flag set. A
 * second hold inside the menu is not the same thing and does not do this -
 * eight hundred milliseconds of B is the gesture that closes the menu, and
 * that reading has to stay the one it has everywhere else.
 *
 * It is less a gesture to learn than a way out of a wait - a rider in a car
 * park who knows there is no hotspot in range would otherwise sit through a
 * scan and a join timeout to be told so - and it costs one GPIO read, which
 * is the only reason it is here. Nothing depends on anybody finding it: the
 * same rider gets the same access point twenty seconds later by doing nothing
 * at all.
 */
static bool app_service_enter(void)
{
    if (!service_enter(board_btn_b())) {
        return false;
    }
    if (otaup_running()) {
        (void)app_service_update(false);
    }
    return true;
}

/* ---- the info page ------------------------------------------------------
 *
 * Which firmware is this? Until now the only answer was `ota` on the console,
 * which needs the cable ADR-0006 exists to avoid - and the rider who most
 * wants the answer is the one standing at the bike wondering whether the
 * update they just ran is the thing now running. So this is the one menu
 * entry that takes nothing down: no radio, no BLE shutdown, and no refusal
 * while a Capture is recording, because reading a version costs the recording
 * nothing.
 *
 * The version is a `git describe` string, so an untagged build reads
 * "v0.1.0-1-gd6515cc-dirty" - 23 characters where the panel fits 22 at scale
 * 1, and the message screen clips rather than wraps. A version that has
 * quietly lost its "-dirty" is worse than no version at all, so it is split
 * here: after a '-' where one falls within reach, so both halves read as
 * pieces of one string rather than as a word cut in half. Two lines cover
 * every case because esp_app_desc_t caps the version at 31 characters.
 *
 * The running slot goes under it. It is one line from the same API and it is
 * the one thing the version does not say: the same tag reaches ota_0 over a
 * cable and ota_1 over the air. Nothing else belongs here - this is a label,
 * not the `ota` dump.
 */
#define INFO_COLS ((size_t)(DISP_W / DISP_CHAR_W(1)))   /* 22 at scale 1 */

static bool app_info_enter(void)
{
    const char *ver = esp_app_get_description()->version;
    size_t      len = strlen(ver);
    size_t      cut = len;
    char        v1[INFO_COLS + 1], v2[INFO_COLS + 1];
    char        slot[INFO_COLS + 1];
    char        chan[WFOTA_CHANNEL_MAX + 1];

    if (len > INFO_COLS) {
        cut = INFO_COLS;        /* the fallback: a break mid-token still reads */
        /* Back from the last character that fits, looking for a '-' to break
         * after, and no further back than leaves a tail that still fits. */
        for (size_t i = INFO_COLS; i > 0 && len - i <= INFO_COLS; i--) {
            if (ver[i - 1] == '-') {
                cut = i;
                break;
            }
        }
    }
    snprintf(v1, sizeof(v1), "%.*s", (int)cut, ver);
    snprintf(v2, sizeof(v2), "%s", ver + cut);
    /* The channel rides on the slot line rather than taking one of its own:
     * a wrapped version can want two of the three, and a Monitor that is not
     * on stable is exactly the one whose INFO screen has to say so. Stable is
     * left unsaid, because it is what every other Monitor is. */
    otaup_channel_get(chan, sizeof(chan));
    if (strcmp(chan, WFOTA_CHANNEL_STABLE) == 0) {
        snprintf(slot, sizeof(slot), "slot %s", ota_running_label());
    } else {
        snprintf(slot, sizeof(slot), "slot %s  %s", ota_running_label(), chan);
    }

    /* Packed rather than placed: draw_message() holds a line's y position even
     * when it is empty, so a short version that needs no second line must not
     * leave a gap where the wrap would have been. */
    const char *line[3] = {v1, NULL, NULL};
    int         n = 1;

    if (v2[0]) {
        line[n++] = v2;
    }
    line[n] = slot;
    ui_message("INFO", line[0], line[1], line[2], "any: BACK");
    ESP_LOGI(TAG, "info: %s from %s on %s", ver, ota_running_label(), chan);

    board_btn_evt_t evt;
    if (!board_btn_wait(&evt, MENU_IDLE_MS)) {
        /* Nobody is reading it: fall out of the way on the menu's own
         * schedule, and do not put the menu back in front of an absent
         * rider. */
        ui_message_clear();
        return true;
    }
    /* Dismissed by hand, so the rider is still there and the menu is where
     * they were. menu_run() paints straight over this screen, so there is
     * nothing to clear first. */
    return false;
}

/* ---- the B menu ---------------------------------------------------------
 *
 * Two buttons cannot carry a gesture per screen, so held-B stopped being the
 * readout shortcut and became the way in to a list (ADR-0006). Service Mode
 * costs one keypress more than readout mode used to; that keypress is what
 * buys somewhere to put INFO, and whatever follows it, without inventing a
 * third gesture nobody would remember. The list is shorter than it was - the
 * two Wi-Fi entries are one (#41) - and the keypress is still worth it,
 * because a menu that shrinks back to one entry is a menu that has to be
 * invented again the next time something needs a place to live.
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
    /* A page hands the panel back rather than keeping it, so a false from one
     * means "the rider dismissed it" and the menu returns. The flag is what
     * keeps that reading off Service Mode, which returns false only when it
     * failed - with its own screen up and BLE already spent, so painting the
     * menu over it would hide the only explanation there is. */
    bool        page;
} menu_entry_t;

static const menu_entry_t s_menu[] = {
    {"SERVICE", app_service_enter, false},
    {"INFO",    app_info_enter,    true},
};
#define MENU_COUNT ((int)(sizeof(s_menu) / sizeof(s_menu[0])))

/* The message screen takes four lines, and the fourth is the button hint, so
 * three entries is the ceiling before this needs a scrolling window. There are
 * two today. A fourth is not a table edit: it is a scrolling window, or a
 * second screen, and the assert is here to make that a decision rather than a
 * surprise on the panel.
 *
 * The rows are padded to a common width for the same reason the cursor is two
 * characters wide: fit_scale() in ui.c drops from 3 to 2 above seven
 * characters, so "> INFO" would be drawn half again as large as "> SERVICE"
 * sitting under it. */
_Static_assert(MENU_COUNT <= 3, "the message screen has three lines for entries");

static void menu_draw(int sel)
{
    char        row[3][MENU_LABEL_MAX];
    const char *line[3] = {NULL, NULL, NULL};

    for (int i = 0; i < MENU_COUNT; i++) {
        snprintf(row[i], sizeof(row[i]), "%c %-7s", i == sel ? '>' : ' ',
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
                /* For Service Mode the menu is over: it owns the panel
                 * from here and never gives it back, and a failure has its own
                 * screen to show. A page that the rider dismissed is the one
                 * case that comes back to the list, so INFO does not cost a
                 * second long-press of B to leave. */
                if (s_menu[sel].enter() || !s_menu[sel].page) {
                    return;
                }
                menu_draw(sel);
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
 * button a rider can find with a glove on. B is the Marker while a Capture is
 * recording and, held, the menu, and nothing at all otherwise. PWR short is
 * otherwise idle time on a button already under the thumb for power-off, so
 * it toggles the live telemetry screen.
 */
static uint32_t s_markers;

/* Set by app_main() when the boot after a rollback put the channel back to
 * stable. The panel says so instead of saying the update failed, because
 * those are two different things for a rider to know and there is one line
 * to say either of them in. */
static bool s_chan_reverted;

static void button_task(void *arg)
{
    (void)arg;

    /* ADR-0006: a rollback is the point of the two slots, and a rollback the
     * rider is not told about looks exactly like an update that quietly did
     * nothing. Said here rather than in app_main() because this is the task
     * that owns the button queue, so the screen can be dismissed the way
     * every other message screen is. */
    if (ota_health_rolled_back()) {
        board_btn_evt_t first;

        ESP_LOGW(TAG, "came up on %s after a rollback from %s",
                 ota_running_label(), ota_rollback_label());
        ui_message("UPDATE", "ROLLED BACK", esp_app_get_description()->version,
                   s_chan_reverted ? "back to stable" : "update failed",
                   "any: BACK");
        (void)board_btn_wait(&first, MENU_IDLE_MS);
        ui_message_clear();
    }

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
            continue;
        }

        if (s_ble_spent && evt.btn == BOARD_BTN_A_ID) {
            /* Service Mode has been through here and failed to come up - the
             * branch above catches it when it did - so BLE is gone and A has
             * no capture left to start. Rebooting is the only thing that gets
             * one back, and it is what A already means on the mode's own
             * screen. B still opens the menu, so the mode can be tried
             * again. */
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
                 * undecoded bytes into labelled sections. */
                char text[32];
                snprintf(text, sizeof(text), "marker %" PRIu32, ++s_markers);
                cap_marker(text);
            }
            /* And otherwise nothing, deliberately. B-short used to toggle the
             * backlight, which put an unlit panel one accidental press away
             * from a rider who then had no way of knowing the Monitor was
             * still alive - and the same press is the Marker as soon as a
             * Capture starts, so the button meant two unrelated things. The
             * backlight is still `disp on|off` on the console, where nothing
             * brushes against it. */
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

    /*
     * A rolled-back image takes its channel down with it, once. The update
     * that failed is the only evidence there is that this Monitor should not
     * be on that stream, and the way back has to need nothing from the rider,
     * who may be a long way from a cable. Erasing the trial is what makes it
     * once: a rider who picks the channel again afterwards keeps it, because
     * there is then nothing outstanding for the next rollback to undo.
     */
    char trial[WFOTA_CHANNEL_MAX + 1];
    if (ota_health_rolled_back() && otaup_trial_get(trial, sizeof(trial))) {
        esp_err_t rerr = otaup_channel_set(NULL);

        otaup_trial_clear();
        if (rerr == ESP_OK) {
            ESP_LOGW(TAG, "the update that rolled back came from the %s "
                          "channel - back to %s", trial, WFOTA_CHANNEL_STABLE);
            s_chan_reverted = true;
        } else {
            ESP_LOGE(TAG, "could not go back to %s: %s", WFOTA_CHANNEL_STABLE,
                     esp_err_to_name(rerr));
        }
    }

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
