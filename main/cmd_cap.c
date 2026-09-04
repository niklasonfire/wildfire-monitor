/*
 * Console commands for the standalone capture.
 *
 * The capture itself is driven by the two buttons on the bike; these commands
 * exist for the bench, where the board is on USB anyway: arm it by hand, look
 * at what is on the flash, pull a capture off over the serial link when Wi-Fi
 * is not an option, and set the RTC - which is the one thing the board cannot
 * do for itself, and without which every capture is stamped "unknown".
 *
 * Output follows the same tagged single-line convention as cmd_ble.c so the
 * host scripts can parse it.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "board.h"
#include "capture.h"
#include "capture_store.h"
#include "display.h"
#include "rtc_bm8563.h"
#include "ui.h"
#include "webdump.h"
#include "wifi_store.h"

#include "esp_console.h"
#include "esp_err.h"

/* Entering the Wi-Fi mode takes BLE down, chooses a link and puts the address
 * on the LCD; main.c owns that sequence because the buttons trigger it too. */
bool app_service_up(void);
void app_service_stop(void);

static void print_status(void)
{
    cap_status_t st;
    cap_status(&st);

    printf("CAP state=%s seq=%d file=%s elapsed_ms=%" PRIu32
           " frames=%" PRIu32 " dropped=%" PRIu32 " bytes=%llu\n",
           cap_state_str(st.state), st.seq, st.file[0] ? st.file : "-",
           st.elapsed_ms, st.frames, st.dropped, (unsigned long long)st.bytes);
    for (int i = 0; i < CAP_LINK_COUNT; i++) {
        const cap_link_status_t *l = &st.link[i];
        printf("CAPLINK %s seen=%d conn=%d sub=%d addr=%s name=%s rssi=%d "
               "frames=%" PRIu32 " reconn=%" PRIu32 "\n",
               i == CAP_LINK_MCU ? "mcu" : "bms", l->seen, l->connected,
               l->subscribed, l->addr[0] ? l->addr : "-",
               l->name[0] ? l->name : "-", l->rssi, l->frames, l->reconnects);
    }
    if (st.err[0]) {
        printf("CAPERR %s\n", st.err);
    }
}

static int cmd_cap(int argc, char **argv)
{
    if (argc < 2 || strcmp(argv[1], "status") == 0) {
        print_status();
        return 0;
    }

    esp_err_t err = ESP_OK;
    if (strcmp(argv[1], "scan") == 0) {
        err = cap_scan_start();
    } else if (strcmp(argv[1], "idle") == 0) {
        err = cap_scan_stop();
    } else if (strcmp(argv[1], "rec") == 0) {
        err = cap_record_start();
    } else if (strcmp(argv[1], "stop") == 0) {
        err = cap_record_stop();
    } else if (strcmp(argv[1], "mark") == 0) {
        /* The same marker the B button writes, for a bench run where the
         * console is the only input. */
        err = cap_marker(argc >= 3 ? argv[2] : "marker");
    } else {
        printf("usage: cap [status|scan|idle|rec|stop|mark [text]]\n");
        return 1;
    }

    if (err != ESP_OK) {
        printf("CAP error=%s\n", esp_err_to_name(err));
    }
    print_status();
    return err == ESP_OK ? 0 : 1;
}

static void print_entry(const store_entry_t *e)
{
    char when[24] = "unknown";
    if (e->unix_start > 0) {
        time_t t = (time_t)e->unix_start;
        struct tm tm_utc;
        gmtime_r(&t, &tm_utc);
        strftime(when, sizeof(when), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
    }
    printf("CAPF seq=%d name=%s size=%" PRIu32 " dur_ms=%" PRIu32 " start=%s\n",
           e->seq, e->name, e->size, e->duration_ms, when);
}

static int cmd_caps(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (!store_ready()) {
        printf("store not mounted\n");
        return 1;
    }

    static store_entry_t entries[STORE_MAX_FILES];
    int n = store_list(entries, STORE_MAX_FILES);
    if (n < 0) {
        printf("store_list failed: %d\n", n);
        return 1;
    }
    for (int i = 0; i < n; i++) {
        print_entry(&entries[i]);
    }

    uint64_t total = 0, freeb = 0;
    store_space(&total, &freeb);
    printf("CAPFS count=%d total_kb=%llu free_kb=%llu\n", n,
           (unsigned long long)(total / 1024), (unsigned long long)(freeb / 1024));
    return 0;
}

static int cmd_caprm(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: caprm <seq>|all\n");
        return 1;
    }
    esp_err_t err = (strcmp(argv[1], "all") == 0) ? store_remove_all()
                                                  : store_remove(atoi(argv[1]));
    printf("CAPRM %s %s\n", argv[1], esp_err_to_name(err));
    return err == ESP_OK ? 0 : 1;
}

/* Serial fallback for pulling a capture off the board. At 115200 baud this
 * runs at roughly 11 KB/s of hex, so a full ride takes minutes - the Wi-Fi
 * mode exists precisely to avoid it. */
static int cmd_capdump(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: capdump <seq> [max_records]\n");
        return 1;
    }
    long max_recs = (argc >= 3) ? strtol(argv[2], NULL, 0) : 0;

    char path[64];
    store_path(atoi(argv[1]), path, sizeof(path));
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        printf("cannot open %s\n", path);
        return 1;
    }

    wflog_hdr_t hdr;
    if (fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr) ||
        memcmp(hdr.magic, WFLOG_MAGIC, strlen(WFLOG_MAGIC)) != 0) {
        printf("bad header in %s\n", path);
        fclose(f);
        return 1;
    }
    printf("CAPHDR seq=%" PRIu32 " version=%u unix_start=%lld boot_ms=%" PRIu32
           " mcu=%s bms=%s note=%s\n",
           hdr.seq, hdr.version, (long long)hdr.unix_start, hdr.boot_ms,
           hdr.mcu_addr, hdr.bms_addr, hdr.note);

    /* Header length is stored so a newer header can be skipped by an older
     * reader; honour it rather than assuming sizeof() matches. */
    if (hdr.hdr_len != sizeof(hdr)) {
        fseek(f, hdr.hdr_len, SEEK_SET);
    }

    uint8_t payload[256];
    long n = 0;
    wflog_rec_t rec;
    while (fread(&rec, 1, sizeof(rec), f) == sizeof(rec)) {
        if (rec.len > 0 && fread(payload, 1, rec.len, f) != rec.len) {
            break;
        }
        if (rec.type == WFREC_EVENT) {
            payload[rec.len] = '\0';
            printf("WFE t=%" PRIu32 " %s\n", rec.t_ms, (char *)payload);
        } else if (rec.type == WFREC_TELEM && rec.len == sizeof(wflog_telem_t)) {
            wflog_telem_t t;
            memcpy(&t, payload, sizeof(t));
            printf("WFT t=%" PRIu32 " batt_mv=%u rssi_mcu=%d rssi_bms=%d "
                   "mcu=%" PRIu32 " bms=%" PRIu32 " dropped=%" PRIu32 "\n",
                   rec.t_ms, t.batt_mv, t.rssi_mcu, t.rssi_bms, t.frames_mcu,
                   t.frames_bms, t.dropped);
        } else if (rec.type == WFREC_IMU && rec.len == sizeof(wflog_imu_t)) {
            wflog_imu_t m;
            memcpy(&m, payload, sizeof(m));
            printf("WFI t=%" PRIu32 " a=%d,%d,%d g=%d,%d,%d\n", rec.t_ms,
                   m.ax, m.ay, m.az, m.gx, m.gy, m.gz);
        } else {
            /* Anything still raw. The source has to be spelled out rather than
             * assumed to be the BMS whenever it is not the MCU: that guess
             * printed every IMU sample as BMS traffic and made the Daly look
             * like it was streaming unsolicited, which it never does. */
            const char *src = "?";
            if (rec.type == WFREC_MCU) {
                src = "mcu";
            } else if (rec.type == WFREC_BMS) {
                src = "bms";
            }
            printf("WFR t=%" PRIu32 " src=%s type=0x%02x len=%u data=",
                   rec.t_ms, src, rec.type, rec.len);
            for (int i = 0; i < rec.len; i++) {
                printf("%02x", payload[i]);
            }
            printf("\n");
        }
        if (max_recs > 0 && ++n >= max_recs) {
            break;
        }
    }
    fclose(f);
    printf("CAPDUMP_END records=%ld\n", n);
    return 0;
}

/*
 * Two jobs under one name, because to a rider they are one thing: the radio.
 * `wifi on|off` is the Wi-Fi mode - the board joins the strongest network it
 * knows and serves its pages over that, or puts up its own access point and
 * serves them over that instead (#41). `wifi add|list|del` is the list it
 * joins from, up to four networks (ADR-0006), so that a second phone is not a
 * data-model change.
 *
 * `wifi on` stops at the mode: it does not go looking for an update, because
 * `ota check` is the command that has always meant that and still does. On
 * the console the two are worth keeping apart; on the panel there is one menu
 * entry and it does both, because a rider has nothing to type.
 *
 * The credentials only ever arrive here, at runtime, over the cable: the
 * repository is public, so nothing in it may carry an SSID or a passphrase.
 * They are stored unencrypted, deliberately - a hotspot key is the right kind
 * of secret to keep on a bike, and it can be rotated on the phone.
 *
 * An SSID or a passphrase with a space in it needs quoting, which the console
 * splitter understands: wifi add "the cafe" "long pass phrase".
 */
static void wifi_usage(void)
{
    printf("usage: wifi on|off|list|add <ssid> [passphrase]|del <ssid>\n");
}

/* Which link the mode landed on, and what is on the other end of it. `ap=1`
 * is the fallback: nothing was stored, or nothing known was in range, or the
 * join was refused - in which case `clients` is the phones that have joined
 * us, and on a network we joined there are none to count. */
static void wifi_print_state(void)
{
    printf("WIFI running=%d ap=%d clients=%d\n", web_running(),
           web_ap_running(), web_clients());
}

static void wifi_print_list(void)
{
    wifi_net_t nets[WIFI_STORE_MAX];
    int        n = wifi_store_load(nets, WIFI_STORE_MAX);

    for (int i = 0; i < n; i++) {
        /* The passphrase is not printed back. It is in NVS in the clear by
         * design, but a console session is copied into log files and pasted
         * into tickets, and its length is enough to spot a typo by. */
        printf("WIFINET %d ssid=%s passlen=%u\n", i, nets[i].ssid,
               (unsigned)strlen(nets[i].pass));
    }
    printf("WIFINETS count=%d max=%d\n", n, WIFI_STORE_MAX);
}

static int cmd_wifi(int argc, char **argv)
{
    if (argc < 2) {
        wifi_usage();
        return 1;
    }

    if (strcmp(argv[1], "list") == 0) {
        wifi_print_list();
        return 0;
    }
    if (strcmp(argv[1], "add") == 0) {
        if (argc < 3 || argc > 4) {
            wifi_usage();
            return 1;
        }
        esp_err_t err = wifi_store_add(argv[2], (argc == 4) ? argv[3] : "");
        if (err != ESP_OK) {
            /* The two that are worth naming: a passphrase WPA2 would refuse
             * anyway, and a list with no room left. */
            printf("WIFI error=%s\n",
                   err == ESP_ERR_INVALID_SIZE ? "ssid or passphrase length"
                   : err == ESP_ERR_NO_MEM     ? "the list is full"
                                               : esp_err_to_name(err));
            return 1;
        }
        wifi_print_list();
        return 0;
    }
    if (strcmp(argv[1], "del") == 0) {
        if (argc != 3) {
            wifi_usage();
            return 1;
        }
        esp_err_t err = wifi_store_del(argv[2]);
        if (err != ESP_OK) {
            printf("WIFI error=%s\n",
                   err == ESP_ERR_NOT_FOUND ? "no such network"
                                            : esp_err_to_name(err));
            return 1;
        }
        wifi_print_list();
        return 0;
    }

    if (argc != 2) {
        wifi_usage();
        return 1;
    }
    if (strcmp(argv[1], "on") == 0) {
        if (!app_service_up()) {
            printf("WIFI error=cannot start the wifi mode\n");
            return 1;
        }
    } else if (strcmp(argv[1], "off") == 0) {
        app_service_stop();
        /* BLE is gone for good once the mode has run; say so rather than
         * letting the user believe a capture would still work. */
        printf("WIFI off - reboot to capture again\n");
    } else {
        wifi_usage();
        return 1;
    }
    wifi_print_state();
    return 0;
}

static int cmd_time(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "set") == 0) {
        struct tm tm_in = {0};
        int y, mo, d, h, mi, s = 0;
        /* Two forms, both UTC: "time set 2026-08-29 14:03:00" and the same
         * with a T separator, because that is what date -u -Is prints. */
        if (argc >= 4 &&
            sscanf(argv[2], "%d-%d-%d", &y, &mo, &d) == 3 &&
            sscanf(argv[3], "%d:%d:%d", &h, &mi, &s) >= 2) {
            /* split form */
        } else if (argc >= 3 &&
                   sscanf(argv[2], "%d-%d-%dT%d:%d:%d", &y, &mo, &d, &h, &mi, &s) >= 5) {
            /* ISO form */
        } else {
            printf("usage: time set <YYYY-MM-DD> <HH:MM[:SS]>   (UTC)\n");
            return 1;
        }
        tm_in.tm_year = y - 1900;
        tm_in.tm_mon = mo - 1;
        tm_in.tm_mday = d;
        tm_in.tm_hour = h;
        tm_in.tm_min = mi;
        tm_in.tm_sec = s;
        esp_err_t err = bm8563_set(&tm_in);
        if (err != ESP_OK) {
            printf("TIME error=%s\n", esp_err_to_name(err));
            return 1;
        }
        bm8563_sync_system_time();
    }

    struct tm now;
    if (bm8563_get(&now) == ESP_OK) {
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &now);
        printf("TIME rtc=%s valid=%d present=%d unix=%lld\n", buf, bm8563_valid(),
               bm8563_present(), (long long)bm8563_unix());
    } else {
        printf("TIME present=%d - no reading\n", bm8563_present());
    }
    return 0;
}

static int cmd_batt(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("BATT mv=%d pct=%d\n", board_battery_mv(), board_battery_pct());
    return 0;
}

static int cmd_disp(int argc, char **argv)
{
    if (argc == 2) {
        disp_backlight(strcmp(argv[1], "on") == 0);
        ui_redraw();
    }
    printf("DISP backlight=%d ready=%d\n", disp_backlight_get(), disp_ready());
    return 0;
}

void cmd_cap_register(void)
{
    const esp_console_cmd_t cmds[] = {
        {.command = "cap",
         .help = "Standalone capture: cap [status|scan|idle|rec|stop|mark [text]]",
         .func = cmd_cap},
        {.command = "caps", .help = "List the captures on flash", .func = cmd_caps},
        {.command = "caprm", .help = "Delete a capture: caprm <seq>|all", .func = cmd_caprm},
        {.command = "capdump",
         .help = "Print a capture over the console: capdump <seq> [max_records]",
         .func = cmd_capdump},
        {.command = "wifi",
         .help = "The wifi mode and the networks it may join: "
                 "wifi on|off|list|add <ssid> [passphrase]|del <ssid>",
         .func = cmd_wifi},
        {.command = "time",
         .help = "Read or set the RTC: time [set <YYYY-MM-DD> <HH:MM[:SS]>] (UTC)",
         .func = cmd_time},
        {.command = "batt", .help = "Battery voltage and rough state of charge", .func = cmd_batt},
        {.command = "disp", .help = "LCD backlight: disp on|off", .func = cmd_disp},
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}
