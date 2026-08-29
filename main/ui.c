/*
 * ui - what the LCD shows while the capture runs.
 *
 * Two things shape every decision in here.
 *
 * There is no framebuffer (see display.h): each primitive streams straight
 * into the panel, so an unconditional repaint of the whole screen at 5 Hz is
 * visible as flicker and eats SPI bandwidth for nothing. Every value is drawn
 * through field_at(), which keeps the last rendered string, colour, scale and
 * left edge in a static and returns without touching the panel when nothing
 * changed. Only a state change (or ui_redraw()) wipes a region.
 *
 * The reader is a rider with the helmet on, looking at a 135x240 panel
 * strapped to the handlebar at arm's length. So the one fact that matters in
 * a given state gets the largest scale that fits the width, and the rest is
 * demoted: in CAP_SCANNING that is the two link lines, in CAP_RECORDING the
 * elapsed time. Everything else is reference material and lives at scale 2.
 */
#include "ui.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "board.h"
#include "capture.h"
#include "capture_store.h"
#include "display.h"
#include "rtc_bm8563.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "ui";

/* ---- layout -------------------------------------------------------------
 *
 * All of it in one place so a row can be nudged without hunting through the
 * drawing code. The character cell is 6*scale by 8*scale, so the panel holds
 * 22 columns at scale 1, 11 at scale 2, 7 at scale 3 and 5 at scale 4. Every
 * format string below is checked against that budget.
 *
 *    0..29    status bar: wall clock, battery percent, battery mV
 *   30        separator
 *   31..215   content, per state
 *  216        separator
 *  218..239   hint line: what the two buttons do right now
 */
#define TICK_MS        200          /* 5 Hz */
#define BAR_H          30
#define BAR_SEP_Y      30
#define CONTENT_Y      36
#define HINT_SEP_Y     216
#define HINT_BAND_Y    218
#define HINT_BAND_H    (DISP_H - HINT_BAND_Y)

#define ROW_CLOCK      3            /* scale 2, x = 2 */
#define ROW_BATT_MV    20           /* scale 1, right aligned */
#define COL_BATT_X     85           /* 4 chars at scale 2 ending 2 px short */
#define COL_BATT_MV_X  97           /* 6 chars at scale 1 */

#define ROW_TITLE      44           /* up to scale 4, centred */
#define ROW_LINK1      92           /* scale 3 in SCAN/ARMED, scale 2 linking */
#define ROW_SUB1       118          /* scale 1: peer name or address */
#define ROW_LINK2      136
#define ROW_SUB2       162
#define COL_LINK_X     4            /* centres "MCU -63" at scale 3 */

#define ROW_IDLE_FREE  112
#define ROW_IDLE_CAPS  138

#define ROW_REC_BANNER CONTENT_Y
#define REC_BANNER_H   26
#define ROW_REC_TIME   68           /* scale 4 "MM:SS", 120 px of 135 */
#define ROW_REC_FRAMES 106
#define ROW_REC_MCU    124
#define ROW_REC_BMS    142
#define ROW_REC_DROP   160
#define ROW_REC_FILE   180
#define ROW_REC_FREE   198

#define ROW_STOP_NOTE  100

#define ROW_DONE_FILE  96
#define ROW_DONE_TIME  122
#define ROW_DONE_FRM   148
#define ROW_DONE_SIZE  174

#define ROW_ERR_TEXT   96
#define ERR_PITCH      14
#define ERR_COLS       22           /* scale 1 fills the width exactly */
#define ERR_LINES      4

/* Live screen: gear borrows the title row, the rest reuses F_V0..F_V5, same
 * as every other per-state screen - see the F_V slot comment above. */
#define ROW_LIVE_SPEED  70
#define ROW_LIVE_RPM    100
#define ROW_LIVE_BRAKE  120
#define ROW_LIVE_MOTION 138
#define ROW_LIVE_TEMP   158
#define ROW_LIVE_ODO    178

#define MSG_TITLE_Y    16
#define MSG_SEP_Y      44
#define MSG_LINE_Y     58
#define MSG_PITCH      40           /* room for a scale 3 line */

#define COL_BG         DISP_BLACK
#define COL_BAR        DISP_DARKGREY
#define COL_DIM        DISP_GREY
#define COL_VALUE      DISP_WHITE
#define COL_HINT       DISP_YELLOW

/* Above this the divider on GPIO38 is reading the charger rail rather than
 * the cell, so the percentage is meaningless and the raw mV goes up too. */
#define BATT_AMBIG_MV  4150
#define BATT_LOW_PCT   15
#define BATT_WARN_PCT  30

/* Free space and the capture count come from the filesystem, which the writer
 * task also holds a lock on, so they are sampled rarely rather than per tick. */
#define SPACE_PERIOD_TICKS 25       /* 5 s */

#define MSG_TEXT_MAX   32
#define FIELD_TEXT_MAX 24

/* ---- cached fields ------------------------------------------------------ */

typedef struct {
    char     text[FIELD_TEXT_MAX];
    uint16_t fg;
    uint16_t bg;
    int16_t  x;
    int8_t   scale;
    bool     valid;
} field_t;

/* Slots are reused across states: a state change clears the content area and
 * invalidates everything, so no stale text can survive the switch. */
enum {
    F_CLOCK = 0, F_BATT, F_BATT_MV,
    F_TITLE, F_LINK1, F_LINK2, F_SUB1, F_SUB2,
    F_V0, F_V1, F_V2, F_V3, F_V4, F_V5, F_V6,
    F_COUNT
};

static field_t s_field[F_COUNT];
static char    s_hint[32];
static bool    s_hint_valid;

/* ---- shared state ------------------------------------------------------- */

typedef struct {
    bool active;
    char title[MSG_TEXT_MAX];
    char line[4][MSG_TEXT_MAX];
} ui_msg_t;

/* s_msg and s_invalidate are written by the button task and read by the
 * redraw task, so they are the only things behind the mutex. Everything else
 * in this file belongs to the redraw task alone. */
static SemaphoreHandle_t s_mtx;
static ui_msg_t          s_msg;
static uint32_t          s_msg_gen;      /* bumped on every message change */
/* True at boot so the first tick paints the chrome over the splash. */
static bool              s_invalidate = true;
/* Which state-screen slot to draw: the status screen for cap_state_t, or the
 * live Fardriver screen. Toggled by PWR-short; see ui_live_toggle(). */
static bool              s_live_screen;

static TaskHandle_t s_task;
static bool         s_started;

/* redraw task only */
static int      s_last_state = -1;
static uint32_t s_drawn_gen;
static bool     s_msg_drawn;
static bool     s_led_owned;             /* we are blinking it, we clear it */
static unsigned s_free_mb;
static int      s_cap_count;
static bool     s_space_ok;
static uint32_t s_tick;

static void ui_lock(void)
{
    if (s_mtx) {
        xSemaphoreTake(s_mtx, portMAX_DELAY);
    }
}

static void ui_unlock(void)
{
    if (s_mtx) {
        xSemaphoreGive(s_mtx);
    }
}

/* ---- drawing helpers ---------------------------------------------------- */

/* Largest integer scale at which s still fits the panel width. */
static int fit_scale(const char *s, int max_scale)
{
    for (int sc = max_scale; sc > 1; sc--) {
        if (disp_text_width(sc, s) <= DISP_W) {
            return sc;
        }
    }
    return 1;
}

static int centre_x(int scale, const char *s)
{
    int x = (DISP_W - disp_text_width(scale, s)) / 2;
    return x < 0 ? 0 : x;
}

/* The one place that touches the panel for a value. Redraws only when
 * something about the rendered result actually changed - that is what keeps
 * the screen from flickering at 5 Hz. pad uses disp_text_line(), which fills
 * out to the right edge so a shrinking value ("100%" -> "99%") leaves no
 * debris; it must be false for a field that shares its row with another. */
static void field_at(field_t *f, int x, int y, int scale, uint16_t fg, uint16_t bg,
                     bool pad, const char *fmt, ...)
    __attribute__((format(printf, 8, 9)));

static void field_at(field_t *f, int x, int y, int scale, uint16_t fg, uint16_t bg,
                     bool pad, const char *fmt, ...)
{
    char buf[FIELD_TEXT_MAX];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (f->valid && f->scale == scale && f->x == x && f->fg == fg && f->bg == bg &&
        strcmp(f->text, buf) == 0) {
        return;
    }
    /* A new scale or a new left edge leaves pixels the new text will not
     * cover, so wipe the taller of the two bands before drawing. */
    if (f->valid && (f->scale != scale || f->x != x)) {
        int h = DISP_CHAR_H(f->scale > scale ? f->scale : scale);
        disp_fill_rect(0, y, DISP_W, h, bg);
    }
    if (pad) {
        disp_text_line(x, y, scale, fg, bg, buf);
    } else {
        disp_text(x, y, scale, fg, bg, buf);
    }
    snprintf(f->text, sizeof(f->text), "%s", buf);
    f->fg = fg;
    f->bg = bg;
    f->x = (int16_t)x;
    f->scale = (int8_t)scale;
    f->valid = true;
}

/* The common case: full-width value on the content background. */
#define FIELD(slot, x, y, sc, fg, ...) \
    field_at(&s_field[slot], (x), (y), (sc), (fg), COL_BG, true, __VA_ARGS__)

static void fields_invalidate(void)
{
    for (int i = 0; i < F_COUNT; i++) {
        s_field[i].valid = false;
    }
    s_hint_valid = false;
}

static void paint_chrome(void)
{
    disp_fill_rect(0, 0, DISP_W, BAR_H, COL_BAR);
    disp_hline(0, BAR_SEP_Y, DISP_W, COL_DIM);
    disp_fill_rect(0, BAR_SEP_Y + 1, DISP_W, HINT_SEP_Y - BAR_SEP_Y - 1, COL_BG);
    disp_hline(0, HINT_SEP_Y, DISP_W, COL_DIM);
    disp_fill_rect(0, HINT_BAND_Y, DISP_W, HINT_BAND_H, COL_BG);
}

static void clear_content(void)
{
    disp_fill_rect(0, BAR_SEP_Y + 1, DISP_W, HINT_SEP_Y - BAR_SEP_Y - 1, COL_BG);
    disp_fill_rect(0, HINT_BAND_Y, DISP_W, HINT_BAND_H, COL_BG);
}

/* Centred, and at whatever scale the word fits: "READY" and "ARMED" land at
 * scale 4, "LINKING" and "SAVING" at scale 3. */
static void draw_title(const char *s, uint16_t fg)
{
    int sc = fit_scale(s, 4);
    field_at(&s_field[F_TITLE], centre_x(sc, s), ROW_TITLE, sc, fg, COL_BG,
             false, "%s", s);
}

/* The hint keeps its own cache because it re-centres and changes scale, which
 * means the whole band has to go before the new text lands. */
static void draw_hint(const char *s)
{
    if (s_hint_valid && strcmp(s_hint, s) == 0) {
        return;
    }
    int sc = fit_scale(s, 2);
    int y = HINT_BAND_Y + (HINT_BAND_H - DISP_CHAR_H(sc)) / 2;

    disp_fill_rect(0, HINT_BAND_Y, DISP_W, HINT_BAND_H, COL_BG);
    disp_text(centre_x(sc, s), y, sc, COL_HINT, COL_BG, s);
    snprintf(s_hint, sizeof(s_hint), "%s", s);
    s_hint_valid = true;
}

/* ---- formatting --------------------------------------------------------- */

/* Counters are printed into an 11 column row, so they are clamped rather
 * than allowed to run off the right edge of the panel: a uint32 at full
 * width is 10 digits, which no label can share a row with. */
static uint32_t clamp_u32(uint32_t v, uint32_t max)
{
    return v > max ? max : v;
}

/* MM:SS, and MMM:SS past the hour and a half - the recording screen picks its
 * scale from the result, so it stays honest on a long ride. */
static void fmt_dur(char *out, size_t cap, uint32_t ms)
{
    uint32_t sec = ms / 1000;
    uint32_t min = sec / 60;

    sec %= 60;
    if (min > 999) {
        min = 999;
        sec = 59;
    }
    snprintf(out, cap, "%02" PRIu32 ":%02" PRIu32, min, sec);
}

/* Greedy word wrap for cap_status_t.err (47 characters at most, so three rows
 * of 22 always hold it; the fourth is slack for an unlucky break). */
static void wrap_err(const char *src, char out[ERR_LINES][ERR_COLS + 1])
{
    const char *p = src;
    int line = 0;

    memset(out, 0, ERR_LINES * (ERR_COLS + 1));
    while (*p && line < ERR_LINES) {
        while (*p == ' ') {
            p++;
        }
        if (!*p) {
            break;
        }
        int n = 0;
        while (p[n] && n < ERR_COLS) {
            n++;
        }
        if (p[n]) {                     /* more text follows: break on a space */
            for (int i = n; i > 0; i--) {
                if (p[i] == ' ') {
                    n = i;
                    break;
                }
            }
        }
        memcpy(out[line], p, (size_t)n);
        out[line][n] = '\0';
        p += n;
        line++;
    }
}

/* ---- status bar --------------------------------------------------------- */

static void draw_bar(void)
{
    struct tm tm_now;
    char clock[8];

    if (bm8563_valid() && bm8563_get(&tm_now) == ESP_OK) {
        snprintf(clock, sizeof(clock), "%02d:%02d", tm_now.tm_hour, tm_now.tm_min);
    } else {
        snprintf(clock, sizeof(clock), "--:--");
    }
    /* Fixed five characters at x = 2, so it never collides with the battery
     * field at x = 85 and never needs padding. */
    field_at(&s_field[F_CLOCK], 2, ROW_CLOCK, 2, COL_VALUE, COL_BAR, false,
             "%s", clock);

    int pct = board_battery_pct();
    int mv = board_battery_mv();
    uint16_t col = COL_VALUE;

    if (pct >= 0 && pct <= BATT_LOW_PCT) {
        col = DISP_RED;
    } else if (pct >= 0 && pct <= BATT_WARN_PCT) {
        col = DISP_YELLOW;
    }
    /* Right aligned but drawn at a fixed x with a fixed width, so the leading
     * blanks erase the digit that "100%" -> "99%" leaves behind. */
    if (pct < 0) {
        field_at(&s_field[F_BATT], COL_BATT_X, ROW_CLOCK, 2, COL_DIM, COL_BAR,
                 false, "  --");
    } else {
        field_at(&s_field[F_BATT], COL_BATT_X, ROW_CLOCK, 2, col, COL_BAR,
                 false, "%3d%%", pct > 100 ? 100 : pct);
    }
    /* On USB the divider reads the charger, not the cell: the percentage
     * pins at 100 and means nothing, so show the raw millivolts as well. */
    if (mv >= BATT_AMBIG_MV) {
        field_at(&s_field[F_BATT_MV], COL_BATT_MV_X, ROW_BATT_MV, 1, DISP_CYAN,
                 COL_BAR, true, "%4dmV", mv > 9999 ? 9999 : mv);
    } else {
        field_at(&s_field[F_BATT_MV], COL_BATT_MV_X, ROW_BATT_MV, 1, DISP_CYAN,
                 COL_BAR, true, "%s", "");
    }
}

/* ---- link rows ---------------------------------------------------------- */

/* "MCU -63" at scale 3 is seven characters, exactly the width of the panel:
 * this is the line the rider watches while the ignition comes on, so it gets
 * the biggest type any per-link value can have. */
static void draw_link_scan(int slot, int sub_slot, int y, int sub_y,
                           const char *label, const cap_link_status_t *l)
{
    int rssi = l->rssi;

    if (rssi < -99) {
        rssi = -99;
    }
    if (rssi > 0) {
        rssi = 0;
    }
    if (l->seen) {
        field_at(&s_field[slot], COL_LINK_X, y, 3, DISP_GREEN, COL_BG, true,
                 "%s %3d", label, rssi);
        field_at(&s_field[sub_slot], COL_LINK_X, sub_y, 1, COL_DIM, COL_BG, true,
                 "%s", l->name);
    } else {
        field_at(&s_field[slot], COL_LINK_X, y, 3, COL_DIM, COL_BG, true,
                 "%s ---", label);
        field_at(&s_field[sub_slot], COL_LINK_X, sub_y, 1, COL_DIM, COL_BG, true,
                 "%s", "not found");
    }
}

/* Same rows, but during CAP_CONNECTING what matters is how far along each
 * link is, so the word replaces the RSSI and the address replaces the name. */
static void draw_link_progress(int slot, int sub_slot, int y, int sub_y,
                               const char *label, const cap_link_status_t *l)
{
    const char *word = "......";
    uint16_t col = COL_DIM;

    if (l->subscribed) {
        word = "SUBSCR";
        col = DISP_GREEN;
    } else if (l->connected) {
        word = "CONN";
        col = DISP_YELLOW;
    }
    field_at(&s_field[slot], COL_LINK_X, y, 2, col, COL_BG, true,
             "%s %s", label, word);
    field_at(&s_field[sub_slot], COL_LINK_X, sub_y, 1, COL_DIM, COL_BG, true,
             "%s", l->addr);
}

/* ---- per state screens -------------------------------------------------- */

static void refresh_space(bool force)
{
    if (!force && (s_tick % SPACE_PERIOD_TICKS) != 0) {
        return;
    }
    if (!store_ready()) {
        s_space_ok = false;
        return;
    }
    uint64_t total = 0, freeb = 0;

    if (store_space(&total, &freeb) == ESP_OK) {
        uint64_t mb = freeb / (1024 * 1024);
        s_free_mb = (unsigned)(mb > 999 ? 999 : mb);
        s_space_ok = true;
    }
    int n = store_count();

    s_cap_count = n < 0 ? 0 : (n > 99 ? 99 : n);
}

static void draw_idle(bool full)
{
    refresh_space(full);
    draw_title("READY", DISP_GREEN);
    if (s_space_ok) {
        FIELD(F_V0, 2, ROW_IDLE_FREE, 2, COL_VALUE, "FREE %u MB", s_free_mb);
        FIELD(F_V1, 2, ROW_IDLE_CAPS, 2, COL_DIM, "%d CAPTURES", s_cap_count);
    } else {
        FIELD(F_V0, 2, ROW_IDLE_FREE, 2, DISP_RED, "NO FLASH");
        FIELD(F_V1, 2, ROW_IDLE_CAPS, 2, COL_DIM, "%s", "");
    }
    draw_hint("A: SCAN  B-hold: WIFI");
}

static void draw_scanning(const cap_status_t *st)
{
    draw_title("SCAN", DISP_CYAN);
    draw_link_scan(F_LINK1, F_SUB1, ROW_LINK1, ROW_SUB1, "MCU",
                   &st->link[CAP_LINK_MCU]);
    draw_link_scan(F_LINK2, F_SUB2, ROW_LINK2, ROW_SUB2, "BMS",
                   &st->link[CAP_LINK_BMS]);
    draw_hint("A: STOP");
}

static void draw_armed(const cap_status_t *st)
{
    /* Deliberately the same rows as CAP_SCANNING: the moment the second
     * device shows up only the title changes, nothing jumps. */
    draw_title("ARMED", DISP_GREEN);
    draw_link_scan(F_LINK1, F_SUB1, ROW_LINK1, ROW_SUB1, "MCU",
                   &st->link[CAP_LINK_MCU]);
    draw_link_scan(F_LINK2, F_SUB2, ROW_LINK2, ROW_SUB2, "BMS",
                   &st->link[CAP_LINK_BMS]);
    draw_hint("A: START");
}

static void draw_connecting(const cap_status_t *st)
{
    draw_title("LINKING", DISP_YELLOW);
    draw_link_progress(F_LINK1, F_SUB1, ROW_LINK1, ROW_SUB1, "MCU",
                       &st->link[CAP_LINK_MCU]);
    draw_link_progress(F_LINK2, F_SUB2, ROW_LINK2, ROW_SUB2, "BMS",
                       &st->link[CAP_LINK_BMS]);
    draw_hint("A: ABORT");
}

static void draw_recording(const cap_status_t *st, bool full)
{
    char dur[12];

    if (full) {
        /* Banner rather than a word: red across the full width is readable
         * from the corner of the eye, which is all the attention a rider has
         * to spare. It is static, so it is painted once per entry. */
        disp_fill_rect(0, ROW_REC_BANNER, DISP_W, REC_BANNER_H, DISP_RED);
        disp_text(centre_x(3, "REC"), ROW_REC_BANNER + 1, 3, DISP_WHITE,
                  DISP_RED, "REC");
    }
    refresh_space(full);
    fmt_dur(dur, sizeof(dur), st->elapsed_ms);

    /* "MM:SS" is five characters, which is scale 4 (120 px of 135); past 100
     * minutes it grows a digit and drops to scale 3. field_at() wipes the
     * band when the scale changes, so the switch leaves nothing behind. */
    int sc = fit_scale(dur, 4);
    field_at(&s_field[F_V0], centre_x(sc, dur), ROW_REC_TIME, sc, COL_VALUE,
             COL_BG, false, "%s", dur);

    FIELD(F_V1, 2, ROW_REC_FRAMES, 2, COL_VALUE, "F %" PRIu32,
          clamp_u32(st->frames, 999999999));
    FIELD(F_V2, 2, ROW_REC_MCU, 2, DISP_CYAN, "MCU %" PRIu32,
          clamp_u32(st->link[CAP_LINK_MCU].frames, 9999999));
    FIELD(F_V3, 2, ROW_REC_BMS, 2, DISP_CYAN, "BMS %" PRIu32,
          clamp_u32(st->link[CAP_LINK_BMS].frames, 9999999));
    /* Dropped frames are the one number that turns a capture into a bad
     * capture, so it goes red the instant it is not zero. */
    FIELD(F_V4, 2, ROW_REC_DROP, 2, st->dropped ? DISP_RED : COL_DIM,
          "DROP %" PRIu32, clamp_u32(st->dropped, 999999));
    FIELD(F_V5, 2, ROW_REC_FILE, 2, COL_DIM, "%s", st->file);
    if (s_space_ok) {
        FIELD(F_V6, 2, ROW_REC_FREE, 2, COL_DIM, "FREE %u MB", s_free_mb);
    } else {
        FIELD(F_V6, 2, ROW_REC_FREE, 2, DISP_RED, "NO FLASH");
    }
    draw_hint("A: STOP");
}

static void draw_stopping(void)
{
    draw_title("SAVING", DISP_YELLOW);
    FIELD(F_V0, centre_x(1, "DO NOT POWER OFF"), ROW_STOP_NOTE, 1, COL_DIM,
          "DO NOT POWER OFF");
    draw_hint("PLEASE WAIT");
}

static void draw_done(const cap_status_t *st)
{
    char dur[12];

    fmt_dur(dur, sizeof(dur), st->elapsed_ms);
    draw_title("SAVED", DISP_GREEN);
    FIELD(F_V0, 2, ROW_DONE_FILE, 2, COL_VALUE, "%s", st->file);
    FIELD(F_V1, 2, ROW_DONE_TIME, 2, COL_VALUE, "TIME %s", dur);
    FIELD(F_V2, 2, ROW_DONE_FRM, 2, COL_VALUE, "FRM %" PRIu32,
          clamp_u32(st->frames, 9999999));

    uint64_t kb = st->bytes / 1024;

    FIELD(F_V3, 2, ROW_DONE_SIZE, 2, COL_DIM, "%" PRIu64 " KB",
          kb > 99999999 ? 99999999 : kb);
    draw_hint("A: SCAN AGAIN");
}

static void draw_error(const cap_status_t *st)
{
    char lines[ERR_LINES][ERR_COLS + 1];

    draw_title("ERROR", DISP_RED);
    wrap_err(st->err[0] ? st->err : "unknown failure", lines);
    for (int i = 0; i < ERR_LINES; i++) {
        FIELD(F_V0 + i, 2, ROW_ERR_TEXT + i * ERR_PITCH, 1, COL_VALUE,
              "%s", lines[i]);
    }
    draw_hint("A: RETRY");
}

/* ---- live screen ---------------------------------------------------------
 *
 * Reads cap_live_get() instead of cap_status(): the Fardriver keeps pushing
 * frames whenever the MCU link is subscribed, capture running or not, so this
 * screen is not one of the cap_state_t cases in draw_state() and can be up at
 * the same time as any of them. It shares F_TITLE/F_V0..F_V5 with those
 * screens rather than getting its own slots, same reuse as every state below. */
static const char *gear_name(uint8_t gear)
{
    switch (gear) {
    case 0:  return "ECO";
    case 1:  return "STANDARD";
    case 2:  return "SPORT";
    default: return "?";
    }
}

static void draw_live(const fardriver_live_t *lv)
{
    if (lv->b0_valid) {
        draw_title(gear_name(lv->gear), DISP_GREEN);
    } else {
        draw_title("--", COL_DIM);
    }

    if (lv->speed_valid) {
        FIELD(F_V0, 2, ROW_LIVE_SPEED, 3, COL_VALUE, "%.0f KM/H", lv->cur_speed_kmh);
    } else {
        FIELD(F_V0, 2, ROW_LIVE_SPEED, 3, COL_DIM, "%s", "-- KM/H");
    }
    if (lv->b0_valid) {
        FIELD(F_V1, 2, ROW_LIVE_RPM, 2, COL_VALUE, "%u RPM", lv->cur_rpm);
        FIELD(F_V2, 2, ROW_LIVE_BRAKE, 2, lv->brake_switch ? DISP_RED : COL_DIM,
              "BRAKE %s", lv->brake_switch ? "ON" : "off");
        FIELD(F_V3, 2, ROW_LIVE_MOTION, 2, lv->motion ? DISP_GREEN : COL_DIM,
              "MOVING %s", lv->motion ? "yes" : "no");
    } else {
        FIELD(F_V1, 2, ROW_LIVE_RPM, 2, COL_DIM, "%s", "-- RPM");
        FIELD(F_V2, 2, ROW_LIVE_BRAKE, 2, COL_DIM, "%s", "BRAKE --");
        FIELD(F_V3, 2, ROW_LIVE_MOTION, 2, COL_DIM, "%s", "MOVING --");
    }
    if (lv->b5_valid) {
        FIELD(F_V4, 2, ROW_LIVE_TEMP, 2, COL_VALUE, "TEMP %d C", lv->engine_temp);
    } else {
        FIELD(F_V4, 2, ROW_LIVE_TEMP, 2, COL_DIM, "%s", "TEMP --");
    }
    if (lv->odo_valid) {
        FIELD(F_V5, 2, ROW_LIVE_ODO, 2, COL_DIM, "ODO %u", lv->odometer_raw);
    } else {
        FIELD(F_V5, 2, ROW_LIVE_ODO, 2, COL_DIM, "%s", "ODO --");
    }
    draw_hint("PWR: BACK");
}

/* ---- message screen ----------------------------------------------------- */

static void draw_message(const ui_msg_t *m)
{
    disp_clear(COL_BG);
    if (m->title[0]) {
        int sc = fit_scale(m->title, 3);
        disp_text(centre_x(sc, m->title), MSG_TITLE_Y, sc, DISP_CYAN, COL_BG,
                  m->title);
    }
    disp_hline(0, MSG_SEP_Y, DISP_W, COL_DIM);
    /* Left aligned and each line at its own scale: the Wi-Fi readout is a
     * list of things to type ("wildfire-a1b2c3" needs scale 1, "192.168.4.1"
     * fits at scale 2), and the reader is copying, not glancing. */
    for (int i = 0; i < 4; i++) {
        if (!m->line[i][0]) {
            continue;
        }
        int sc = fit_scale(m->line[i], 3);
        disp_text_line(2, MSG_LINE_Y + i * MSG_PITCH, sc, DISP_WHITE, COL_BG,
                       m->line[i]);
    }
}

/* ---- redraw task -------------------------------------------------------- */

static void draw_state(const cap_status_t *st, bool full)
{
    switch (st->state) {
    case CAP_IDLE:       draw_idle(full);            break;
    case CAP_SCANNING:   draw_scanning(st);          break;
    case CAP_ARMED:      draw_armed(st);             break;
    case CAP_CONNECTING: draw_connecting(st);        break;
    case CAP_RECORDING:  draw_recording(st, full);   break;
    case CAP_STOPPING:   draw_stopping();            break;
    case CAP_DONE:       draw_done(st);              break;
    case CAP_ERROR:      draw_error(st);             break;
    default:             draw_title("?", DISP_RED);  break;
    }
}

/* The display may be off to save battery, and then the LED is the only sign
 * that the board is still recording. One blip a second: 200 ms on out of the
 * five ticks, which is visible in daylight and cheap on the cell. */
static void drive_led(cap_state_t state)
{
    if (state == CAP_RECORDING) {
        board_led_set((s_tick % 5) == 0);
        s_led_owned = true;
    } else if (s_led_owned) {
        board_led_set(false);
        s_led_owned = false;
    }
}

static void ui_task(void *arg)
{
    (void)arg;

    while (true) {
        ui_msg_t msg;
        uint32_t gen;
        bool invalidate;
        bool live;

        /* The only lock this task takes, and the only holders on the other
         * side are ui_message() and ui_live_toggle() doing a memcpy or a flag
         * flip, so neither can stall a tick. */
        ui_lock();
        msg = s_msg;
        gen = s_msg_gen;
        invalidate = s_invalidate;
        live = s_live_screen;
        s_invalidate = false;
        ui_unlock();

        cap_status_t st;
        cap_status(&st);
        drive_led(st.state);

        if (!disp_ready()) {
            /* Nothing to paint on. Hand the invalidate back, so the first
             * tick after the panel comes up still repaints everything. */
            if (invalidate) {
                ui_lock();
                s_invalidate = true;
                ui_unlock();
            }
            vTaskDelay(pdMS_TO_TICKS(TICK_MS));
            s_tick++;
            continue;
        }
        if (msg.active) {
            /* A message owns the whole panel: the state screens stay off it
             * until ui_message_clear(), and the message itself is static, so
             * it is painted only when it changes. */
            if (!s_msg_drawn || gen != s_drawn_gen || invalidate) {
                draw_message(&msg);
                s_msg_drawn = true;
                s_drawn_gen = gen;
            }
        } else {
            if (s_msg_drawn) {
                s_msg_drawn = false;
                invalidate = true;      /* the message wiped our chrome */
            }
            bool full = invalidate || (int)st.state != s_last_state;

            if (invalidate) {
                paint_chrome();
                fields_invalidate();
            } else if (full) {
                /* State change: wipe the content region rather than track
                 * which of the old rows the new screen fails to overwrite. */
                clear_content();
                fields_invalidate();
            }
            s_last_state = (int)st.state;
            draw_bar();
            if (live) {
                fardriver_live_t lv;
                cap_live_get(&lv);
                draw_live(&lv);
            } else {
                draw_state(&st, full);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        s_tick++;
    }
}

/* ---- api ---------------------------------------------------------------- */

esp_err_t ui_init(void)
{
    if (!s_mtx) {
        s_mtx = xSemaphoreCreateMutex();
        if (!s_mtx) {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t err = disp_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "display init failed: %s", esp_err_to_name(err));
        return err;
    }
    disp_backlight(true);

    disp_clear(COL_BG);
    disp_text(centre_x(2, "WILDFIRE"), 80, 2, DISP_WHITE, COL_BG, "WILDFIRE");
    disp_text(centre_x(2, "MONITOR"), 104, 2, DISP_WHITE, COL_BG, "MONITOR");
    disp_text(centre_x(1, "starting"), 150, 1, COL_DIM, COL_BG, "starting");
    return ESP_OK;
}

esp_err_t ui_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    if (!s_mtx) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Priority 3, same as the heartbeat: the UI must never get in the way of
     * the NimBLE host or the flash writer. 4 KB covers vsnprintf and the
     * cap_status_t snapshot on the stack. */
    if (xTaskCreate(ui_task, "ui", 4096, NULL, 3, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "cannot create the redraw task");
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    return ESP_OK;
}

void ui_message(const char *title, const char *l1, const char *l2,
                const char *l3, const char *l4)
{
    const char *lines[4] = {l1, l2, l3, l4};

    /* Copied, not referenced: the Wi-Fi readout builds its SSID and IP on the
     * button task's stack and returns as soon as this call does. */
    ui_lock();
    snprintf(s_msg.title, sizeof(s_msg.title), "%s", title ? title : "");
    for (int i = 0; i < 4; i++) {
        snprintf(s_msg.line[i], sizeof(s_msg.line[i]), "%s",
                 lines[i] ? lines[i] : "");
    }
    s_msg.active = true;
    s_msg_gen++;
    ui_unlock();
}

void ui_message_clear(void)
{
    ui_lock();
    s_msg.active = false;
    s_msg_gen++;
    s_invalidate = true;
    ui_unlock();
}

void ui_redraw(void)
{
    ui_lock();
    s_invalidate = true;
    ui_unlock();
}

void ui_live_toggle(void)
{
    ui_lock();
    s_live_screen = !s_live_screen;
    s_invalidate = true;      /* the field slots belong to whichever screen
                                * was showing; force the wipe-and-redraw a
                                * state change would otherwise have done. */
    ui_unlock();
}
