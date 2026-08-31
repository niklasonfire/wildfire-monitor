/*
 * ST7789v2 LCD on the M5StickC PLUS2, driven without a framebuffer.
 *
 * A full 135x240 RGB565 framebuffer is 65 KB, which is RAM that NimBLE and the
 * Wi-Fi dump server need far more than the UI does. So every primitive builds
 * its pixels in one small DMA-capable scratch buffer and pushes them out in
 * horizontal bands. The cost is that nothing can be read back from the panel -
 * which is fine, the UI always repaints what it owns.
 */
#include "display.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "disp";

/* LCD pins, see the pin table in README.md. The backlight is a plain GPIO on
 * this board (no PWM channel wired to it), so it is on or off, nothing else. */
#define PIN_LCD_MOSI GPIO_NUM_15
#define PIN_LCD_SCLK GPIO_NUM_13
#define PIN_LCD_DC   GPIO_NUM_14
#define PIN_LCD_RST  GPIO_NUM_12
#define PIN_LCD_CS   GPIO_NUM_5
#define PIN_LCD_BL   GPIO_NUM_27

#define LCD_HOST     SPI2_HOST
#define LCD_PCLK_HZ  (40 * 1000 * 1000)

/* The 135x240 visible area sits at this offset inside the controller's 240x320
 * memory. Wrong values here show up as the picture being shifted by a few
 * pixels with a coloured band at the edge; they are the first thing to try. */
#define LCD_GAP_X 52
#define LCD_GAP_Y 40

/* One scratch line buffer instead of a framebuffer: 2048 pixels is 4 KB, which
 * is 15 full panel rows per SPI transaction. Larger buys nothing at 40 MHz. */
#define DISP_SCRATCH_PX 2048

/* The ST7789 clocks RGB565 in most significant byte first while the ESP32 is
 * little endian, so pixels are byte swapped on their way into the scratch
 * buffer. This is the ONE place to change if the panel comes up wrong:
 *   - picture recognisable but red and blue swapped -> not this, flip
 *     .rgb_ele_order to LCD_RGB_ELEMENT_ORDER_BGR in disp_init();
 *   - colours nonsensical (greys tinted, DISP_RED coming out dark blue-green)
 *     -> set DISP_SWAP_BYTES to 0;
 *   - everything inverted (black background glowing white) -> drop the
 *     esp_lcd_panel_invert_color(true) call in disp_init().
 * None of this could be verified on hardware when the driver was written. */
#define DISP_SWAP_BYTES 1
#if DISP_SWAP_BYTES
#define DISP_PIXEL(c) ((uint16_t)((((uint16_t)(c) & 0x00FFu) << 8) | ((uint16_t)(c) >> 8)))
#else
#define DISP_PIXEL(c) ((uint16_t)(c))
#endif

/* ------------------------------------------------------------------- font */

/* Classic public domain 5x7 LCD font, ASCII 32..126. Column major: each byte
 * is one pixel column, bit 0 is the top row, bits 0..6 are used. The sixth
 * column and eighth row of the character cell are blank spacing, added by the
 * renderer rather than stored. */
static const uint8_t font5x7[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, /* 32 space */
    {0x00, 0x00, 0x5F, 0x00, 0x00}, /* 33 ! */
    {0x00, 0x07, 0x00, 0x07, 0x00}, /* 34 " */
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, /* 35 # */
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, /* 36 $ */
    {0x23, 0x13, 0x08, 0x64, 0x62}, /* 37 % */
    {0x36, 0x49, 0x55, 0x22, 0x50}, /* 38 & */
    {0x00, 0x05, 0x03, 0x00, 0x00}, /* 39 ' */
    {0x00, 0x1C, 0x22, 0x41, 0x00}, /* 40 ( */
    {0x00, 0x41, 0x22, 0x1C, 0x00}, /* 41 ) */
    {0x14, 0x08, 0x3E, 0x08, 0x14}, /* 42 * */
    {0x08, 0x08, 0x3E, 0x08, 0x08}, /* 43 + */
    {0x00, 0x50, 0x30, 0x00, 0x00}, /* 44 , */
    {0x08, 0x08, 0x08, 0x08, 0x08}, /* 45 - */
    {0x00, 0x60, 0x60, 0x00, 0x00}, /* 46 . */
    {0x20, 0x10, 0x08, 0x04, 0x02}, /* 47 / */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 48 0 */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 49 1 */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* 50 2 */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 51 3 */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 52 4 */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* 53 5 */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 54 6 */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 55 7 */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 56 8 */
    {0x06, 0x49, 0x49, 0x29, 0x1E}, /* 57 9 */
    {0x00, 0x36, 0x36, 0x00, 0x00}, /* 58 : */
    {0x00, 0x56, 0x36, 0x00, 0x00}, /* 59 ; */
    {0x00, 0x08, 0x14, 0x22, 0x41}, /* 60 < */
    {0x14, 0x14, 0x14, 0x14, 0x14}, /* 61 = */
    {0x41, 0x22, 0x14, 0x08, 0x00}, /* 62 > */
    {0x02, 0x01, 0x51, 0x09, 0x06}, /* 63 ? */
    {0x32, 0x49, 0x79, 0x41, 0x3E}, /* 64 @ */
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* 65 A */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* 66 B */
    {0x3E, 0x41, 0x41, 0x41, 0x22}, /* 67 C */
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* 68 D */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* 69 E */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* 70 F */
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* 71 G */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* 72 H */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* 73 I */
    {0x20, 0x40, 0x41, 0x3F, 0x01}, /* 74 J */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* 75 K */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* 76 L */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* 77 M */
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* 78 N */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* 79 O */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* 80 P */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* 81 Q */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* 82 R */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* 83 S */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* 84 T */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* 85 U */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* 86 V */
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* 87 W */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* 88 X */
    {0x03, 0x04, 0x78, 0x04, 0x03}, /* 89 Y */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* 90 Z */
    {0x00, 0x7F, 0x41, 0x41, 0x00}, /* 91 [ */
    {0x02, 0x04, 0x08, 0x10, 0x20}, /* 92 backslash */
    {0x00, 0x41, 0x41, 0x7F, 0x00}, /* 93 ] */
    {0x04, 0x02, 0x01, 0x02, 0x04}, /* 94 ^ */
    {0x40, 0x40, 0x40, 0x40, 0x40}, /* 95 _ */
    {0x00, 0x01, 0x02, 0x04, 0x00}, /* 96 ` */
    {0x20, 0x54, 0x54, 0x54, 0x78}, /* 97 a */
    {0x7F, 0x48, 0x44, 0x44, 0x38}, /* 98 b */
    {0x38, 0x44, 0x44, 0x44, 0x20}, /* 99 c */
    {0x38, 0x44, 0x44, 0x48, 0x7F}, /* 100 d */
    {0x38, 0x54, 0x54, 0x54, 0x18}, /* 101 e */
    {0x08, 0x7E, 0x09, 0x01, 0x02}, /* 102 f */
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, /* 103 g */
    {0x7F, 0x08, 0x04, 0x04, 0x78}, /* 104 h */
    {0x00, 0x44, 0x7D, 0x40, 0x00}, /* 105 i */
    {0x20, 0x40, 0x44, 0x3D, 0x00}, /* 106 j */
    {0x7F, 0x10, 0x28, 0x44, 0x00}, /* 107 k */
    {0x00, 0x41, 0x7F, 0x40, 0x00}, /* 108 l */
    {0x7C, 0x04, 0x78, 0x04, 0x7C}, /* 109 m */
    {0x7C, 0x08, 0x04, 0x04, 0x78}, /* 110 n */
    {0x38, 0x44, 0x44, 0x44, 0x38}, /* 111 o */
    {0x7C, 0x14, 0x14, 0x14, 0x08}, /* 112 p */
    {0x08, 0x14, 0x14, 0x18, 0x7C}, /* 113 q */
    {0x7C, 0x08, 0x04, 0x04, 0x08}, /* 114 r */
    {0x48, 0x54, 0x54, 0x54, 0x20}, /* 115 s */
    {0x04, 0x3F, 0x44, 0x40, 0x20}, /* 116 t */
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, /* 117 u */
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, /* 118 v */
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, /* 119 w */
    {0x44, 0x28, 0x10, 0x28, 0x44}, /* 120 x */
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, /* 121 y */
    {0x44, 0x64, 0x54, 0x4C, 0x44}, /* 122 z */
    {0x00, 0x08, 0x36, 0x41, 0x00}, /* 123 { */
    {0x00, 0x00, 0x7F, 0x00, 0x00}, /* 124 | */
    {0x00, 0x41, 0x36, 0x08, 0x00}, /* 125 } */
    {0x08, 0x04, 0x08, 0x10, 0x08}, /* 126 ~ */
};

/* ---------------------------------------------------------------- statics */

static esp_lcd_panel_io_handle_t s_io;
static esp_lcd_panel_handle_t s_panel;
static bool s_ready;
static bool s_bl_on;

/* The UI task and the console task both draw, so the panel, the scratch buffer
 * and the column cache below are all owned by whoever holds s_mtx. */
static SemaphoreHandle_t s_mtx;
static SemaphoreHandle_t s_blit_done;
static uint16_t *s_scratch;

/* One byte per panel column of the text run being drawn: bit 0 is the top row
 * of the character cell. Built once per run so the per-row inner loop is a
 * bit test instead of a divide. */
static uint8_t s_colbits[DISP_W];

/* --------------------------------------------------------------- plumbing */

/* Runs in the SPI ISR when a colour transaction has been clocked out. Without
 * this the next band would overwrite the scratch buffer while DMA is still
 * reading it, which shows up as torn or repeated bands. */
static bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                          esp_lcd_panel_io_event_data_t *edata,
                                          void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_blit_done, &woken);
    return woken == pdTRUE;
}

/* Pushes the first w*h pixels of the scratch buffer. draw_bitmap takes an
 * exclusive end coordinate in IDF v6. */
static void blit_locked(int x, int y, int w, int h)
{
    if (esp_lcd_panel_draw_bitmap(s_panel, x, y, x + w, y + h, s_scratch) != ESP_OK) {
        return;
    }
    /* The timeout only guards against a wedged bus: a 4 KB band at 40 MHz is
     * under a millisecond. */
    xSemaphoreTake(s_blit_done, pdMS_TO_TICKS(100));
}

/* Clips a rectangle to the panel. Returns false if nothing is left of it. */
static bool clip_rect(int *x, int *y, int *w, int *h)
{
    if (*w <= 0 || *h <= 0) {
        return false;
    }
    if (*x < 0) { *w += *x; *x = 0; }
    if (*y < 0) { *h += *y; *y = 0; }
    if (*x + *w > DISP_W) { *w = DISP_W - *x; }
    if (*y + *h > DISP_H) { *h = DISP_H - *y; }
    return *w > 0 && *h > 0 && *x < DISP_W && *y < DISP_H;
}

static void fill_rect_locked(int x, int y, int w, int h, uint16_t color)
{
    if (!clip_rect(&x, &y, &w, &h)) {
        return;
    }
    int band = DISP_SCRATCH_PX / w;      /* w <= 135, so this is at least 15 */
    if (band < 1) {
        band = 1;
    }
    uint16_t raw = DISP_PIXEL(color);
    for (int i = 0; i < w * band; i++) {
        s_scratch[i] = raw;
    }
    for (int yy = y; yy < y + h; ) {
        int rows = (y + h - yy < band) ? (y + h - yy) : band;
        blit_locked(x, yy, w, rows);
        yy += rows;
    }
}

/* ------------------------------------------------------------------- init */

#define LCD_TRY(call)                                                   \
    do {                                                                \
        err = (call);                                                   \
        if (err != ESP_OK) {                                            \
            ESP_LOGE(TAG, "%s: %s", #call, esp_err_to_name(err));       \
            return err;                                                 \
        }                                                               \
    } while (0)

esp_err_t disp_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    /* Allocated once even if an earlier attempt failed further down. */
    if (!s_mtx) {
        s_mtx = xSemaphoreCreateMutex();
    }
    if (!s_blit_done) {
        s_blit_done = xSemaphoreCreateBinary();
    }
    if (!s_scratch) {
        s_scratch = heap_caps_malloc(DISP_SCRATCH_PX * sizeof(uint16_t), MALLOC_CAP_DMA);
    }
    if (!s_mtx || !s_blit_done || !s_scratch) {
        ESP_LOGE(TAG, "out of memory");
        return ESP_ERR_NO_MEM;
    }

    /* Backlight off until the panel has been cleared, otherwise the uninitialised
     * controller RAM is visible as noise for a few hundred milliseconds. */
    gpio_config_t bl_cfg = {
        .pin_bit_mask = BIT64(PIN_LCD_BL),
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&bl_cfg);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(PIN_LCD_BL, 0);
    s_bl_on = false;

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,               /* the panel is write only on this board */
        .sclk_io_num = PIN_LCD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISP_SCRATCH_PX * (int)sizeof(uint16_t),
    };
    err = spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = LCD_PCLK_HZ,
        .trans_queue_depth = 4,
        .on_color_trans_done = on_color_trans_done,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {
            .dc_low_on_data = 0,         /* DC high for data, as the ST7789 wants */
        },
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &s_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "panel io: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_st7789(s_io, &panel_cfg, &s_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "st7789: %s", esp_err_to_name(err));
        return err;
    }

    /* Deliberately not ESP_ERROR_CHECK: a dead panel must not take the capture
     * down with it, the board is useful with a blank screen. */
    LCD_TRY(esp_lcd_panel_reset(s_panel));
    LCD_TRY(esp_lcd_panel_init(s_panel));
    /* This module is wired as a normally black panel, so the controller has to
     * invert; without it the picture comes up as a photographic negative. */
    LCD_TRY(esp_lcd_panel_invert_color(s_panel, true));
    LCD_TRY(esp_lcd_panel_swap_xy(s_panel, false));
    LCD_TRY(esp_lcd_panel_mirror(s_panel, false, false));
    LCD_TRY(esp_lcd_panel_set_gap(s_panel, LCD_GAP_X, LCD_GAP_Y));
    LCD_TRY(esp_lcd_panel_disp_on_off(s_panel, true));

    s_ready = true;
    disp_clear(DISP_BLACK);
    disp_backlight(true);
    ESP_LOGI(TAG, "ST7789 up, %dx%d", DISP_W, DISP_H);
    return ESP_OK;
}

bool disp_ready(void)
{
    return s_ready;
}

void disp_backlight(bool on)
{
    /* Deliberately not gated on s_ready: the pin is configured before the panel
     * is, and the UI may want the light off while an init failure is logged. */
    gpio_set_level(PIN_LCD_BL, on ? 1 : 0);
    s_bl_on = on;
}

bool disp_backlight_get(void)
{
    return s_bl_on;
}

/* ------------------------------------------------------------- primitives */

void disp_clear(uint16_t color)
{
    disp_fill_rect(0, 0, DISP_W, DISP_H, color);
}

void disp_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (!s_ready) {
        return;
    }
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    fill_rect_locked(x, y, w, h, color);
    xSemaphoreGive(s_mtx);
}

void disp_hline(int x, int y, int w, uint16_t color)
{
    disp_fill_rect(x, y, w, 1, color);
}

/* ------------------------------------------------------------------- text */

static int clamp_scale(int scale)
{
    if (scale < 1) {
        return 1;
    }
    return (scale > 16) ? 16 : scale;
}

/* Draws a whole string as one run: the glyph pixels and their background go
 * into the scratch buffer together and leave in as few transactions as the
 * buffer allows, rather than one transaction per character. */
static void text_locked(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *s)
{
    int len = (int)strlen(s);
    if (len <= 0) {
        return;
    }
    int cell = DISP_CHAR_W(scale);
    int rw = len * cell;
    int rh = DISP_CHAR_H(scale);

    /* Clip the run against the panel. xl/yl are run local, x/y stay the origin
     * of the run so the character grid does not shift when it is clipped. */
    int xl0 = (x < 0) ? -x : 0;
    int xl1 = (x + rw > DISP_W) ? DISP_W - x : rw;
    int yl0 = (y < 0) ? -y : 0;
    int yl1 = (y + rh > DISP_H) ? DISP_H - y : rh;
    if (xl0 >= xl1 || yl0 >= yl1) {
        return;
    }
    int w = xl1 - xl0;

    for (int xl = xl0; xl < xl1; xl++) {
        int ci = xl / cell;
        int gc = (xl - ci * cell) / scale;    /* 0..5, column 5 is the spacing */
        uint8_t bits = 0;
        if (gc < 5) {
            unsigned char c = (unsigned char)s[ci];
            /* A filled box for anything unprintable: a mangled string is then
             * visible on the panel instead of silently turning into blanks. */
            bits = (c < 32 || c > 126) ? 0x7F : font5x7[c - 32][gc];
        }
        s_colbits[xl - xl0] = bits;
    }

    uint16_t raw_fg = DISP_PIXEL(fg);
    uint16_t raw_bg = DISP_PIXEL(bg);
    int band = DISP_SCRATCH_PX / w;
    if (band < 1) {
        band = 1;
    }
    for (int yl = yl0; yl < yl1; ) {
        int rows = (yl1 - yl < band) ? (yl1 - yl) : band;
        uint16_t *p = s_scratch;
        for (int r = 0; r < rows; r++) {
            int gr = (yl + r) / scale;        /* 0..7, row 7 is the spacing */
            if (gr >= 7) {
                for (int i = 0; i < w; i++) {
                    *p++ = raw_bg;
                }
                continue;
            }
            for (int i = 0; i < w; i++) {
                *p++ = ((s_colbits[i] >> gr) & 1) ? raw_fg : raw_bg;
            }
        }
        blit_locked(x + xl0, y + yl, w, rows);
        yl += rows;
    }
}

void disp_text(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *s)
{
    if (!s_ready || !s) {
        return;
    }
    scale = clamp_scale(scale);
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    text_locked(x, y, scale, fg, bg, s);
    xSemaphoreGive(s_mtx);
}

void disp_text_line(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *s)
{
    if (!s_ready || !s) {
        return;
    }
    scale = clamp_scale(scale);
    int end = x + (int)strlen(s) * DISP_CHAR_W(scale);
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    text_locked(x, y, scale, fg, bg, s);
    /* Padding to the right edge is what lets "100%" shrink to "99%" without
     * leaving the old last character on the panel. */
    fill_rect_locked(end, y, DISP_W - end, DISP_CHAR_H(scale), bg);
    xSemaphoreGive(s_mtx);
}

void disp_textf(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *fmt, ...)
{
    char buf[128];
    va_list ap;

    if (!s_ready || !fmt) {
        return;
    }
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    disp_text(x, y, scale, fg, bg, buf);
}

void disp_textf_line(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *fmt, ...)
{
    char buf[128];
    va_list ap;

    if (!s_ready || !fmt) {
        return;
    }
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    disp_text_line(x, y, scale, fg, bg, buf);
}

int disp_text_width(int scale, const char *s)
{
    if (!s) {
        return 0;
    }
    return (int)strlen(s) * DISP_CHAR_W(clamp_scale(scale));
}
