/*
 * ST7789v2 LCD on the M5StickC PLUS2, 135x240 portrait.
 *
 * Deliberately framebuffer-free: NimBLE and Wi-Fi need the RAM more than the
 * UI does, so every primitive streams straight into the panel through a small
 * scratch buffer. Text is a 5x7 font drawn at an integer scale, which makes
 * the character cell 6*scale by 8*scale pixels.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define DISP_W 135
#define DISP_H 240

/* Character cell including the one pixel of spacing right of / below a glyph. */
#define DISP_CHAR_W(scale) (6 * (scale))
#define DISP_CHAR_H(scale) (8 * (scale))

/* RGB565. */
#define DISP_BLACK   0x0000
#define DISP_WHITE   0xFFFF
#define DISP_RED     0xF800
#define DISP_GREEN   0x07E0
#define DISP_BLUE    0x001F
#define DISP_YELLOW  0xFFE0
#define DISP_CYAN    0x07FF
#define DISP_MAGENTA 0xF81F
#define DISP_GREY    0x8410
#define DISP_DARKGREY 0x4208
#define DISP_ORANGE  0xFD20

esp_err_t disp_init(void);
bool      disp_ready(void);
void      disp_backlight(bool on);
bool      disp_backlight_get(void);

void disp_clear(uint16_t color);
void disp_fill_rect(int x, int y, int w, int h, uint16_t color);
void disp_hline(int x, int y, int w, uint16_t color);

/* Draws text with an opaque background, so a redraw overwrites the previous
 * value without having to clear the screen first. Unknown characters render
 * as a filled box. Text is clipped at the panel edge, never wrapped. */
void disp_text(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *s);
void disp_textf(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *fmt, ...)
    __attribute__((format(printf, 6, 7)));
/* Same, but pads the line with bg out to the right edge of the panel: that is
 * what makes a shrinking value ("100%" -> "99%") leave no debris behind. */
void disp_text_line(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *s);
void disp_textf_line(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *fmt, ...)
    __attribute__((format(printf, 6, 7)));

int disp_text_width(int scale, const char *s);
