/*
 * Kickstarter - UF2 Firmware Launcher for RP2350
 *
 * Scans /kickstarter on SD card for .uf2 files.
 * Displays cover art (BMP) and metadata (XML) on 800x600 HDMI.
 * Left/Right to browse, launch functionality TBD.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/structs/bus_ctrl.h"
#if !PICO_RP2040
#include "hardware/structs/qmi.h"
#endif

#include "board_config.h"
#include "dvi.h"
#include "dvi_timing.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "tmds_encode.h"
#include "ff.h"

// ============================================================================
// Configuration
// ============================================================================

#define SCREEN_W 400
#define SCREEN_H 300
#define DVI_TIMING dvi_timing_800x600p_60hz
#define BASE_DIR "/kickstarter"
#define MAX_ENTRIES 64
#define MAX_PATH 128
#define MAX_TEXT 256

// Layout constants
#define MARGIN       16
#define IMG_X        MARGIN
#define IMG_Y        MARGIN
#define IMG_MAX_W    100
#define IMG_MAX_H    150
#define TEXT_X       (IMG_X + IMG_MAX_W + MARGIN)
#define TEXT_Y       MARGIN
#define TEXT_W       (SCREEN_W - TEXT_X - MARGIN)
#define HINT_Y       (SCREEN_H - 20)
#define DESC_Y       (IMG_Y + IMG_MAX_H + 10)

// Font: 5x7 bitmap (6px advance)
#define FONT_W 5
#define FONT_H 7
#define FONT_ADV 6
#define LINE_H 10

// Palette indices
#define COL_BG      0
#define COL_WHITE   1
#define COL_GRAY    2
#define COL_TITLE   3
#define COL_VALUE   4
#define COL_HINT    5
#define COL_IMGBG   6
#define COL_BORDER  7

// ============================================================================
// Globals
// ============================================================================

struct dvi_inst dvi0;
struct semaphore dvi_start_sem;

static uint8_t framebuffer[SCREEN_W * SCREEN_H];
static uint32_t palette_rgb888[256];
static uint8_t palette_rgb332[256];
static uint8_t scanline_rgb332[SCREEN_W] __attribute__((aligned(4)));

static FATFS fs;

// 5x7 font glyphs (ASCII 32-126)
static const uint8_t font_5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // space
    {0x00,0x00,0x5F,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14}, // #
    {0x24,0x2A,0x7F,0x2A,0x12}, // $
    {0x23,0x13,0x08,0x64,0x62}, // %
    {0x36,0x49,0x55,0x22,0x50}, // &
    {0x00,0x00,0x07,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00}, // )
    {0x14,0x08,0x3E,0x08,0x14}, // *
    {0x08,0x08,0x3E,0x08,0x08}, // +
    {0x00,0x50,0x30,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08}, // -
    {0x00,0x60,0x60,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02}, // /
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x36,0x36,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00}, // ;
    {0x08,0x14,0x22,0x41,0x00}, // <
    {0x14,0x14,0x14,0x14,0x14}, // =
    {0x00,0x41,0x22,0x14,0x08}, // >
    {0x02,0x01,0x51,0x09,0x06}, // ?
    {0x3E,0x41,0x5D,0x55,0x5E}, // @
    {0x7E,0x09,0x09,0x09,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x0C,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x3F,0x40,0x38,0x40,0x3F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x07,0x08,0x70,0x08,0x07}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
    {0x00,0x7F,0x41,0x41,0x00}, // [
    {0x02,0x04,0x08,0x10,0x20}, // backslash
    {0x00,0x41,0x41,0x7F,0x00}, // ]
    {0x04,0x02,0x01,0x02,0x04}, // ^
    {0x40,0x40,0x40,0x40,0x40}, // _
    {0x00,0x01,0x02,0x04,0x00}, // `
    {0x20,0x54,0x54,0x54,0x78}, // a
    {0x7F,0x48,0x44,0x44,0x38}, // b
    {0x38,0x44,0x44,0x44,0x20}, // c
    {0x38,0x44,0x44,0x48,0x7F}, // d
    {0x38,0x54,0x54,0x54,0x18}, // e
    {0x08,0x7E,0x09,0x01,0x02}, // f
    {0x0C,0x52,0x52,0x52,0x3E}, // g
    {0x7F,0x08,0x04,0x04,0x78}, // h
    {0x00,0x44,0x7D,0x40,0x00}, // i
    {0x20,0x40,0x44,0x3D,0x00}, // j
    {0x7F,0x10,0x28,0x44,0x00}, // k
    {0x00,0x41,0x7F,0x40,0x00}, // l
    {0x7C,0x04,0x18,0x04,0x78}, // m
    {0x7C,0x08,0x04,0x04,0x78}, // n
    {0x38,0x44,0x44,0x44,0x38}, // o
    {0x7C,0x14,0x14,0x14,0x08}, // p
    {0x08,0x14,0x14,0x18,0x7C}, // q
    {0x7C,0x08,0x04,0x04,0x08}, // r
    {0x48,0x54,0x54,0x54,0x20}, // s
    {0x04,0x3F,0x44,0x40,0x20}, // t
    {0x3C,0x40,0x40,0x20,0x7C}, // u
    {0x1C,0x20,0x40,0x20,0x1C}, // v
    {0x3C,0x40,0x30,0x40,0x3C}, // w
    {0x44,0x28,0x10,0x28,0x44}, // x
    {0x0C,0x50,0x50,0x50,0x3C}, // y
    {0x44,0x64,0x54,0x4C,0x44}, // z
    {0x00,0x08,0x36,0x41,0x00}, // {
    {0x00,0x00,0x7F,0x00,0x00}, // |
    {0x00,0x41,0x36,0x08,0x00}, // }
    {0x08,0x04,0x08,0x10,0x08}, // ~
};

// ============================================================================
// Drawing primitives
// ============================================================================

static inline void fb_pixel(int x, int y, uint8_t c) {
    if (x >= 0 && x < SCREEN_W && y >= 0 && y < SCREEN_H)
        framebuffer[y * SCREEN_W + x] = c;
}

static void fb_rect(int x, int y, int w, int h, uint8_t c) {
    for (int j = y; j < y + h; j++)
        for (int i = x; i < x + w; i++)
            fb_pixel(i, j, c);
}

static void fb_char(int x, int y, char ch, uint8_t c) {
    if (ch < 32 || ch > 126) ch = '?';
    const uint8_t *glyph = font_5x7[ch - 32];
    for (int col = 0; col < FONT_W; col++)
        for (int row = 0; row < FONT_H; row++)
            if (glyph[col] & (1 << row))
                fb_pixel(x + col, y + row, c);
}

static void fb_text(int x, int y, const char *s, uint8_t c) {
    while (*s) {
        fb_char(x, y, *s, c);
        x += FONT_ADV;
        s++;
    }
}

// Draw text, return number of lines used. Wraps at max_w pixels.
static int fb_text_wrap(int x, int y, const char *s, uint8_t c, int max_w) {
    int cx = x, cy = y, lines = 1;
    int max_chars = max_w / FONT_ADV;
    while (*s) {
        // Find next word boundary
        const char *word = s;
        int wlen = 0;
        while (*word && *word != ' ' && *word != '\n') { word++; wlen++; }

        // Wrap if word doesn't fit
        if (cx > x && (cx - x) / FONT_ADV + wlen > max_chars) {
            cx = x;
            cy += LINE_H;
            lines++;
        }

        // Draw word
        while (s < word) {
            fb_char(cx, cy, *s, c);
            cx += FONT_ADV;
            s++;
        }

        // Handle space/newline
        if (*s == '\n') { cx = x; cy += LINE_H; lines++; s++; }
        else if (*s == ' ') { cx += FONT_ADV; s++; }
    }
    return lines;
}

static void fb_text_center(int y, const char *s, uint8_t c) {
    int w = strlen(s) * FONT_ADV;
    fb_text((SCREEN_W - w) / 2, y, s, c);
}

// ============================================================================
// UF2 entry data
// ============================================================================

typedef struct {
    char filename[48];      // e.g. "murmsnes.uf2"
    char basename[48];      // e.g. "murmsnes" (without .uf2)
    // Metadata from XML
    char title[64];
    char version[32];
    char author[64];
    char website[128];
    char description[MAX_TEXT];
    char controls[128];
    // Cover art
    bool has_bmp;
    uint8_t bmp_palette[256]; // RGB332 palette for BMP
    int bmp_w, bmp_h;
    uint8_t *bmp_pixels;     // allocated, bmp_w * bmp_h bytes (palette indices)
} uf2_entry_t;

static uf2_entry_t entries[MAX_ENTRIES];
static int entry_count = 0;
static int selected = 0;

// Shared BMP pixel buffer (reused per entry since only one displayed at a time)
static uint8_t bmp_pixel_buf[200 * 200]; // max 200x200 cover art

// ============================================================================
// Simple XML parser (extracts tag values)
// ============================================================================

static void xml_extract(const char *xml, const char *tag, char *out, int max_len) {
    out[0] = '\0';
    char open[64], close[64];
    snprintf(open, sizeof(open), "<%s>", tag);
    snprintf(close, sizeof(close), "</%s>", tag);

    const char *start = strstr(xml, open);
    if (!start) return;
    start += strlen(open);

    const char *end = strstr(start, close);
    if (!end) return;

    int len = end - start;
    if (len >= max_len) len = max_len - 1;
    memcpy(out, start, len);
    out[len] = '\0';
}

static void palette_set(uint8_t index, uint32_t rgb888);

// ============================================================================
// BMP loader (into entry's pixel buffer)
// ============================================================================

static bool load_entry_bmp(uf2_entry_t *e, const char *path) {
    FIL file;
    UINT br;

    if (f_open(&file, path, FA_READ) != FR_OK) return false;

    uint8_t hdr[54];
    if (f_read(&file, hdr, 54, &br) != FR_OK || br != 54 ||
        hdr[0] != 'B' || hdr[1] != 'M') {
        f_close(&file);
        return false;
    }

    uint32_t pixel_offset = hdr[10] | (hdr[11] << 8) | (hdr[12] << 16) | (hdr[13] << 24);
    int32_t  bmp_w = hdr[18] | (hdr[19] << 8) | (hdr[20] << 16) | (hdr[21] << 24);
    int32_t  bmp_h = hdr[22] | (hdr[23] << 8) | (hdr[24] << 16) | (hdr[25] << 24);
    uint16_t bpp   = hdr[28] | (hdr[29] << 8);
    uint32_t comp  = hdr[30] | (hdr[31] << 8) | (hdr[32] << 16) | (hdr[33] << 24);

    bool top_down = (bmp_h < 0);
    if (top_down) bmp_h = -bmp_h;

    if (bpp != 8 || comp != 0 || bmp_w > 200 || bmp_h > 200) {
        f_close(&file);
        return false;
    }

    // Read palette
    uint8_t pal[256 * 4];
    uint32_t pal_size = pixel_offset - 54;
    if (pal_size > sizeof(pal)) pal_size = sizeof(pal);
    if (f_read(&file, pal, pal_size, &br) != FR_OK || br != pal_size) {
        f_close(&file);
        return false;
    }

    // Store BMP palette as screen palette indices starting at 16
    // (0-15 reserved for UI colors)
    int ncolors = pal_size / 4;
    if (ncolors > 240) ncolors = 240;
    for (int i = 0; i < ncolors; i++) {
        uint8_t b = pal[i * 4 + 0], g = pal[i * 4 + 1], r = pal[i * 4 + 2];
        e->bmp_palette[i] = 16 + i;  // map BMP index i -> screen palette 16+i
        palette_set(16 + i, (r << 16) | (g << 8) | b);
    }

    f_lseek(&file, pixel_offset);

    int row_stride = (bmp_w + 3) & ~3;
    uint8_t row_buf[256];

    e->bmp_w = bmp_w;
    e->bmp_h = bmp_h;
    e->bmp_pixels = bmp_pixel_buf;

    for (int row = 0; row < bmp_h; row++) {
        if (f_read(&file, row_buf, row_stride, &br) != FR_OK) break;
        int dst_y = top_down ? row : (bmp_h - 1 - row);
        memcpy(&e->bmp_pixels[dst_y * bmp_w], row_buf, bmp_w);
    }

    f_close(&file);
    e->has_bmp = true;
    return true;
}

// ============================================================================
// Scan directory and load metadata
// ============================================================================

static void scan_entries(void) {
    DIR dir;
    FILINFO fno;

    if (f_opendir(&dir, BASE_DIR) != FR_OK) {
        printf("Cannot open " BASE_DIR "\n");
        return;
    }

    entry_count = 0;
    while (entry_count < MAX_ENTRIES) {
        if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == '\0') break;
        if (fno.fattrib & AM_DIR) continue;

        // Check for .uf2 extension
        const char *ext = strrchr(fno.fname, '.');
        if (!ext || (strcasecmp(ext, ".uf2") != 0)) continue;

        uf2_entry_t *e = &entries[entry_count];
        memset(e, 0, sizeof(*e));

        strncpy(e->filename, fno.fname, sizeof(e->filename) - 1);

        // Derive basename (without .uf2)
        strncpy(e->basename, fno.fname, sizeof(e->basename) - 1);
        char *dot = strrchr(e->basename, '.');
        if (dot) *dot = '\0';

        // Default title = basename
        strncpy(e->title, e->basename, sizeof(e->title) - 1);

        entry_count++;
    }
    f_closedir(&dir);

    printf("Found %d UF2 file(s)\n", entry_count);
}

static void load_entry_metadata(int idx) {
    uf2_entry_t *e = &entries[idx];
    char path[MAX_PATH];

    // Try loading XML
    snprintf(path, sizeof(path), BASE_DIR "/%s.xml", e->basename);
    FIL file;
    UINT br;
    if (f_open(&file, path, FA_READ) == FR_OK) {
        static char xml_buf[1024];
        UINT read;
        f_read(&file, xml_buf, sizeof(xml_buf) - 1, &read);
        xml_buf[read] = '\0';
        f_close(&file);

        xml_extract(xml_buf, "title", e->title, sizeof(e->title));
        xml_extract(xml_buf, "version", e->version, sizeof(e->version));
        xml_extract(xml_buf, "author", e->author, sizeof(e->author));
        xml_extract(xml_buf, "website", e->website, sizeof(e->website));
        xml_extract(xml_buf, "description", e->description, sizeof(e->description));
        xml_extract(xml_buf, "controls", e->controls, sizeof(e->controls));

        if (e->title[0] == '\0')
            strncpy(e->title, e->basename, sizeof(e->title) - 1);
    }

    // Try loading BMP
    snprintf(path, sizeof(path), BASE_DIR "/%s.bmp", e->basename);
    load_entry_bmp(e, path);
}

// ============================================================================
// UI rendering
// ============================================================================

static inline uint8_t rgb888_to_rgb332(uint32_t rgb888) {
    uint8_t r = (rgb888 >> 16) & 0xFF;
    uint8_t g = (rgb888 >> 8) & 0xFF;
    uint8_t b = rgb888 & 0xFF;
    return ((r >> 5) << 5) | ((g >> 5) << 2) | (b >> 6);
}

static void palette_set(uint8_t index, uint32_t rgb888) {
    palette_rgb888[index] = rgb888 & 0x00FFFFFF;
    palette_rgb332[index] = rgb888_to_rgb332(rgb888);
}

static void setup_palette(void) {
    palette_set(COL_BG,     0x1A1A2E);  // dark blue-gray background
    palette_set(COL_WHITE,  0xFFFFFF);
    palette_set(COL_GRAY,   0x888888);
    palette_set(COL_TITLE,  0x00D4FF);  // cyan title
    palette_set(COL_VALUE,  0xE0E0E0);  // light gray values
    palette_set(COL_HINT,   0x666666);  // dim hint text
    palette_set(COL_IMGBG,  0x0D0D1A);  // image background
    palette_set(COL_BORDER, 0x333355);  // border
    // Indices 16-255 reserved for BMP palette (set when BMP is loaded)
}

static void draw_label(int x, int y, const char *label, const char *value) {
    fb_text(x, y, label, COL_GRAY);
    fb_text(x + strlen(label) * FONT_ADV, y, value, COL_VALUE);
}

static void render_entry(int idx) {
    // Clear
    memset(framebuffer, COL_BG, sizeof(framebuffer));

    if (entry_count == 0) {
        fb_text_center(SCREEN_H / 2 - 4, "No UF2 files found", COL_WHITE);
        fb_text_center(SCREEN_H / 2 + 10, "Place .uf2 files in /kickstarter", COL_GRAY);
        return;
    }

    uf2_entry_t *e = &entries[idx];

    // Draw image (no border)
    if (e->has_bmp && e->bmp_pixels) {
        int ox = IMG_X;
        int oy = IMG_Y;
        for (int y = 0; y < e->bmp_h; y++) {
            for (int x = 0; x < e->bmp_w; x++) {
                uint8_t pi = e->bmp_pixels[y * e->bmp_w + x];
                fb_pixel(ox + x, oy + y, e->bmp_palette[pi]);
            }
        }
    }

    // Title + metadata to the right of image
    int ty = TEXT_Y;
    fb_text(TEXT_X, ty, e->title, COL_TITLE);
    ty += LINE_H + 4;

    if (e->version[0]) { draw_label(TEXT_X, ty, "Version: ", e->version); ty += LINE_H; }
    if (e->author[0])  { draw_label(TEXT_X, ty, "Author: ", e->author); ty += LINE_H; }
    if (e->website[0]) { draw_label(TEXT_X, ty, "Web: ", e->website); ty += LINE_H; }

    // Description below metadata (still to the right of image)
    if (e->description[0]) {
        ty += 4;
        int lines = fb_text_wrap(TEXT_X, ty, e->description, COL_VALUE, TEXT_W);
        ty += lines * LINE_H;
    }

    // Controls below description (with wrapping)
    if (e->controls[0]) {
        ty += 4;
        fb_text(TEXT_X, ty, "Controls:", COL_GRAY);
        ty += LINE_H;
        int lines = fb_text_wrap(TEXT_X, ty, e->controls, COL_VALUE, TEXT_W);
        ty += lines * LINE_H;
    }

    // Filename below controls
    ty += 4;
    fb_text(TEXT_X, ty, e->filename, COL_GRAY);

    // Bottom hint bar
    char hint[64];
    snprintf(hint, sizeof(hint), "< LEFT   %d / %d   RIGHT >", selected + 1, entry_count);
    fb_text_center(HINT_Y, hint, COL_HINT);
}

// ============================================================================
// DVI output (Core 1)
// ============================================================================

static void __not_in_flash_func(flash_timings)(void) {
#if !PICO_RP2040
    const int max_flash_freq = 66 * 1000000;
    const int clock_hz = DVI_TIMING.bit_clk_khz * 1000;
    int divisor = (clock_hz + max_flash_freq - 1) / max_flash_freq;
    if (divisor == 1 && clock_hz > 100000000) divisor = 2;
    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000) rxdelay += 1;
    qmi_hw->m[0].timing = 0x60007000 |
                        rxdelay << QMI_M0_TIMING_RXDELAY_LSB |
                        divisor << QMI_M0_TIMING_CLKDIV_LSB;
#endif
    sleep_ms(100);
    set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
}

void __not_in_flash_func(core1_main)(void) {
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_1);
    dvi_start(&dvi0);
    sem_acquire_blocking(&dvi_start_sem);

    uint pixwidth = dvi0.timing->h_active_pixels;
    uint words_per_channel = pixwidth / DVI_SYMBOLS_PER_WORD;
    uint32_t *tmdsbuf;

    while (true) {
        for (uint y = 0; y < SCREEN_H; y++) {
            queue_remove_blocking_u32(&dvi0.q_tmds_free, &tmdsbuf);

            const uint8_t *src = &framebuffer[y * SCREEN_W];
            for (int x = 0; x < SCREEN_W; x++)
                scanline_rgb332[x] = palette_rgb332[src[x]];

            const uint32_t *pixbuf = (const uint32_t *)scanline_rgb332;
            tmds_encode_data_channel_8bpp(pixbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_8BPP_BLUE_MSB,  DVI_8BPP_BLUE_LSB);
            tmds_encode_data_channel_8bpp(pixbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_8BPP_GREEN_MSB, DVI_8BPP_GREEN_LSB);
            tmds_encode_data_channel_8bpp(pixbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_8BPP_RED_MSB,   DVI_8BPP_RED_LSB);

            queue_add_blocking_u32(&dvi0.q_tmds_valid, &tmdsbuf);
        }
    }
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    vreg_disable_voltage_limit();
    vreg_set_voltage(VREG_VOLTAGE_1_60);
    flash_timings();
    sleep_ms(10);

    stdio_init_all();

    for (int i = 3; i > 0; i--) {
        printf("Starting in %d...\n", i);
        sleep_ms(1000);
    }

    printf("Kickstarter - UF2 Launcher\n");
    printf("Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);

    // Setup palette and clear
    setup_palette();
    memset(framebuffer, COL_BG, sizeof(framebuffer));

    // Init DVI
    dvi0.timing = &DVI_TIMING;
    dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
    dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

    // Mount SD
    printf("Mounting SD card...\n");
    FRESULT res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        printf("SD mount failed: %d\n", res);
        fb_text_center(SCREEN_H / 2, "SD Card Error", COL_WHITE);
        goto start_dvi;
    }
    printf("SD card mounted.\n");

    // Scan for UF2 files
    scan_entries();

    // Load first entry metadata
    if (entry_count > 0) {
        load_entry_metadata(0);
    }

    // Render initial screen
    render_entry(selected);

start_dvi:
    // Launch Core 1 for DVI
    sem_init(&dvi_start_sem, 0, 1);
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);
    multicore_launch_core1(core1_main);
    sem_release(&dvi_start_sem);
    printf("DVI started.\n");

    // Main loop: handle input
    // TODO: Add NES gamepad and PS/2 keyboard input
    // For now, use USB serial: 'l' = left, 'r' = right
    while (1) {
        int c = getchar_timeout_us(50000); // 50ms poll
        if (c == PICO_ERROR_TIMEOUT) continue;

        bool changed = false;
        if ((c == 'l' || c == 'h') && entry_count > 0) {
            selected = (selected - 1 + entry_count) % entry_count;
            changed = true;
        }
        if ((c == 'r' || c == 'l' + 6) && entry_count > 0) { // 'r' or 'l'
            selected = (selected + 1) % entry_count;
            changed = true;
        }

        if (changed) {
            load_entry_metadata(selected);
            render_entry(selected);
            printf("Selected: %s (%d/%d)\n", entries[selected].filename, selected + 1, entry_count);
        }
    }
}
