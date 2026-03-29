/*
 * Frank-Kickstart - UF2 Firmware Launcher for RP2350
 *
 * Scans /kickstart on SD card for .uf2 files.
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

#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "board_config.h"
#include "nespad.h"
#include "ps2kbd_wrapper.h"
#include "dvi.h"
#include "dvi_timing.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "tmds_encode.h"
#include "ff.h"
#if !PICO_RP2040
#include "psram_init.h"
#endif
#include "welcome_data.h"

// ============================================================================
// Configuration
// ============================================================================

#define SCREEN_W 400
#define SCREEN_H 300
#define DVI_TIMING dvi_timing_800x600p_60hz
#define BASE_DIR "/kickstart"
#define MAX_ENTRIES_SRAM  16
#define MAX_ENTRIES_PSRAM 128
#define PSRAM_BASE        ((uint8_t *)(intptr_t)0x11000000)
#define MAX_PATH 128
#define MAX_TEXT 512

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
static volatile bool core1_stop = false;

static uint8_t framebuffer[SCREEN_W * SCREEN_H];
static uint32_t palette_rgb888[256];
static uint8_t palette_rgb332[256];
static volatile bool dvi_loading = false;  // When true, Core 1 outputs blank instead of encoding
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
    uint8_t bmp_palette[256]; // maps BMP index -> screen palette index
    uint32_t bmp_rgb888[256]; // BMP palette colors for reload on display
    int bmp_ncolors;
    int bmp_w, bmp_h;
    uint8_t *bmp_pixels;     // allocated, bmp_w * bmp_h bytes (palette indices)
} uf2_entry_t;

static uf2_entry_t entries_sram[MAX_ENTRIES_SRAM];  // fallback when no PSRAM
static uf2_entry_t *entries = entries_sram;
static int max_entries = MAX_ENTRIES_SRAM;
static int entry_count = 0;
static int selected = 0;
static bool has_psram = false;

// Simple PSRAM bump allocator (entries array sits at PSRAM_BASE, allocs follow)
static size_t psram_alloc_offset = 0;
#define PSRAM_SIZE (8 * 1024 * 1024)

static void *psram_malloc(size_t size) {
    if (!has_psram) return malloc(size);
    size = (size + 3) & ~3;  // align to 4 bytes
    if (psram_alloc_offset + size > PSRAM_SIZE) return NULL;
    void *ptr = PSRAM_BASE + psram_alloc_offset;
    psram_alloc_offset += size;
    return ptr;
}

// Temporary BMP pixel buffer for loading (copied to malloc'd per-entry storage)
static uint8_t bmp_pixel_buf[200 * 200];

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
    e->bmp_ncolors = ncolors;
    for (int i = 0; i < ncolors; i++) {
        uint8_t b = pal[i * 4 + 0], g = pal[i * 4 + 1], r = pal[i * 4 + 2];
        e->bmp_palette[i] = 16 + i;
        e->bmp_rgb888[i] = (r << 16) | (g << 8) | b;
    }

    f_lseek(&file, pixel_offset);

    int row_stride = (bmp_w + 3) & ~3;
    uint8_t row_buf[256];

    e->bmp_w = bmp_w;
    e->bmp_h = bmp_h;

    // Read into temp buffer, then malloc per-entry storage
    for (int row = 0; row < bmp_h; row++) {
        if (f_read(&file, row_buf, row_stride, &br) != FR_OK) break;
        int dst_y = top_down ? row : (bmp_h - 1 - row);
        memcpy(&bmp_pixel_buf[dst_y * bmp_w], row_buf, bmp_w);
    }
    f_close(&file);

    e->bmp_pixels = (uint8_t *)psram_malloc(bmp_w * bmp_h);
    if (!e->bmp_pixels) return false;
    memcpy(e->bmp_pixels, bmp_pixel_buf, bmp_w * bmp_h);
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
    while (entry_count < max_entries) {
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

    // Reset metadata (keep filename/basename)
    e->title[0] = e->version[0] = e->author[0] = 0;
    e->website[0] = e->description[0] = e->controls[0] = 0;
    e->has_bmp = false;
    strncpy(e->title, e->basename, sizeof(e->title) - 1);

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
    // Clear framebuffer in small chunks to avoid bus starvation of Core 1 DVI
    for (int y = 0; y < SCREEN_H; y++)
        memset(&framebuffer[y * SCREEN_W], COL_BG, SCREEN_W);

    if (entry_count == 0) {
        fb_text_center(SCREEN_H / 2 - 4, "No UF2 files found", COL_WHITE);
        fb_text_center(SCREEN_H / 2 + 10, "Place .uf2 files in /kickstart", COL_GRAY);
        return;
    }

    uf2_entry_t *e = &entries[idx];

    // Reload BMP palette into screen palette for this entry
    if (e->has_bmp) {
        for (int i = 0; i < e->bmp_ncolors; i++)
            palette_set(16 + i, e->bmp_rgb888[i]);
    }

    // Draw image
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
// UF2 Flashing (from pico-launcher m1p2-uf2)
// ============================================================================

typedef struct {
    uint32_t magicStart0;
    uint32_t magicStart1;
    uint32_t flags;
    uint32_t targetAddr;
    uint32_t payloadSize;
    uint32_t blockNo;
    uint32_t numBlocks;
    uint32_t fileSize;
    uint8_t data[476];
    uint32_t magicEnd;
} UF2_Block_t;

#define ZERO_BLOCK_OFFSET   ((16ul << 20) - (256ul << 10) - (4ul << 10))
#define ZERO_BLOCK_ADDRESS  (XIP_BASE + ZERO_BLOCK_OFFSET)
#define FLASH_MAGIC_OVER    0x3836d91au
#define SRAM_MAGIC_BOOT     0x383da910u

static inline int __not_in_flash_func(memcmp32)(const uint32_t *p1, const uint32_t *p2, size_t len) {
    len >>= 2;
    while (len--) { if (*p1++ != *p2++) return 1; }
    return 0;
}

static bool __not_in_flash_func(flash_uf2)(const char *path) {
    FIL file;
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Stop DVI BEFORE any SD access (APB contention at 400 MHz)
    core1_stop = true;
    sleep_ms(100);

    // Lower clock — ROM flash functions reset QMI, XIP fails at 400 MHz
    set_sys_clock_khz(150000, true);
    sleep_ms(50);

    // Now lockout can work — DVI IRQ is no longer firing
    multicore_lockout_start_blocking();

    if (f_open(&file, path, FA_READ) != FR_OK) {
        multicore_lockout_end_blocking();
        return false;
    }
    uint32_t ints = save_and_disable_interrupts();

    uint32_t flash_offset = 0;
    while (true) {
        uint8_t buffer[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));
        UINT bytes_read = 0;
        UF2_Block_t uf2;
        uint32_t next_offset = flash_offset;

        for (uint32_t idx = 0; idx < FLASH_SECTOR_SIZE; idx += 256) {
            f_read(&file, &uf2, sizeof(UF2_Block_t), &bytes_read);
            if (!bytes_read) break;
            if (uf2.targetAddr == XIP_BASE + 0xFFFF00) {
                f_read(&file, &uf2, sizeof(UF2_Block_t), &bytes_read);
                if (!bytes_read) break;
            }
            if (next_offset != uf2.targetAddr - XIP_BASE) {
                f_lseek(&file, f_tell(&file) - sizeof(UF2_Block_t));
                next_offset = uf2.targetAddr - XIP_BASE;
                break;
            }
            memcpy(buffer + idx, uf2.data, 256);
            next_offset += 256;
            gpio_put(PICO_DEFAULT_LED_PIN, (next_offset >> 13) & 1);
        }

        if (next_offset == flash_offset) break;

        // Sector 0: save to ZERO_BLOCK with magic, patch Reset vector
        if (flash_offset == 0) {
            uint32_t *v = (uint32_t *)buffer;
            uint32_t orig = v[1023];
            v[1023] = FLASH_MAGIC_OVER;
            if (memcmp32((uint32_t *)buffer, (uint32_t *)ZERO_BLOCK_ADDRESS, FLASH_SECTOR_SIZE)) {
                flash_range_erase(ZERO_BLOCK_OFFSET, FLASH_SECTOR_SIZE);
                flash_range_program(ZERO_BLOCK_OFFSET, buffer, FLASH_SECTOR_SIZE);
            }
            v[1] = *(uint32_t *)(XIP_BASE + 4);  // patch Reset to frank-kickstart
            v[1023] = orig;
        }

        if (memcmp32((uint32_t *)buffer, (uint32_t *)(flash_offset + XIP_BASE), FLASH_SECTOR_SIZE)) {
            flash_range_erase(flash_offset, FLASH_SECTOR_SIZE);
            flash_range_program(flash_offset, buffer, FLASH_SECTOR_SIZE);
        }

        flash_offset = next_offset;
    }

    restore_interrupts(ints);
    multicore_lockout_end_blocking();
    gpio_put(PICO_DEFAULT_LED_PIN, false);
    f_close(&file);

    // SRAM magic tells before_main() to jump to firmware on next boot
    *(volatile uint32_t *)(0x20000000 + (512 << 10) - 8) = SRAM_MAGIC_BOOT;
    watchdog_enable(100, true);
    while (true) tight_loop_contents();
    return true;
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
    multicore_lockout_victim_init();
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_1);
    dvi_start(&dvi0);
    sem_acquire_blocking(&dvi_start_sem);

    uint pixwidth = dvi0.timing->h_active_pixels;
    uint words_per_channel = pixwidth / DVI_SYMBOLS_PER_WORD;
    uint32_t *tmdsbuf;

    // Pre-compute blank scanline (background color TMDS)
    memset(scanline_rgb332, palette_rgb332[COL_BG], SCREEN_W);
    const uint32_t *blankbuf = (const uint32_t *)scanline_rgb332;
    static uint32_t blank_tmds[3 * (800 / DVI_SYMBOLS_PER_WORD)];
    tmds_encode_data_channel_8bpp(blankbuf, blank_tmds + 0 * words_per_channel, pixwidth / 2, DVI_8BPP_BLUE_MSB,  DVI_8BPP_BLUE_LSB);
    tmds_encode_data_channel_8bpp(blankbuf, blank_tmds + 1 * words_per_channel, pixwidth / 2, DVI_8BPP_GREEN_MSB, DVI_8BPP_GREEN_LSB);
    tmds_encode_data_channel_8bpp(blankbuf, blank_tmds + 2 * words_per_channel, pixwidth / 2, DVI_8BPP_RED_MSB,   DVI_8BPP_RED_LSB);

    while (true) {
        for (uint y = 0; y < SCREEN_H; y++) {
            queue_remove_blocking_u32(&dvi0.q_tmds_free, &tmdsbuf);

            if (dvi_loading) {
                // During SD access: output pre-computed blank — no SRAM reads that compete with Core 0
                memcpy(tmdsbuf, blank_tmds, 3 * words_per_channel * sizeof(uint32_t));
            } else {
                const uint8_t *src = &framebuffer[y * SCREEN_W];
                for (int x = 0; x < SCREEN_W; x++)
                    scanline_rgb332[x] = palette_rgb332[src[x]];

                const uint32_t *pixbuf = (const uint32_t *)scanline_rgb332;
                tmds_encode_data_channel_8bpp(pixbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_8BPP_BLUE_MSB,  DVI_8BPP_BLUE_LSB);
                tmds_encode_data_channel_8bpp(pixbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_8BPP_GREEN_MSB, DVI_8BPP_GREEN_LSB);
                tmds_encode_data_channel_8bpp(pixbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_8BPP_RED_MSB,   DVI_8BPP_RED_LSB);
            }

            queue_add_blocking_u32(&dvi0.q_tmds_valid, &tmdsbuf);
        }

        // Core 0 requested stop for flash — enter RAM-only idle
        if (core1_stop) {
            // Disable DVI PIO and DMA so no APB traffic
            pio_set_sm_mask_enabled(dvi0.ser_cfg.pio, 0x0F, false);
            for (int i = 0; i < N_TMDS_LANES; i++) {
                dma_channel_abort(dvi0.dma_cfg[i].chan_ctrl);
                dma_channel_abort(dvi0.dma_cfg[i].chan_data);
            }
            // Now lockout handler can fire (no more DMA IRQ)
            while (true) __wfe();
        }
    }
}

// ============================================================================
// Welcome screen
// ============================================================================

// Welcome palette uses indices 16+ (same range as BMP palettes)
#define WELCOME_PAL_BASE 16

static void render_welcome(void) {
    memset(framebuffer, COL_BG, sizeof(framebuffer));

    // Load PCB palette into screen palette
    for (int i = 1; i < 14; i++)  // skip index 0 (transparent)
        palette_set(WELCOME_PAL_BASE + i, pcb_palette[i]);

    // Draw PCB at 3x scale, centered horizontally
    int pcb_scale = 3;
    int pcb_draw_w = PCB_W * pcb_scale;
    int pcb_draw_h = PCB_H * pcb_scale;
    // Vertically center: PCB + gap + FRANK
    int gap = 16;
    int total_h = pcb_draw_h + gap + FRANK_H;
    int pcb_ox = (SCREEN_W - pcb_draw_w) / 2;
    int pcb_oy = (SCREEN_H - total_h) / 2;

    for (int y = 0; y < PCB_H; y++) {
        for (int x = 0; x < PCB_W; x++) {
            uint8_t pi = pcb_pixels[y * PCB_W + x];
            if (pi == 0) continue;  // transparent
            uint8_t ci = WELCOME_PAL_BASE + pi;
            for (int sy = 0; sy < pcb_scale; sy++)
                for (int sx = 0; sx < pcb_scale; sx++)
                    fb_pixel(pcb_ox + x * pcb_scale + sx,
                             pcb_oy + y * pcb_scale + sy, ci);
        }
    }

    // Draw FRANK text at 1x (native pixel art), centered, below PCB
    int frank_ox = (SCREEN_W - FRANK_W) / 2;
    int frank_oy = pcb_oy + pcb_draw_h + gap;

    for (int y = 0; y < FRANK_H; y++) {
        for (int bx = 0; bx < FRANK_BPR; bx++) {
            uint8_t byte = frank_bitmap[y * FRANK_BPR + bx];
            for (int bit = 0; bit < 8; bit++) {
                int px = bx * 8 + bit;
                if (px >= FRANK_W) break;
                if (!(byte & (0x80 >> bit))) continue;
                fb_pixel(frank_ox + px, frank_oy + y, COL_WHITE);
            }
        }
    }
}

// ============================================================================
// PSRAM detection
// ============================================================================

#if !PICO_RP2040
static bool psram_detect(void) {
    uint cs_pin = get_psram_pin();
    psram_init(cs_pin);

    // Write test pattern and read back to verify PSRAM is present
    volatile uint8_t *psram = (volatile uint8_t *)PSRAM_BASE;
    psram[0] = 0xA5;
    psram[1] = 0x5A;
    psram[2] = 0x42;
    if (psram[0] != 0xA5 || psram[1] != 0x5A || psram[2] != 0x42)
        return false;

    // Second pattern to rule out bus floating
    psram[0] = 0x00;
    if (psram[0] != 0x00)
        return false;

    return true;
}
#endif

// ============================================================================
// Main
// ============================================================================

int main(void) {
    vreg_disable_voltage_limit();
    vreg_set_voltage(VREG_VOLTAGE_1_60);
    flash_timings();
    sleep_ms(10);

    stdio_init_all();

    // Early gamepad init for escape check
    nespad_begin(clock_get_hz(clk_sys) / 1000, NESPAD_GPIO_CLK, NESPAD_GPIO_DATA, NESPAD_GPIO_LATCH);
    sleep_ms(50);
    nespad_read();

    // Boot redirect handled by before_main() constructor in fw_boot_redirect.c

    printf("Frank-Kickstart - UF2 Launcher\n");
    printf("Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);

    // Detect PSRAM and allocate entries accordingly
#if !PICO_RP2040
    has_psram = psram_detect();
    if (has_psram) {
        entries = (uf2_entry_t *)PSRAM_BASE;
        max_entries = MAX_ENTRIES_PSRAM;
        // Reserve space for entries array, bump allocator starts after
        psram_alloc_offset = sizeof(uf2_entry_t) * MAX_ENTRIES_PSRAM;
        psram_alloc_offset = (psram_alloc_offset + 3) & ~3;
        printf("PSRAM detected - %d entry slots available\n", max_entries);
    } else {
        printf("No PSRAM - using SRAM (%d entry slots)\n", max_entries);
    }
#else
    printf("RP2040 - using SRAM (%d entry slots)\n", max_entries);
#endif

    // Setup palette and render welcome screen into framebuffer
    setup_palette();
    render_welcome();

    // Init DVI (but don't start yet — need SD first)
    dvi0.timing = &DVI_TIMING;
    dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
    dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

    // Mount SD and load all data BEFORE starting DVI
    // (SPI and DVI are incompatible at 400 MHz — APB bus contention)
    printf("Mounting SD card...\n");
    FRESULT res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        printf("SD mount failed: %d\n", res);
        memset(framebuffer, COL_BG, sizeof(framebuffer));
        fb_text_center(SCREEN_H / 2, "SD Card Error", COL_WHITE);
        goto start_dvi;
    }
    printf("SD card mounted.\n");

    // Initialize PS/2 keyboard (nespad already initialized for escape check)
    ps2kbd_init();
    printf("PS/2 keyboard initialized (CLK=%d, DATA=%d)\n", PS2_PIN_CLK, PS2_PIN_DATA);

    // Scan for UF2 files and pre-load ALL metadata before DVI starts
    scan_entries();
    for (int i = 0; i < entry_count; i++) {
        load_entry_metadata(i);
        printf("  [%d/%d] %s\n", i + 1, entry_count, entries[i].title);
    }
    printf("All data pre-loaded.\n");

start_dvi:
    // Launch Core 1 for DVI — welcome screen shows immediately
    sem_init(&dvi_start_sem, 0, 1);
    hw_set_bits(&bus_ctrl_hw->priority,
        BUSCTRL_BUS_PRIORITY_PROC1_BITS |   // Core 1 CPU priority
        BUSCTRL_BUS_PRIORITY_DMA_R_BITS |   // DMA read priority (TMDS -> PIO)
        BUSCTRL_BUS_PRIORITY_DMA_W_BITS);   // DMA write priority
    multicore_launch_core1(core1_main);
    sem_release(&dvi_start_sem);
    printf("DVI started (welcome screen).\n");

    // Show welcome screen for 2.5 seconds, then switch to firmware list
    sleep_ms(2500);
    if (entry_count > 0)
        render_entry(selected);

    // Main loop: handle input from gamepad, keyboard, and serial
    uint32_t repeat_timer = 0;
    uint32_t prev_input = 0;
    bool input_armed = false;  // Don't accept actions until all inputs released once

    // Wait for clean input state: all buttons must be released before we
    // accept any presses. This prevents auto-launch after a flash-reboot
    // cycle (held button or PS/2 boot noise).
    printf("Waiting for input release...\n");

    while (1) {
        sleep_ms(16); // ~60 Hz poll rate

        // Read all input sources
        nespad_read();
        ps2kbd_tick();
        uint16_t kbd = ps2kbd_get_state();

        // Combine into a unified input bitmask
        uint32_t input = 0;
        if (nespad_state & DPAD_LEFT)   input |= 1;
        if (nespad_state & DPAD_RIGHT)  input |= 2;
        if (nespad_state & DPAD_A)      input |= 4;
        if (nespad_state & DPAD_START)  input |= 4;
        if (kbd & KBD_STATE_LEFT)       input |= 1;
        if (kbd & KBD_STATE_RIGHT)      input |= 2;
        if (kbd & KBD_STATE_A)          input |= 4;
        if (kbd & KBD_STATE_START)      input |= 4;

        // USB serial fallback
        int c = getchar_timeout_us(0);
        if (c == 'l' || c == 'h') input |= 1;
        if (c == 'r')             input |= 2;

        // Arm input after all buttons have been released at least once
        if (!input_armed) {
            if (input == 0) {
                input_armed = true;
                printf("Input armed (nespad=0x%lx kbd=0x%x)\n",
                       (unsigned long)nespad_state, kbd);
            } else {
                printf("Input blocked: input=0x%lx nespad=0x%lx kbd=0x%x\n",
                       (unsigned long)input, (unsigned long)nespad_state, kbd);
                prev_input = input;
                continue;
            }
        }

        // Edge detection with auto-repeat
        uint32_t pressed = input & ~prev_input; // newly pressed
        if (input && input == prev_input) {
            repeat_timer++;
            if (repeat_timer > 20 && repeat_timer % 5 == 0) // repeat after 320ms, every 80ms
                pressed = input;
        } else {
            repeat_timer = 0;
        }
        prev_input = input;

        // Handle navigation
        bool changed = false;
        if ((pressed & 1) && entry_count > 0) {
            selected = (selected - 1 + entry_count) % entry_count;
            changed = true;
        }
        if ((pressed & 2) && entry_count > 0) {
            selected = (selected + 1) % entry_count;
            changed = true;
        }

        if (changed) {
            render_entry(selected);
            printf("Selected: %s (%d/%d)\n", entries[selected].filename, selected + 1, entry_count);
        }

        // Launch selected firmware (A or Start button)
        if ((pressed & 4) && entry_count > 0) {
            printf("Launching: %s\n", entries[selected].filename);

            // Build full path
            char path[MAX_PATH];
            snprintf(path, sizeof(path), BASE_DIR "/%s", entries[selected].filename);

            // Show flashing message
            memset(framebuffer, COL_BG, sizeof(framebuffer));
            fb_text_center(SCREEN_H / 2 - 10, "Flashing firmware...", COL_WHITE);
            fb_text_center(SCREEN_H / 2 + 4, entries[selected].title, COL_TITLE);
            fb_text_center(SCREEN_H / 2 + 18, "Do not power off!", COL_GRAY);

            sleep_ms(2000);  // Show message for 2 seconds

            // Stop DVI — Core 1 will be locked out during flash
            // (multicore_lockout_start_blocking requires Core 1 to be in lockout handler)

            // Flash the UF2 (re-mounts SD, flashes, reboots)
            flash_uf2(path);
            // If we get here, flashing failed
            fb_text_center(SCREEN_H / 2 + 32, "Flash failed!", COL_WHITE);
        }
    }
}
