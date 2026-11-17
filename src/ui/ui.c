/*
 * frank-kickstart - the launcher screens
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Layout (640x480):
 *   header   0..51    wordmark, board and version
 *   page     52..427  cover card (left), the entry's details (right)
 *   footer   428..479 key hints
 *
 * Screens are composed off-screen (PSRAM) and copied in. Moving between
 * entries fades the text out through its palette ramps while the cover
 * slides away and fades, swaps everything at the faded point, then slides
 * and fades the new entry in.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "catalog.h"
#include "gfx.h"
#include "input.h"
#include "music.h"
#include "palette.h"
#include "theme.h"
#include "ui.h"
#include "video.h"

#ifndef FK_VERSION_MAJOR
#define FK_VERSION_MAJOR 2
#define FK_VERSION_MINOR 0
#endif
#ifndef BOARD_NAME
#define BOARD_NAME "M2"
#endif

int ui_installed = -1;

#define HEADER_H   52
#define FOOTER_Y   428
#define COVER_X    36
#define COVER_Y    82
#define COVER_W    200
#define COVER_H    300
#define COVER_R    10
#define SHADOW_BLUR 16
#define SHADOW_DY   6
#define SLIDE_PX    16
#define INFO_X     272
#define INFO_W     (640 - INFO_X - 36)

/* the area the cover (and its shadow) may cover while it slides */
#define AREA_X0 (COVER_X - SLIDE_PX - SHADOW_BLUR)
#define AREA_X1 (COVER_X + COVER_W + SLIDE_PX + SHADOW_BLUR)
#define AREA_Y0 (COVER_Y - SHADOW_BLUR)
#define AREA_Y1 (COVER_Y + COVER_H + SHADOW_DY + SHADOW_BLUR)

#define MASK_W (COVER_W + 2 * SHADOW_BLUR)
#define MASK_H (COVER_H + 2 * SHADOW_BLUR)

static uint8_t *layer_bg;   /* the page without any entry */
static uint8_t *layer_back; /* the next screen being composed */
static uint8_t *shadow_mask;

/* ------------------------------------------------------------------ */
/* Palette                                                             */
/* ------------------------------------------------------------------ */

void theme_load_palette(void) {
    pal_set(C_BG, RGB_PAGE);
    pal_set(C_BAR, RGB_BAR);
    pal_set(C_CARD, RGB_CARD);
    pal_set(C_DIVIDER, RGB_DIVIDER);
    pal_set(C_GREEN, RGB_GREEN);
    pal_set(C_ACCENT, RGB_ACCENT);
    pal_set(C_WHITE, RGB_TEXT);
    pal_set(C_BLACK, 0);
    pal_ramp(RAMP_TEXT, RGB_PAGE, RGB_TEXT);
    pal_ramp(RAMP_MUTED, RGB_PAGE, RGB_MUTED);
    pal_ramp(RAMP_ACCENT, RGB_PAGE, RGB_ACCENT);
    pal_ramp(RAMP_SHADOW, RGB_PAGE, RGB_SHADOW);
    pal_ramp(RAMP_CARD, RGB_PAGE, RGB_CARD);
    pal_ramp(RAMP_ONCARD, RGB_CARD, RGB_TEXT);
    pal_ramp(RAMP_GREEN, RGB_PAGE, RGB_GREEN);
    /* in sRGB steps, not linear light: its dark end gives the logo chip
     * its fine gradient */
    for (int i = 0; i < 16; i++) {
        uint32_t c = 0;
        for (int sh = 0; sh <= 16; sh += 8) {
            int a = (RGB_BAR >> sh) & 255, b = (RGB_TEXT >> sh) & 255;
            c |= (uint32_t)(a + (b - a) * i / 15) << sh;
        }
        pal_set(RAMP_CHROME + i, c);
    }
    /* no glow until an entry gives it a colour (message screens) */
    for (int i = 0; i < N_GLOW; i++) pal_set(C_GLOW + i, RGB_PAGE);
}

/* The glow levels, from the page colour towards the tint. The cover's
 * shadow lies where the glow is at about level 2: its ramp starts there, or
 * its faint edge shows as a dark frame. */
static void load_glow_palette(uint32_t tint) {
    for (int i = 0; i < N_GLOW; i++)
        pal_set(C_GLOW + i, pal_mix(RGB_PAGE, tint, GLOW_MAX * i / (N_GLOW - 1)));
    pal_ramp(RAMP_SHADOW, pal_master[C_GLOW + 2], RGB_SHADOW);
}

static void load_art_palette(const art_t *a) {
    for (int c = 0; c < ART_COLORS; c++) pal_set(ART_BASE + c, a && c < a->ncolors ? a->pal[c] : RGB_CARD);
    load_glow_palette(a ? a->tint : RGB_ACCENT);
}

/* The palette with the entry's text and cover faded: k = 256 fully shown,
 * 0 gone; `all` = 256..0 the whole screen towards black on top. */
static void show_faded(int k, int all) {
    pal_begin();
    pal_fade(RAMP_TEXT, 16, RGB_PAGE, k);
    pal_fade(RAMP_MUTED, 16, RGB_PAGE, k);
    pal_fade(RAMP_ACCENT, 16, RGB_PAGE, k);
    pal_fade(RAMP_GREEN, 16, RGB_PAGE, k);
    pal_fade(RAMP_SHADOW, 16, RGB_PAGE, k);
    pal_fade(C_GLOW, N_GLOW, RGB_PAGE, k);
    pal_fade(ART_BASE, ART_COLORS, RGB_PAGE, k);
    if (all < 256) pal_fade(0, 256, 0, all);
    pal_end();
}

/* ------------------------------------------------------------------ */
/* Pieces                                                              */
/* ------------------------------------------------------------------ */

/* The ambient glow: a soft bloom from behind the cover, wider than tall,
 * fading out before the text column (whose ramps assume the page colour).
 * Its colour is the palette's (load_glow_palette); here only the shape,
 * dithered over N_GLOW levels. */
#define GLOW_RX 250.0f
#define GLOW_RY 230.0f

static void draw_glow(void) {
    static const uint8_t bayer[4][4] = {
        {0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
    const float cx = COVER_X + COVER_W * 0.5f, cy = COVER_Y + COVER_H * 0.5f;
    uint8_t *fb = gfx_target_buf();
    for (int y = HEADER_H; y < FOOTER_Y; y++)
        for (int x = 0; x < INFO_X - 8; x++) {
            float dx = (x - cx) / GLOW_RX, dy = (y - cy) / GLOW_RY;
            /* to the right it has to be gone by the text column */
            if (x > cx) dx = (x - cx) / (INFO_X - 8 - cx);
            float t = sqrtf(dx * dx + dy * dy);
            if (t >= 1.0f) continue;
            float g = (1.0f - t) * (1.0f - t);
            int lv = ((int)(g * (N_GLOW - 1) * 16.0f) + bayer[y & 3][x & 3]) >> 4;
            if (lv > N_GLOW - 1) lv = N_GLOW - 1;
            if (lv > 0) fb[y * GFX_W + x] = (uint8_t)(C_GLOW + lv);
        }
}

static void draw_page_bg(void) {
    gfx_fill(0, 0, GFX_W, HEADER_H, C_BAR);
    gfx_hline(0, HEADER_H - 1, GFX_W, C_DIVIDER);
    /* flat: every anti-aliasing ramp starts from this colour, and a dark
     * gradient only bands on 15-bit (HSTX) and 104-level (PIO) output */
    gfx_fill(0, HEADER_H, GFX_W, FOOTER_Y - HEADER_H, C_BG);
    gfx_fill(0, FOOTER_Y, GFX_W, GFX_H - FOOTER_Y, C_BAR);
    gfx_hline(0, FOOTER_Y, GFX_W, C_DIVIDER);
    draw_glow();
}

/* A key cap with a label, right-aligned at x1 if right; returns its width. */
static int keycap(int x, int y, const char *label, bool right) {
    int w = text_width(&font_bold, label, -1) + 18;
    if (w < 28) w = 28;
    if (right) x -= w;
    gfx_round_rect(x, y, w, 26, 7, RAMP_CARD);
    text_draw(&font_bold, x + (w - text_width(&font_bold, label, -1)) / 2, y + 18, label, RAMP_ONCARD);
    return w;
}

/* A pill outline with text in one ramp. */
static int pill(int x, int y, const char *label, uint8_t ramp) {
    int w = text_width(&font_small, label, -1) + 16;
    gfx_round_frame(x, y, w, 20, 10, ramp);
    text_draw(&font_small, x + 8, y + 14, label, ramp);
    return w;
}

static void draw_header(void) {
    /* the mark: the board's RP2350, as the real one looks: a black QFN
     * package with silver pads along its edges and the raspberry etched in
     * grey (centred on 40,26, where the welcome screen's board lands) */
    {
        static const char *const berry[10] = {
            "..#...#..",
            "...#.#...",
            "..#####..",
            ".#.#.#.#.",
            "#########",
            ".#.#.#.#.",
            "#########",
            ".#.#.#.#.",
            "..#####..",
            "...###...",
        };
        const int x0 = 28, y0 = 14, n = 24;
        /* the body: a soft gradient, dark grey at the top to black */
        for (int j = 0; j < n; j++) {
            int lv = 4 - j * 5 / n; /* 4 .. 0, then black */
            uint8_t c = lv > 0 ? (uint8_t)(RAMP_CHROME + lv) : C_BLACK;
            int inset = (j == 0 || j == n - 1);
            gfx_fill(x0 + inset, y0 + j, n - 2 * inset, 1, c);
        }
        /* a faint rim, so black reads on the dark header */
        gfx_hline(x0 + 1, y0, n - 2, C_DIVIDER);
        gfx_hline(x0 + 1, y0 + n - 1, n - 2, C_DIVIDER);
        gfx_fill(x0, y0 + 1, 1, n - 2, C_DIVIDER);
        gfx_fill(x0 + n - 1, y0 + 1, 1, n - 2, C_DIVIDER);
        /* QFN pads: five a side, at the edge */
        for (int k = 0; k < 5; k++) {
            int o = 4 + k * 4;
            gfx_fill(x0 + o, y0, 2, 2, RAMP_CHROME + 5);
            gfx_fill(x0 + o, y0 + n - 2, 2, 2, RAMP_CHROME + 5);
            gfx_fill(x0, y0 + o, 2, 2, RAMP_CHROME + 5);
            gfx_fill(x0 + n - 2, y0 + o, 2, 2, RAMP_CHROME + 5);
        }
        /* the etched raspberry */
        for (int j = 0; j < 10; j++)
            for (int i = 0; i < 9; i++)
                if (berry[j][i] == '#') gfx_fill(x0 + 8 + i, y0 + 7 + j, 1, 1, RAMP_CHROME + 10);
        /* the pin-1 dot */
        gfx_fill(x0 + 4, y0 + 4, 2, 2, RAMP_CHROME + 6);
    }
    /* the header's own ramp: it stays while entries fade */
    int x = text_draw(&font_bold, 62, 32, "FRANK", RAMP_CHROME);
    text_level_max(9);
    text_draw(&font_bold, x + 6, 32, "KICKSTART", RAMP_CHROME);
    char right[48];
    snprintf(right, sizeof right, "Board %s  \xC2\xB7  v%d.%02d", BOARD_NAME, FK_VERSION_MAJOR, FK_VERSION_MINOR);
    text_draw(&font_small, GFX_W - 28 - text_width(&font_small, right, -1), 31, right, RAMP_CHROME);
    text_level_max(15);
}

/* the position bar under the header */
static void draw_position(int sel) {
    if (cat_count <= 1) return;
    int w = GFX_W / cat_count;
    if (w < 12) w = 12;
    int x = (GFX_W - w) * sel / (cat_count - 1);
    gfx_fill(x, HEADER_H - 2, w, 2, C_ACCENT);
}

static void draw_footer(int sel, bool confirm) {
    int y = FOOTER_Y + 13;
    int x = 28;
    x += keycap(x, y, "\xE2\x97\x80", false) + 4;  /* U+25C0 */
    x += keycap(x, y, "\xE2\x96\xB6", false) + 10; /* U+25B6 */
    text_level_max(9);
    x = text_draw(&font_small, x, y + 17, "Browse", RAMP_CHROME) + 24;
    if (music_available()) {
        x += keycap(x, y, "M", false) + 10;
        text_draw(&font_small, x, y + 17, music_is_on() ? "Mute" : "Music", RAMP_CHROME);
    }
    text_level_max(15);

    const char *action;
    uint8_t ramp = RAMP_MUTED;
    if (cat_count == 0) return;
    if (cat[sel].status != UF2_OK) action = "Cannot be installed";
    else if (confirm) { action = "Press again to install anyway"; ramp = RAMP_ACCENT; }
    else if (sel == ui_installed) action = "Launch";
    else action = "Install and launch";
    int rx = GFX_W - 28;
    int aw = text_width(&font_small, action, -1);
    text_draw(&font_small, rx - aw, y + 17, action, ramp);
    rx -= aw + 10;
    rx -= keycap(rx, y, "Enter", true) + 4;
    keycap(rx, y, "A", true);
}

/* A cover with a transparent colour: a cut-out on the page, centred in the
 * cover's box (not scaled), its shadow from its own shape. */
#define CUTOUT_SHADOW_DY 10

static void cutout_origin(const art_t *a, int x, int y, int *ox, int *oy) {
    *ox = x + (COVER_W - a->w) / 2;
    *oy = y + (COVER_H - a->h) / 2;
}

static void draw_cover_at(int i, int x, int y) {
    const art_t *a = cat_count ? cat_art(i) : NULL;
    if (a && a->key >= 0) {
        int ox, oy;
        cutout_origin(a, x, y, &ox, &oy);
        gfx_blit(ox, oy, a->px, a->w, a->h, a->w, ART_BASE, 1, 0, a->key);
        return;
    }
    if (a) {
        /* 2x for the old 100x150 covers; centred in the card, cropped */
        int scale = (a->w * 2 <= COVER_W + 40 && a->h * 2 <= COVER_H + 40) ? 2 : 1;
        int dw = a->w * scale, dh = a->h * scale;
        gfx_clip(x, y, x + COVER_W, y + COVER_H);
        if (dw < COVER_W || dh < COVER_H) gfx_round_rect(x, y, COVER_W, COVER_H, COVER_R, RAMP_CARD);
        int ox = x + (COVER_W - dw) / 2, oy = y + (COVER_H - dh) / 2;
        int r = (dw >= COVER_W && dh >= COVER_H) ? COVER_R : 0;
        if (r) {
            /* the rounded corners belong to the card, not the image */
            gfx_noclip();
            gfx_clip(x, y, x + COVER_W, y + COVER_H);
            const uint8_t *src = a->px;
            int sx = ox < x ? (x - ox + scale - 1) / scale : 0;
            int sy = oy < y ? (y - oy + scale - 1) / scale : 0;
            int w = (COVER_W + scale - 1) / scale, h = (COVER_H + scale - 1) / scale;
            if (w > a->w - sx) w = a->w - sx;
            if (h > a->h - sy) h = a->h - sy;
            gfx_blit(ox + sx * scale, oy + sy * scale, src + sy * a->w + sx, w, h, a->w, ART_BASE, scale, COVER_R, -1);
        } else {
            gfx_blit(ox, oy, a->px, a->w, a->h, a->w, ART_BASE, scale, 0, -1);
        }
        gfx_noclip();
        return;
    }
    /* no cover: a card with the title's initial */
    gfx_round_rect(x, y, COVER_W, COVER_H, COVER_R, RAMP_CARD);
    if (!cat_count) return;
    char init[5] = {0};
    const char *t = cat[i].title;
    int n = 1;
    if ((uint8_t)t[0] >= 0xF0) n = 4; else if ((uint8_t)t[0] >= 0xE0) n = 3; else if ((uint8_t)t[0] >= 0xC0) n = 2;
    memcpy(init, t, (size_t)n);
    text_draw_center(&font_title, x + COVER_W / 2, y + COVER_H / 2 + 10, init, RAMP_ONCARD);
}

static void draw_shadow_at(int i, int x, int y) {
    const art_t *a = cat_count && i >= 0 ? cat_art(i) : NULL;
    if (a && a->key >= 0) {
        int ox, oy;
        cutout_origin(a, x, y, &ox, &oy);
        gfx_shadow_map(ox - 2 * ART_SHADOW_R, oy - 2 * ART_SHADOW_R + CUTOUT_SHADOW_DY, a->shadow,
                       a->sw, a->sh, RAMP_SHADOW);
        return;
    }
    uint8_t *fb = gfx_target_buf();
    int mx = x - SHADOW_BLUR, my = y - SHADOW_BLUR + SHADOW_DY;
    for (int j = 0; j < MASK_H; j++) {
        int yy = my + j;
        if (yy < 0 || yy >= GFX_H) continue;
        const uint8_t *m = shadow_mask + j * MASK_W;
        uint8_t *d = fb + yy * GFX_W + mx;
        for (int i = 0; i < MASK_W; i++)
            if (m[i]) d[i] = RAMP_SHADOW + m[i];
    }
}

static int info_line(int y, const char *label, const char *text, int lines) {
    if (!text[0]) return y;
    text_draw(&font_small, INFO_X, y, label, RAMP_ACCENT);
    y += 20;
    int n = text_draw_wrap(&font_body, INFO_X, y, INFO_W, 20, lines, text, RAMP_MUTED, true);
    return y + n * 20 + 8;
}

static void draw_info(int sel) {
    if (!cat_count) return;
    const cat_entry_t *e = &cat[sel];
    char buf[96];
    snprintf(buf, sizeof buf, "FIRMWARE %d / %d", sel + 1, cat_count);
    text_draw(&font_small, INFO_X, 100, buf, RAMP_ACCENT);

    int y = 138;
    int lines = text_draw_wrap(&font_title, INFO_X, y, INFO_W, 36, 2, e->title, RAMP_TEXT, true);
    y += (lines - 1) * 36 + 26;

    /* version · author */
    buf[0] = 0;
    if (e->version[0]) snprintf(buf, sizeof buf, "%s%s", e->version[0] == 'v' || e->version[0] == 'V' ? "" : "v", e->version);
    if (e->author[0]) {
        size_t l = strlen(buf);
        snprintf(buf + l, sizeof buf - l, "%s%s", l ? "  \xC2\xB7  " : "", e->author);
    }
    if (buf[0]) {
        text_draw_fit(&font_bold, INFO_X, y, INFO_W, buf, RAMP_MUTED);
        y += 14;
    }

    /* status pills */
    y += 8;
    int x = INFO_X;
    if (sel == ui_installed) x += pill(x, y, "\xE2\x9C\x93 INSTALLED", RAMP_GREEN) + 8; /* U+2713 */
    switch (e->status) {
    case UF2_UNREADABLE: x += pill(x, y, "NOT A UF2 FILE", RAMP_ACCENT) + 8; break;
    case UF2_RP2040: x += pill(x, y, "RP2040 IMAGE", RAMP_ACCENT) + 8; break;
    case UF2_RISCV: x += pill(x, y, "RISC-V IMAGE", RAMP_ACCENT) + 8; break;
    case UF2_TOO_BIG: x += pill(x, y, "TOO LARGE", RAMP_ACCENT) + 8; break;
    default: break;
    }
    if (e->boards) {
        static const char *names[3] = {"M1", "M2", "Z0"};
        for (int b = 0; b < 3; b++)
            if (e->boards & (1 << b))
                x += pill(x, y, names[b], (1 << b) == cat_board_bit() ? RAMP_GREEN : RAMP_MUTED) + 6;
        if (cat_wrong_board(sel)) {
            snprintf(buf, sizeof buf, "NOT FOR %s", BOARD_NAME);
            x += pill(x, y, buf, RAMP_ACCENT) + 6;
        }
    }
    if (x != INFO_X) y += 40; else y += 12;

    /* description: as many lines as leave room for the rest */
    int rest = (e->controls[0] ? 2 * 20 + 28 : 0) + (e->website[0] ? 22 : 0);
    int max_lines = (FOOTER_Y - 30 - rest - y) / 21;
    if (max_lines < 1) max_lines = 1;
    if (e->description[0]) {
        y += 4;
        int n = text_draw_wrap(&font_body, INFO_X, y, INFO_W, 21, max_lines, e->description, RAMP_TEXT, true);
        y += n * 21 + 10;
    }
    y = info_line(y + 6, "CONTROLS", e->controls, 2);
    if (e->website[0]) text_draw_fit(&font_small, INFO_X, FOOTER_Y - 22, INFO_W, e->website, RAMP_ACCENT);
    (void)y;
}

/* ------------------------------------------------------------------ */
/* Browser                                                             */
/* ------------------------------------------------------------------ */

static int shown = -1;
static bool shown_confirm;

static void compose(int sel, bool confirm) {
    cat_check(sel); /* its status: the pills and the footer show it */
    gfx_target(layer_back);
    memcpy(layer_back, layer_bg, GFX_W * GFX_H);
    draw_position(sel);
    draw_info(sel);
    draw_footer(sel, confirm);
    draw_shadow_at(sel, COVER_X, COVER_Y);
    draw_cover_at(sel, COVER_X, COVER_Y);
    gfx_target(NULL);
}

/* The back layer into the framebuffer, all of it, or all but the cover area. */
static void present(bool skip_cover) {
    for (int y = 0; y < GFX_H; y++) {
        uint8_t *d = video_fb + y * GFX_W;
        const uint8_t *s = layer_back + y * GFX_W;
        if (skip_cover && y >= AREA_Y0 && y < AREA_Y1) {
            memcpy(d + AREA_X1, s + AREA_X1, GFX_W - AREA_X1);
            if (AREA_X0 > 0) memcpy(d, s, AREA_X0);
        } else {
            memcpy(d, s, GFX_W);
        }
    }
}

/* The next frame of the cover area, the cover at x offset dx: drawn
 * off-screen (the back layer is free once presented), then copied in at
 * the vblank, top down and ahead of the beam (drawn on screen, the beam
 * caught the area empty: the cover flickered). */
static void cover_frame(int sel, int dx) {
    const int w = AREA_X1 - AREA_X0;
    for (int y = AREA_Y0; y < AREA_Y1; y++)
        memcpy(layer_back + y * GFX_W + AREA_X0, layer_bg + y * GFX_W + AREA_X0, w);
    gfx_target(layer_back);
    gfx_clip(AREA_X0, AREA_Y0, AREA_X1, AREA_Y1);
    draw_shadow_at(sel, COVER_X + dx, COVER_Y);
    draw_cover_at(sel, COVER_X + dx, COVER_Y);
    gfx_noclip();
    gfx_target(NULL);
    video_wait_vblank();
    for (int y = AREA_Y0; y < AREA_Y1; y++)
        memcpy(video_fb + y * GFX_W + AREA_X0, layer_back + y * GFX_W + AREA_X0, w);
}

void ui_init(void) {
    layer_bg = pmem_alloc(GFX_W * GFX_H);
    layer_back = pmem_alloc(GFX_W * GFX_H);
    shadow_mask = pmem_alloc(MASK_W * MASK_H);
    /* the shadow's levels, drawn once into the back layer and kept */
    gfx_target(layer_back);
    gfx_fill(0, 0, MASK_W, MASK_H, RAMP_SHADOW);
    gfx_shadow(SHADOW_BLUR, SHADOW_BLUR, COVER_W, COVER_H, COVER_R, SHADOW_BLUR, 11, RAMP_SHADOW);
    for (int j = 0; j < MASK_H; j++)
        for (int i = 0; i < MASK_W; i++) shadow_mask[j * MASK_W + i] = layer_back[j * GFX_W + i] - RAMP_SHADOW;
    gfx_target(layer_bg);
    draw_page_bg();
    draw_header();
    gfx_target(NULL);
    theme_load_palette();
}

static void fade_in_all(void) {
    for (int f = 1; f <= 12; f++) {
        show_faded(256, f * 256 / 12);
        video_wait_vblank();
    }
}

void ui_fade_out(void) {
    for (int f = 11; f >= 0; f--) {
        show_faded(256, f * 256 / 12);
        video_wait_vblank();
    }
    video_wait_vblank();
}

void ui_browser_show(int sel) {
    theme_load_palette();
    if (cat_count) load_art_palette(cat_art(sel));
    compose(sel, false);
    show_faded(256, 0);
    video_wait_vblank();
    video_wait_vblank();
    present(false);
    fade_in_all();
    shown = sel;
    shown_confirm = false;
}

/* ease-out cubic, t = 0..n */
static int ease(int t, int n, int dist) {
    float u = 1.0f - (float)t / (float)n;
    return (int)((1.0f - u * u * u) * dist + 0.5f);
}

void ui_browser_move(int to, int dir) {
    const int N = 7;
    int from = shown;
    /* out: text and cover fade, the cover slides away */
    for (int f = 1; f <= N; f++) {
        cover_frame(from, -dir * ease(f, N, SLIDE_PX));
        show_faded(256 - f * 256 / N, 256);
    }
    /* swap while nothing of either entry shows */
    compose(to, false);
    video_wait_vblank();
    present(true);
    load_art_palette(cat_art(to));
    show_faded(0, 256);
    /* in: from the other side */
    for (int f = 1; f <= N; f++) {
        cover_frame(to, dir * (SLIDE_PX - ease(f, N, SLIDE_PX)));
        show_faded(f * 256 / N, 256);
    }
    cover_frame(to, 0);
    shown = to;
    shown_confirm = false;
}

void ui_browser_confirm(int sel, bool confirm) {
    if (confirm == shown_confirm) return;
    ui_browser_footer(sel, confirm);
}

void ui_browser_footer(int sel, bool confirm) {
    shown_confirm = confirm;
    compose(sel, confirm);
    /* only the footer changes */
    video_wait_vblank();
    memcpy(video_fb + FOOTER_Y * GFX_W, layer_back + FOOTER_Y * GFX_W, (GFX_H - FOOTER_Y) * GFX_W);
}

/* ------------------------------------------------------------------ */
/* Messages and the install screen                                     */
/* ------------------------------------------------------------------ */

static void message_screen(const char *title, const char *line1, const char *line2) {
    gfx_target(layer_back);
    memcpy(layer_back, layer_bg, GFX_W * GFX_H);
    int cy = 230;
    text_draw_center(&font_title, GFX_W / 2, cy, title, RAMP_TEXT);
    if (line1) text_draw_center(&font_body, GFX_W / 2, cy + 40, line1, RAMP_MUTED);
    if (line2) text_draw_center(&font_body, GFX_W / 2, cy + 64, line2, RAMP_MUTED);
    gfx_target(NULL);
}

void ui_message(const char *title, const char *line1, const char *line2) {
    theme_load_palette();
    if (!layer_back) {
        /* no PSRAM: straight into the framebuffer */
        show_faded(256, 0);
        video_wait_vblank();
        gfx_target(NULL);
        draw_page_bg();
        draw_header();
        text_draw_center(&font_title, GFX_W / 2, 230, title, RAMP_TEXT);
        if (line1) text_draw_center(&font_body, GFX_W / 2, 270, line1, RAMP_MUTED);
        if (line2) text_draw_center(&font_body, GFX_W / 2, 294, line2, RAMP_MUTED);
        fade_in_all();
        return;
    }
    message_screen(title, line1, line2);
    show_faded(256, 0);
    video_wait_vblank();
    present(false);
    fade_in_all();
    shown = -1;
}

#define BAR_X 120
#define BAR_Y 300
#define BAR_W 400
#define BAR_H 10

void ui_flash_begin(int sel, bool reboot_only) {
    theme_load_palette();
    gfx_target(layer_back);
    memcpy(layer_back, layer_bg, GFX_W * GFX_H);
    const cat_entry_t *e = &cat[sel];
    text_draw_center(&font_small, GFX_W / 2, 176, reboot_only ? "STARTING" : "INSTALLING", RAMP_ACCENT);
    char t[80];
    snprintf(t, sizeof t, "%s", e->title);
    text_draw_center(&font_title, GFX_W / 2, 222, t, RAMP_TEXT);
    if (!reboot_only) {
        text_draw_center(&font_body, GFX_W / 2, 258, "Writing the firmware to flash. Do not power off the board.", RAMP_MUTED);
        gfx_round_rect(BAR_X, BAR_Y, BAR_W, BAR_H, BAR_H / 2, RAMP_CARD);
    }
    gfx_target(NULL);
    ui_fade_out();
    present(false);
    fade_in_all();
    shown = -1;
}

void ui_flash_progress(uint32_t done, uint32_t total) {
    if (!total) return;
    int w = (int)((uint64_t)BAR_W * done / total);
    if (w < BAR_H) w = BAR_H;
    gfx_round_rect(BAR_X, BAR_Y, w, BAR_H, BAR_H / 2, RAMP_ACCENT);
    /* the percentage under the bar, over a fresh piece of the page */
    const int ty = BAR_Y + 22, th = 24;
    for (int y = ty; y < ty + th; y++)
        memcpy(video_fb + y * GFX_W + BAR_X, layer_bg + y * GFX_W + BAR_X, BAR_W);
    char pct[16];
    snprintf(pct, sizeof pct, "%lu%%", (unsigned long)((uint64_t)100 * done / total));
    text_draw_center(&font_bold, GFX_W / 2, ty + 17, pct, RAMP_MUTED);
}

/* ------------------------------------------------------------------ */
/* Start countdown                                                     */
/* ------------------------------------------------------------------ */

#define RING_CX    (INFO_X + 72)
#define RING_CY    300
#define RING_RO    70.0f
#define RING_RI    61.0f
#define RAMP_DIGIT RAMP_GREEN /* not used on this screen: the digit's fades */
#define DIGIT_BOX  84         /* cleared for the digit: inside the ring */

/* The ring: accent from the top clockwise to frac of the turn, the rest a
 * dim track of the same ramp, so every edge (across the band and at both
 * ends of the arc) blends through one ramp. */
#define RING_TRACK 2 /* the track's level in RAMP_ACCENT */

static void draw_ring(float frac) {
    const float two_pi = 6.2831853f, lim = frac * two_pi;
    const float lo = (RING_RI - 1.5f) * (RING_RI - 1.5f), hi = (RING_RO + 1.5f) * (RING_RO + 1.5f);
    const int r = (int)RING_RO + 2;
    uint8_t *fb = gfx_target_buf();
    for (int y = -r; y <= r; y++) {
        uint8_t *row = fb + (RING_CY + y) * GFX_W + RING_CX;
        for (int x = -r; x <= r; x++) {
            float d2 = (float)(x * x + y * y);
            if (d2 < lo || d2 > hi) continue;
            float d = sqrtf(d2);
            float c = fminf(d - RING_RI, RING_RO - d) + 0.5f;
            if (c <= 0.0f) {
                row[x] = C_BG;
                continue;
            }
            if (c > 1.0f) c = 1.0f;
            /* how much of the pixel is on the arc: its distance in pixels
             * from the nearer end, clockwise from the top */
            float a = atan2f((float)x, (float)-y);
            if (a < 0.0f) a += two_pi;
            float on = fminf(a, lim - a) * d + 0.5f;
            if (a > lim) on = fmaxf(on, (a - two_pi) * d + 0.5f); /* just left of the top */
            if (on < 0.0f) on = 0.0f;
            if (on > 1.0f) on = 1.0f;
            if (lim <= 0.0f) on = 0.0f;
            float lv = c * (RING_TRACK + (15 - RING_TRACK) * on);
            row[x] = (uint8_t)(RAMP_ACCENT + (int)(lv + 0.5f));
        }
    }
}

/* The digit n in the ring, dy pixels low, at brightness k (0..256). */
static void draw_digit(int n, int dy, int k) {
    pal_ramp(RAMP_DIGIT, RGB_PAGE, pal_mix(RGB_PAGE, RGB_TEXT, k));
    pal_show();
    const int x0 = RING_CX - DIGIT_BOX / 2, y0 = RING_CY - DIGIT_BOX / 2;
    gfx_fill(x0, y0, DIGIT_BOX, DIGIT_BOX, C_BG);
    if (n < 1 || k <= 0) return;
    char s[2] = {(char)('0' + n), 0};
    gfx_clip(x0, y0, x0 + DIGIT_BOX, y0 + DIGIT_BOX);
    text_draw_center(&font_count, RING_CX, RING_CY + 30 + dy, s, RAMP_DIGIT);
    gfx_noclip();
}

bool ui_countdown(int sel, int seconds) {
    const cat_entry_t *e = sel >= 0 ? &cat[sel] : NULL;
    theme_load_palette();
    load_art_palette(e ? cat_art(sel) : NULL);
    pal_ramp(RAMP_DIGIT, RGB_PAGE, RGB_PAGE);

    gfx_target(layer_back);
    memcpy(layer_back, layer_bg, GFX_W * GFX_H);
    if (e) {
        draw_shadow_at(sel, COVER_X, COVER_Y);
        draw_cover_at(sel, COVER_X, COVER_Y);
    } else {
        draw_shadow_at(-1, COVER_X, COVER_Y);
        gfx_round_rect(COVER_X, COVER_Y, COVER_W, COVER_H, COVER_R, RAMP_CARD);
        text_draw_center(&font_title, COVER_X + COVER_W / 2, COVER_Y + COVER_H / 2 + 10, "?", RAMP_ONCARD);
    }
    text_draw(&font_small, INFO_X, 100, "LOADING", RAMP_ACCENT);
    int lines = text_draw_wrap(&font_title, INFO_X, 138, INFO_W, 36, 2, e ? e->title : "Installed firmware",
                               RAMP_TEXT, true);
    if (e && e->version[0]) {
        char buf[48];
        snprintf(buf, sizeof buf, "%s%s", e->version[0] == 'v' || e->version[0] == 'V' ? "" : "v", e->version);
        text_draw(&font_bold, INFO_X, 138 + (lines - 1) * 36 + 26, buf, RAMP_MUTED);
    }
    text_draw(&font_bold, RING_CX + 96, RING_CY - 4, "Press any key", RAMP_TEXT);
    text_draw(&font_body, RING_CX + 96, RING_CY + 18, "to open the menu", RAMP_MUTED);
    {
        int y = FOOTER_Y + 13, x = 28;
        x += keycap(x, y, "Any key", false) + 10;
        text_level_max(9);
        text_draw(&font_small, x, y + 17, "Menu", RAMP_CHROME);
        text_level_max(15);
    }
    draw_ring(1.0f);
    gfx_target(NULL);

    show_faded(256, 0);
    video_wait_vblank();
    video_wait_vblank();
    present(false);
    shown = -1;
    fade_in_all();

    /* each second: the digit rises in and fades up, then out and up again
     * near its end; the ring runs down all the while */
    const int per = 60, total = seconds * per;
    bool key = false;
    uint32_t held = input_poll();
    for (int f = 0; f <= total && !key; f++) {
        video_wait_vblank();
        /* a key pressed now: the NES pad's first read after a reset has
         * every button down, so whatever is held from the start waits
         * until it is let go */
        uint32_t in = input_poll();
        if (in & ~held) key = true;
        held &= in;
        int n = seconds - f / per, p = f % per, dy = 0, k = 256;
        if (f == total) {
            n = 0;
        } else if (p < 14) {
            dy = 14 - ease(p, 14, 14);
            k = ease(p, 14, 256);
        } else if (p >= per - 10) {
            int t = p - (per - 10);
            dy = -t;
            k = 256 - t * 256 / 10;
        }
        draw_digit(n, dy, k);
        draw_ring(1.0f - (float)f / (float)total);
    }
    ui_fade_out();
    return key;
}
