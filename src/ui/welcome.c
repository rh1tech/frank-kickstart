/*
 * frank-kickstart - the welcome screen: a synthwave dusk. A neon grid runs
 * towards the viewer, stars twinkle, the FRANK board hovers over the
 * horizon.
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Every smooth gradient is built from colours the video output really has
 * (video_quantize) and dithered between them, so it does not band on the
 * 15-bit HSTX or the 104-level PIO output. Per frame only the floor (rows
 * below the horizon, drawn just behind vblank, ahead of the beam) and the
 * board's rectangle are redrawn; the stars twinkle in the palette.
 *
 * Palette: 0 black, 1 horizon line, 2..5 stars; 16..47 sky; 48.. logo ramp;
 * 64.. accent ramp; 80.. muted ramp; 96..142 the board (FRANK Next render,
 * tools/mkboardart.py);
 * 144..255 floor (7 depth bands x 16 line levels).
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "catalog.h"
#include "gfx.h"
#include "input.h"
#include "palette.h"
#include "theme.h"
#include "ui.h"
#include "video.h"
#include "board_art.h"

#ifndef FK_VERSION_MAJOR
#define FK_VERSION_MAJOR 2
#define FK_VERSION_MINOR 0
#endif

#define HOR       300          /* the horizon row */
#define P_HORIZON 1
#define P_SKY     16
#define N_SKY     32
#define P_LOGO    48
#define P_ACCENT  64
#define P_MUTED   80
#define P_STARS   2            /* 4 */
#define P_BOARD   95           /* + 1..47: the board picture */
#define P_FLOOR   144
#define N_BANDS   7

/* the board stands on the horizon, its lower part over the grid; the
 * rectangle it is redrawn in has 4 px of hover each way */
#define BOARD_TOP (HOR - 150)
#define BOARD_X0  ((GFX_W - BOARD_ART_W) / 2)
#define BOARD_Y0  (BOARD_TOP - 4)

static uint8_t *board_px; /* the picture, unpacked into PSRAM */

static void unpack_board(void) {
    if (!board_px) board_px = pmem_alloc(BOARD_ART_W * BOARD_ART_H);
    if (!board_px) return;
    const uint8_t *s = board_art_rle, *end = board_art_rle + sizeof board_art_rle;
    uint8_t *d = board_px, *dend = board_px + BOARD_ART_W * BOARD_ART_H;
    while (s < end && d < dend) {
        uint8_t c = *s++;
        if (c < 128) {
            for (int n = c + 1; n-- && s < end && d < dend;) *d++ = *s++;
        } else {
            uint8_t v = *s++;
            for (int n = c - 125; n-- && d < dend;) *d++ = v;
        }
    }
}

/* the floor: z = FH / dy; grid lines every SZ in depth, every SX across */
#define FOCAL     320.0f
#define FH        (FOCAL * 1.7f)
#define SZ        1.0f
#define SX        0.55f

static const uint8_t bayer[4][4] = {
    {0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

static uint8_t *layer;

/* --- exact-colour gradients --- */

typedef struct {
    int first, n;
    float pos[32]; /* where along the gradient (0..1) each entry is centred */
} xgrad_t;

/* Fills palette[first..] with the distinct output colours along f(0..1)
 * (at most max of them), each placed at the middle of its stretch. */
static void xgrad_build(xgrad_t *g, int first, int max, uint32_t (*f)(float)) {
    g->first = first;
    g->n = 0;
    float start = 0;
    uint32_t cur = video_quantize(f(0));
    const int steps = 2048;
    /* first count the stretches, to thin them out if there are too many */
    int distinct = 1;
    for (int i = 1; i <= steps; i++) {
        uint32_t c = video_quantize(f((float)i / steps));
        if (c != cur) { distinct++; cur = c; }
    }
    int every = (distinct + max - 1) / max;
    cur = video_quantize(f(0));
    int k = 0;
    for (int i = 1; i <= steps; i++) {
        float t = (float)i / steps;
        uint32_t c = video_quantize(f(t));
        if (c != cur || i == steps) {
            if (k++ % every == 0 && g->n < max) {
                pal_set(first + g->n, cur);
                g->pos[g->n] = (start + t) * 0.5f;
                g->n++;
            }
            start = t;
            cur = c;
        }
    }
}

/* The palette index for gradient position t at pixel (x, y), dithered. */
static inline uint8_t xgrad_at(const xgrad_t *g, float t, int x, int y) {
    if (t <= g->pos[0]) return (uint8_t)g->first;
    for (int i = 0; i < g->n - 1; i++) {
        if (t < g->pos[i + 1]) {
            float fr = (t - g->pos[i]) / (g->pos[i + 1] - g->pos[i]);
            return (uint8_t)(g->first + i + ((int)(fr * 16.0f) > bayer[y & 3][x & 3]));
        }
    }
    return (uint8_t)(g->first + g->n - 1);
}

static uint32_t lerp3(float t, float t1, uint32_t a, uint32_t b, uint32_t c) {
    if (t < t1) return pal_mix(a, b, (int)(t / t1 * 256));
    return pal_mix(b, c, (int)((t - t1) / (1 - t1) * 256));
}

/* the sky, top to the horizon */
static uint32_t sky_color(float t) {
    if (t < 0.5f) return pal_mix(0x05040E, 0x150A2E, (int)(t / 0.5f * 256));
    return lerp3((t - 0.5f) / 0.5f, 0.6f, 0x150A2E, 0x3C1352, 0x7A2468);
}

static xgrad_t sky;

/* --- the parts --- */

static void build_palette(void) {
    for (int i = 0; i < 256; i++) pal_set(i, 0);
    pal_set(P_HORIZON, 0xFF8FD6);
    xgrad_build(&sky, P_SKY, N_SKY, sky_color);
    /* text ramps start from the sky where the text is */
    pal_ramp(P_LOGO, sky_color(104.0f / HOR), 0xFFFFFF);
    pal_ramp(P_ACCENT, sky_color(124.0f / HOR), RGB_ACCENT);
    pal_ramp(P_MUTED, sky_color(24.0f / HOR), 0x8E86B0);
    for (int i = 1; i <= BOARD_ART_COLORS; i++) pal_set(P_BOARD + i, board_art_palette[i]);
    /* floor: band 0 at the horizon (in the glow) .. band 6 at the viewer */
    for (int b = 0; b < N_BANDS; b++) {
        int k = b * 256 / (N_BANDS - 1);
        uint32_t base = pal_mix(0x2A0C40, 0x0A0517, k);
        uint32_t line = pal_mix(0x9A55FF, 0xFF4FD8, k);
        for (int l = 0; l < 16; l++) pal_set(P_FLOOR + b * 16 + l, pal_mix(base, line, l * 256 / 15));
    }
}

static void twinkle(int frame) {
    for (int g = 0; g < 4; g++) {
        float k = 0.55f + 0.45f * sinf(frame * 0.06f + g * 1.7f);
        pal_set(P_STARS + g, pal_mix(0x20143A, 0xFFFFFF, (int)(k * 256)));
    }
}

static void draw_static(void) {
    gfx_target(layer);
    /* sky */
    for (int y = 0; y < HOR; y++)
        for (int x = 0; x < GFX_W; x++) layer[y * GFX_W + x] = xgrad_at(&sky, (float)y / HOR, x, y);
    /* stars (a fixed pseudo-random field) */
    uint32_t seed = 0x2F6B1D3u;
    for (int i = 0; i < 170; i++) {
        seed = seed * 1664525u + 1013904223u;
        int x = (int)(seed >> 8) % GFX_W;
        seed = seed * 1664525u + 1013904223u;
        int y = 6 + (int)((seed >> 8) % 230u);
        /* fewer towards the horizon glow */
        if ((seed & 0xFF) < (uint32_t)(y * 255 / 236)) continue;
        uint8_t c = P_STARS + (uint8_t)(i & 3);
        layer[y * GFX_W + x] = c;
        if (i % 9 == 0 && x + 1 < GFX_W && y + 1 < HOR) {
            layer[y * GFX_W + x + 1] = c;
            layer[(y + 1) * GFX_W + x] = c;
            layer[(y + 1) * GFX_W + x + 1] = c;
        }
    }
    /* logo and credits */
    text_draw_center(&font_huge, GFX_W / 2, 104, "FRANK", P_LOGO);
    /* KICKSTART: heavy, letter-spaced, close under the logo */
    const char *k = "KICKSTART";
    int sp = 7, w = 0;
    for (const char *c = k; *c; c++) w += text_width(&font_kick, c, 1) + sp;
    int x = (GFX_W - (w - sp)) / 2;
    for (const char *c = k; *c; c++) {
        char ch[2] = {*c, 0};
        x = text_draw(&font_kick, x, 129, ch, P_ACCENT) + sp;
    }
    char ver[48];
    snprintf(ver, sizeof ver, "v%d.%02d  \xC2\xB7  Mikhail Matveev", FK_VERSION_MAJOR, FK_VERSION_MINOR);
    text_draw(&font_small, 24, 26, ver, P_MUTED);
    const char *url = "github.com/rh1tech/frank-kickstart";
    text_draw(&font_small, GFX_W - 24 - text_width(&font_small, url, -1), 26, url, P_MUTED);
    /* the horizon */
    memset(layer + HOR * GFX_W, P_HORIZON, GFX_W);
    gfx_target(NULL);
}

/* The board's rectangle: sky from the layer, then the board, hovering. */
/* The board's part above the grid (the scene's rows back under it, the
 * horizon line too). Drawn first after vblank: the beam is still above it. */
static void draw_board(int hover) {
    for (int y = BOARD_Y0; y <= HOR; y++)
        memcpy(video_fb + y * GFX_W + BOARD_X0, layer + y * GFX_W + BOARD_X0, BOARD_ART_W);
    if (!board_px) return;
    int by = BOARD_TOP + hover;
    for (int y = by; y <= HOR && y < by + BOARD_ART_H; y++) {
        const uint8_t *src = board_px + (y - by) * BOARD_ART_W;
        uint8_t *dst = video_fb + y * GFX_W + BOARD_X0;
        for (int x = 0; x < BOARD_ART_W; x++)
            if (src[x]) dst[x] = (uint8_t)(P_BOARD + src[x]); /* 0: see-through */
    }
}

/* The board's part over the grid, one row: draw_floor() lays it on each
 * row as soon as the row is drawn, so the beam never shows the row without
 * it (the board would look like it stood behind the horizon). */
static inline void board_on_floor_row(int y, int by) {
    if (!board_px || y < by || y >= by + BOARD_ART_H) return;
    const uint8_t *src = board_px + (y - by) * BOARD_ART_W;
    uint8_t *dst = video_fb + y * GFX_W + BOARD_X0;
    for (int x = 0; x < BOARD_ART_W; x++)
        if (src[x]) dst[x] = (uint8_t)(P_BOARD + src[x]);
}

/* How much of the pixel [d - 0.5, d + 0.5] a line [-w/2, w/2] covers: the
 * same total for every sub-pixel phase, so far lines do not shimmer. */
static inline float overlap(float d, float w) {
    float lo = d - 0.5f > -0.5f * w ? d - 0.5f : -0.5f * w;
    float hi = d + 0.5f < 0.5f * w ? d + 0.5f : 0.5f * w;
    return hi > lo ? hi - lo : 0.0f;
}

/* The floor, rows HOR+1.., grid lines anti-aliased, fading into the glow. */
static void draw_floor(float scroll, int board_y) {
    const int nrows = GFX_H - HOR - 1;
    for (int dy = 1; dy <= nrows; dy++) {
        int y = HOR + dy;
        float z = FH / dy;
        float near = (float)dy / nrows; /* 0 horizon .. 1 viewer */
        /* the line across at this depth */
        float u = (z + scroll) / SZ;
        float fr = u - floorf(u);
        float rows_per_z = dy / z;
        float dpx = (fr < 0.5f ? fr : 1.0f - fr) * SZ * rows_per_z;
        float lw = 1.0f + 1.0f * near;
        /* lines closer than a few rows blur into the glow */
        float hfade = (SZ * rows_per_z - 3.0f) / 9.0f;
        if (hfade > 1) hfade = 1;
        if (hfade < 0) hfade = 0;
        int hlev = (int)(overlap(dpx, lw) * hfade * 15.0f + 0.5f);
        /* lines along the depth: grid units per pixel, fixed point */
        float per_px = z / FOCAL / SX;
        int32_t step = (int32_t)(per_px * 65536.0f);
        int32_t pos = (int32_t)((0.5f - 320.0f) * per_px * 65536.0f) + (int32_t)(1 << 30);
        float vfade = (1.0f / per_px - 8.0f) / 18.0f;
        if (vfade > 1) vfade = 1;
        if (vfade < 0) vfade = 0;
        float vlw = 1.0f + 0.8f * near;
        /* only pixels this close (in grid units, 16.16) touch a line */
        uint32_t dthr = (uint32_t)((0.5f * vlw + 0.5f) * per_px * 65536.0f);
        float inv = 1.0f / (per_px * 65536.0f);
        float vscale = 15.0f * vfade;
        int bandpos = (int)(near * (N_BANDS - 1) * 16.0f);
        uint8_t *row = video_fb + y * GFX_W;
        const uint8_t *bay = bayer[y & 3];
        for (int x = 0; x < GFX_W; x++, pos += step) {
            int lev = hlev;
            if (vfade > 0) {
                uint32_t f = (uint32_t)pos & 0xFFFF;
                uint32_t d = f < 0x8000 ? f : 0x10000 - f;
                if (d < dthr) {
                    /* the level's fraction dithered, not rounded: rounding
                     * dim lines makes moire */
                    int vlev = ((int)(overlap((float)d * inv, vlw) * vscale * 16.0f) + bay[(x + 1) & 3]) >> 4;
                    if (vlev > lev) lev = vlev;
                }
            }
            int b = (bandpos + bay[x & 3]) >> 4;
            if (b > N_BANDS - 1) b = N_BANDS - 1;
            row[x] = (uint8_t)(P_FLOOR + b * 16 + lev);
        }
        board_on_floor_row(y, board_y);
    }
}

void ui_welcome(void) {
    if (!layer) layer = pmem_alloc(GFX_W * GFX_H);
    unpack_board();
    build_palette();
    twinkle(0);
    draw_static();

    pal_begin();
    pal_fade(0, 256, 0, 0);
    pal_end();
    video_wait_vblank();
    memcpy(video_fb, layer, GFX_W * GFX_H);

    /* no skip from a key still held at power-on */
    for (int w = 0; w < 60 && input_poll(); w++) video_wait_vblank();

    float scroll = 0, speed = 0;
    int fade = 0, hover = 0;
    for (int frame = 0; frame < 600; frame++) {
        if (input_poll() || frame > 540) break;
        if (fade < 256) fade += 8;
        if (fade > 256) fade = 256;

        /* the camera picks up speed, then cruises */
        if (speed < 0.045f) speed += 0.0012f;
        scroll -= speed;
        if (scroll < -1000.0f) scroll += 1000.0f;
        hover = (int)lroundf(3.0f * sinf(frame * 0.045f));

        video_wait_vblank();
        twinkle(frame);
        pal_begin();
        pal_fade(0, 256, 0, fade);
        pal_end();
        /* the board's upper part first, then the grid row by row with the
         * board's lower part laid on each row: both ahead of the beam */
        draw_board(hover);
        draw_floor(scroll, BOARD_TOP + hover);
    }
    /* out: a fade to black, the scene still moving */
    for (; fade > 0; fade -= 24) {
        scroll -= speed;
        video_wait_vblank();
        pal_begin();
        pal_fade(0, 256, 0, fade);
        pal_end();
        draw_board(hover);
        draw_floor(scroll, BOARD_TOP + hover);
    }
    pal_begin();
    pal_fade(0, 256, 0, 0);
    pal_end();
    video_wait_vblank();
    memset(video_fb, 0, GFX_W * GFX_H);
}
