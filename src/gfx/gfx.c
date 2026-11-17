/*
 * frank-kickstart - drawing on the 8-bit framebuffer
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <math.h>
#include <string.h>
#include "gfx.h"

static int cx0 = 0, cy0 = 0, cx1 = GFX_W, cy1 = GFX_H;
static uint8_t *gfx_buf = video_fb;

void gfx_target(uint8_t *buf) {
    gfx_buf = buf ? buf : video_fb;
}

uint8_t *gfx_target_buf(void) {
    return gfx_buf;
}

void gfx_clip(int x0, int y0, int x1, int y1) {
    cx0 = x0 < 0 ? 0 : x0;
    cy0 = y0 < 0 ? 0 : y0;
    cx1 = x1 > GFX_W ? GFX_W : x1;
    cy1 = y1 > GFX_H ? GFX_H : y1;
}

void gfx_noclip(void) {
    cx0 = 0; cy0 = 0; cx1 = GFX_W; cy1 = GFX_H;
}

static inline bool inside(int x, int y) {
    return x >= cx0 && x < cx1 && y >= cy0 && y < cy1;
}

static inline void put(int x, int y, uint8_t c) {
    if (inside(x, y)) gfx_buf[y * GFX_W + x] = c;
}

/* Coverage a (1..15) of the ramp's colour; where the pixel already holds
 * more of the same ramp (overlapping glyphs, shapes), it stays. */
static inline void blend(int x, int y, uint8_t ramp, int a) {
    if (a <= 0 || !inside(x, y)) return;
    if (a > 15) a = 15;
    uint8_t *p = &gfx_buf[y * GFX_W + x];
    if (*p >= ramp && *p < ramp + 16 && *p - ramp >= a) return;
    *p = ramp + a;
}

void gfx_fill(int x, int y, int w, int h, uint8_t c) {
    int x1 = x + w, y1 = y + h;
    if (x < cx0) x = cx0;
    if (y < cy0) y = cy0;
    if (x1 > cx1) x1 = cx1;
    if (y1 > cy1) y1 = cy1;
    if (x >= x1) return;
    for (int j = y; j < y1; j++) memset(&gfx_buf[j * GFX_W + x], c, x1 - x);
}

void gfx_hline(int x, int y, int w, uint8_t c) {
    gfx_fill(x, y, w, 1, c);
}

static const uint8_t bayer4[4][4] = {
    {0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

void gfx_vgradient(int x, int y, int w, int h, uint8_t first, int n) {
    if (h <= 1 || n <= 1) {
        gfx_fill(x, y, w, h, first);
        return;
    }
    for (int j = 0; j < h; j++) {
        int yy = y + j;
        if (yy < cy0 || yy >= cy1) continue;
        int p = j * (n - 1) * 16 / (h - 1);
        int i = p >> 4, frac = p & 15;
        for (int xx = x; xx < x + w; xx++) {
            if (xx < cx0 || xx >= cx1) continue;
            int c = i + (frac > bayer4[yy & 3][xx & 3]);
            if (c > n - 1) c = n - 1;
            gfx_buf[yy * GFX_W + xx] = first + c;
        }
    }
}

/* Coverage (0..15) of the pixel at (px,py) by a disc of radius r centred
 * on (ccx,ccy), all in pixel-centre coordinates. */
static inline int disc_cov(float px, float py, float ccx, float ccy, float r) {
    float dx = px - ccx, dy = py - ccy;
    float d = sqrtf(dx * dx + dy * dy);
    float c = r + 0.5f - d;
    if (c <= 0.0f) return 0;
    if (c >= 1.0f) return 15;
    return (int)(c * 15.0f + 0.5f);
}

void gfx_round_rect(int x, int y, int w, int h, int r, uint8_t ramp) {
    if (r * 2 > w) r = w / 2;
    if (r * 2 > h) r = h / 2;
    uint8_t c = ramp + 15;
    for (int j = 0; j < h; j++) {
        int yy = y + j;
        bool top = j < r, bot = j >= h - r;
        if (!top && !bot) {
            gfx_fill(x, yy, w, 1, c);
            continue;
        }
        float ccy = top ? (float)(y + r) - 0.5f : (float)(y + h - r) - 0.5f;
        /* corner columns get coverage, the middle is solid */
        for (int i = 0; i < r; i++) {
            int a = disc_cov((float)(x + i), (float)yy, (float)(x + r) - 0.5f, ccy, (float)r - 0.5f);
            if (a >= 15) put(x + i, yy, c); else blend(x + i, yy, ramp, a);
            int xr = x + w - 1 - i;
            a = disc_cov((float)xr, (float)yy, (float)(x + w - r) - 0.5f, ccy, (float)r - 0.5f);
            if (a >= 15) put(xr, yy, c); else blend(xr, yy, ramp, a);
        }
        gfx_fill(x + r, yy, w - 2 * r, 1, c);
    }
}

void gfx_round_frame(int x, int y, int w, int h, int r, uint8_t ramp) {
    if (r * 2 > w) r = w / 2;
    if (r * 2 > h) r = h / 2;
    uint8_t c = ramp + 15;
    gfx_fill(x + r, y, w - 2 * r, 1, c);
    gfx_fill(x + r, y + h - 1, w - 2 * r, 1, c);
    gfx_fill(x, y + r, 1, h - 2 * r, c);
    gfx_fill(x + w - 1, y + r, 1, h - 2 * r, c);
    float rr = (float)r - 0.5f;
    for (int j = 0; j < r; j++)
        for (int i = 0; i < r; i++) {
            float dx = (float)(r - i) - 0.5f, dy = (float)(r - j) - 0.5f;
            float d = sqrtf(dx * dx + dy * dy);
            float cov = 1.0f - fabsf(d - rr);
            if (cov <= 0.0f) continue;
            int a = (int)(cov * 15.0f + 0.5f);
            blend(x + i, y + j, ramp, a);
            blend(x + w - 1 - i, y + j, ramp, a);
            blend(x + i, y + h - 1 - j, ramp, a);
            blend(x + w - 1 - i, y + h - 1 - j, ramp, a);
        }
}

void gfx_shadow(int x, int y, int w, int h, int r, int blur, int strength, uint8_t ramp) {
    float hw = w * 0.5f, hh = h * 0.5f;
    float mx = x + hw - 0.5f, my = y + hh - 0.5f;
    for (int yy = y - blur; yy < y + h + blur; yy++) {
        if (yy < cy0 || yy >= cy1) continue;
        for (int xx = x - blur; xx < x + w + blur; xx++) {
            if (xx < cx0 || xx >= cx1) continue;
            /* signed distance to the rounded rectangle */
            float qx = fabsf(xx - mx) - (hw - r), qy = fabsf(yy - my) - (hh - r);
            float ox = qx > 0 ? qx : 0, oy = qy > 0 ? qy : 0;
            float d = sqrtf(ox * ox + oy * oy) + fminf(fmaxf(qx, qy), 0.0f) - r;
            if (d <= 0) {
                gfx_buf[yy * GFX_W + xx] = ramp + strength;
                continue;
            }
            float t = 1.0f - d / blur;
            if (t <= 0) continue;
            int a = (int)(strength * t * t + 0.5f);
            blend(xx, yy, ramp, a);
        }
    }
}

void gfx_disc(int ccx, int ccy, int r, uint8_t ramp) {
    for (int yy = ccy - r - 1; yy <= ccy + r + 1; yy++)
        for (int xx = ccx - r - 1; xx <= ccx + r + 1; xx++) {
            int a = disc_cov((float)xx, (float)yy, (float)ccx, (float)ccy, (float)r);
            if (a >= 15) put(xx, yy, ramp + 15); else blend(xx, yy, ramp, a);
        }
}

void gfx_shadow_map(int x, int y, const uint8_t *map, int w, int h, uint8_t ramp) {
    for (int j = 0; j < 2 * h; j++) {
        int yy = y + j;
        if (yy < cy0 || yy >= cy1) continue;
        const uint8_t *row = map + (j >> 1) * w;
        for (int i = 0; i < 2 * w; i++) {
            int a = row[i >> 1];
            if (a) blend(x + i, yy, ramp, a);
        }
    }
}

void gfx_blit(int x, int y, const uint8_t *src, int w, int h, int stride,
              uint8_t base, int scale, int r, int key) {
    int dw = w * scale, dh = h * scale;
    float rr = (float)r;
    for (int j = 0; j < dh; j++) {
        int yy = y + j;
        if (yy < cy0 || yy >= cy1) continue;
        const uint8_t *row = src + (j / scale) * stride;
        uint8_t *dst = &gfx_buf[yy * GFX_W];
        /* rounded corners: the span of this row inside the rounded box */
        int inset = 0;
        if (r > 0 && (j < r || j >= dh - r)) {
            float dy = (j < r) ? (rr - j - 0.5f) : (j - (dh - r) + 0.5f);
            inset = (int)(rr - sqrtf(rr * rr - dy * dy) + 0.5f);
        }
        int i0 = inset, i1 = dw - inset;
        if (x + i0 < cx0) i0 = cx0 - x;
        if (x + i1 > cx1) i1 = cx1 - x;
        if (key >= 0) {
            for (int i = i0; i < i1; i++) {
                uint8_t v = row[i / scale];
                if (v != key) dst[x + i] = base + v;
            }
        } else if (scale == 1) {
            for (int i = i0; i < i1; i++) dst[x + i] = base + row[i];
        } else {
            for (int i = i0; i < i1; i++) dst[x + i] = base + row[i / scale];
        }
    }
}

/* ------------------------------------------------------------------ */
/* Text                                                                */
/* ------------------------------------------------------------------ */

/* Next code point of a UTF-8 string; *n bytes left (decremented). */
static uint32_t utf8_next(const char **ps, int *n) {
    const uint8_t *s = (const uint8_t *)*ps;
    uint32_t c = s[0];
    int len = 1;
    if (c >= 0xF0 && *n >= 4) { c = (c & 0x07) << 18 | (s[1] & 0x3F) << 12 | (s[2] & 0x3F) << 6 | (s[3] & 0x3F); len = 4; }
    else if (c >= 0xE0 && *n >= 3) { c = (c & 0x0F) << 12 | (s[1] & 0x3F) << 6 | (s[2] & 0x3F); len = 3; }
    else if (c >= 0xC0 && *n >= 2) { c = (c & 0x1F) << 6 | (s[1] & 0x3F); len = 2; }
    *ps += len;
    *n -= len;
    return c;
}

static const font_glyph_t *glyph(const font_t *f, uint32_t cp) {
    if (cp == 0xA0 || cp == '\t') cp = ' ';
    int lo = 0, hi = f->nglyphs - 1;
    while (lo <= hi) {
        int mid = (lo + hi) >> 1;
        uint32_t g = f->glyphs[mid].cp;
        if (g == cp) return &f->glyphs[mid];
        if (g < cp) lo = mid + 1; else hi = mid - 1;
    }
    if (cp >= 'a' && cp <= 'z') return glyph(f, cp - 32); /* font_huge: capitals only */
    if (cp != '?') return glyph(f, '?');
    return &f->glyphs[0];
}

int text_width(const font_t *f, const char *s, int n) {
    if (n < 0) n = (int)strlen(s);
    int w16 = 0;
    while (n > 0 && *s) w16 += glyph(f, utf8_next(&s, &n))->adv16;
    return (w16 + 8) >> 4;
}

static int level_max = 15;

void text_level_max(int max) {
    level_max = max < 1 ? 1 : max > 15 ? 15 : max;
}

static int draw_n(const font_t *f, int x, int y, const char *s, int n, uint8_t ramp) {
    int pen16 = x * 16;
    while (n > 0 && *s) {
        const font_glyph_t *g = glyph(f, utf8_next(&s, &n));
        if (g->w) {
            const uint8_t *bm = f->bitmap + g->off;
            int bpr = (g->w + 1) >> 1;
            int gx = ((pen16 + 8) >> 4) + g->x0, gy = y + g->y0;
            for (int j = 0; j < g->h; j++) {
                const uint8_t *row = bm + j * bpr;
                for (int i = 0; i < g->w; i++) {
                    int a = (i & 1) ? (row[i >> 1] & 15) : (row[i >> 1] >> 4);
                    if (a) blend(gx + i, gy + j, ramp, (a * level_max + 7) / 15);
                }
            }
        }
        pen16 += g->adv16;
    }
    return (pen16 + 8) >> 4;
}

int text_draw(const font_t *f, int x, int y, const char *s, uint8_t ramp) {
    return draw_n(f, x, y, s, (int)strlen(s), ramp);
}

/* n bytes of s, cut to fit max_w with an ellipsis (always, if `more`). */
static int draw_fit_n(const font_t *f, int x, int y, int max_w, const char *s, int n,
                      bool more, uint8_t ramp) {
    static const char ell[] = "\xE2\x80\xA6"; /* U+2026 */
    if (!more && text_width(f, s, n) <= max_w) return draw_n(f, x, y, s, n, ramp);
    int ew = text_width(f, ell, -1);
    /* longest prefix (on a character boundary) that fits with the ellipsis */
    int best = 0, w16 = 0;
    const char *p = s;
    int left = n;
    while (left > 0 && *p) {
        const char *q = p;
        int l2 = left;
        uint32_t cp = utf8_next(&q, &l2);
        w16 += glyph(f, cp)->adv16;
        if (((w16 + 8) >> 4) + ew > max_w) break;
        p = q;
        left = l2;
        best = (int)(p - s);
    }
    /* no trailing space before the ellipsis */
    while (best > 0 && s[best - 1] == ' ') best--;
    int xe = draw_n(f, x, y, s, best, ramp);
    return draw_n(f, xe, y, ell, 3, ramp);
}

int text_draw_fit(const font_t *f, int x, int y, int max_w, const char *s, uint8_t ramp) {
    return draw_fit_n(f, x, y, max_w, s, (int)strlen(s), false, ramp);
}

void text_draw_center(const font_t *f, int cx, int y, const char *s, uint8_t ramp) {
    text_draw(f, cx - text_width(f, s, -1) / 2, y, s, ramp);
}

int text_draw_wrap(const font_t *f, int x, int y, int max_w, int lh, int max_lines,
                   const char *s, uint8_t ramp, bool draw) {
    int lines = 0;
    while (*s && lines < max_lines) {
        while (*s == ' ') s++;
        if (!*s) break;
        /* the longest run of whole words that fits */
        const char *end = s, *brk = NULL;
        while (*end && *end != '\n') {
            const char *w = end;
            while (*w == ' ') w++;
            const char *we = w;
            while (*we && *we != ' ' && *we != '\n') we++;
            if (text_width(f, s, (int)(we - s)) > max_w) break;
            brk = we;
            end = we;
        }
        bool hard = false;
        if (!brk) {
            /* a single word wider than the line: cut it */
            const char *we = s;
            while (*we && *we != ' ' && *we != '\n') we++;
            brk = we;
            hard = true;
        }
        const char *next = brk;
        while (*next == ' ') next++;
        if (*next == '\n') next++;
        bool last = lines == max_lines - 1;
        bool more = *next != 0;
        if (draw) {
            if (last || hard) draw_fit_n(f, x, y + lines * lh, max_w, s, (int)(brk - s), last && more, ramp);
            else draw_n(f, x, y + lines * lh, s, (int)(brk - s), ramp);
        }
        lines++;
        s = next;
    }
    return lines;
}
