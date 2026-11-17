/*
 * frank-kickstart - drawing on the 8-bit framebuffer (video_fb, 640x480)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Colours are palette indices. Anti-aliasing goes through 16-entry "ramps"
 * (palette.h): ramp + 0 is the background a shape sits on, ramp + 15 its
 * full colour, the indices between are the blends.
 */
#ifndef FK_GFX_H
#define FK_GFX_H

#include <stdbool.h>
#include <stdint.h>
#include "font.h"
#include "video.h"

#define GFX_W VIDEO_W
#define GFX_H VIDEO_H

/* Where everything below draws: a 640x480 buffer (NULL: video_fb). Whole
 * screens are composed in PSRAM and copied in with gfx_present(). */
void gfx_target(uint8_t *buf);
uint8_t *gfx_target_buf(void);

/* Clip rectangle for everything below (x0,y0 inclusive, x1,y1 exclusive). */
void gfx_clip(int x0, int y0, int x1, int y1);
void gfx_noclip(void);

void gfx_fill(int x, int y, int w, int h, uint8_t c);
void gfx_hline(int x, int y, int w, uint8_t c);

/* Vertical gradient over the n palette entries first..first+n-1, ordered
 * (4x4 Bayer) dither between neighbours. */
void gfx_vgradient(int x, int y, int w, int h, uint8_t first, int n);

/* Rounded rectangle filled with ramp+15, its corners anti-aliased through
 * ramp+1..14 (ramp+0 = the colour around it). */
void gfx_round_rect(int x, int y, int w, int h, int r, uint8_t ramp);

/* 1-pixel rounded outline in ramp+15, anti-aliased the same way. */
void gfx_round_frame(int x, int y, int w, int h, int r, uint8_t ramp);

/* Soft shadow under a rounded rectangle: falls off over `blur` pixels
 * through the ramp (ramp+15 darkest, `strength` 0..15 at the edge). */
void gfx_shadow(int x, int y, int w, int h, int r, int blur, int strength, uint8_t ramp);

/* Anti-aliased filled circle. */
void gfx_disc(int cx, int cy, int r, uint8_t ramp);

/* 8-bit image: every source index i is drawn as base + i, except index key
 * (see-through; -1: none), scaled by `scale` (1 or 2), corners rounded to r. */
void gfx_blit(int x, int y, const uint8_t *src, int w, int h, int stride,
              uint8_t base, int scale, int r, int key);

/* Shadow levels (0..15, a w x h map at half resolution) drawn at 2x through
 * the ramp, each pixel keeping the darker of what it has and the shadow. */
void gfx_shadow_map(int x, int y, const uint8_t *map, int w, int h, uint8_t ramp);

/* --- text (gfx.c, UTF-8) --- */

/* Text drawn after this goes at most to ramp + max (1..15, default 15):
 * a dimmer colour from the same ramp, still anti-aliased. */
void text_level_max(int max);

/* Width in pixels of s (the first n bytes; n < 0: all of it). */
int text_width(const font_t *f, const char *s, int n);

/* Draws s with its baseline at y; returns the x after it. */
int text_draw(const font_t *f, int x, int y, const char *s, uint8_t ramp);

/* As text_draw, cut with an ellipsis to fit max_w. */
int text_draw_fit(const font_t *f, int x, int y, int max_w, const char *s, uint8_t ramp);

/* Centred on cx. */
void text_draw_center(const font_t *f, int cx, int y, const char *s, uint8_t ramp);

/* Word-wrapped into max_w, at most max_lines lines (the last one ending in
 * an ellipsis if the text goes on), line pitch lh, first baseline at y.
 * Returns the number of lines drawn; with draw=false only counts them. */
int text_draw_wrap(const font_t *f, int x, int y, int max_w, int lh, int max_lines,
                   const char *s, uint8_t ramp, bool draw);

#endif
