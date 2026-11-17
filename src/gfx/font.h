/*
 * frank-kickstart - 4-bit anti-aliased proportional fonts (tools/mkfont.py)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FK_FONT_H
#define FK_FONT_H

#include <stdint.h>

typedef struct {
    uint16_t cp;       /* Unicode code point; the table is sorted by it */
    uint8_t w, h;      /* ink box */
    int8_t x0, y0;     /* ink box offset from the pen on the baseline (y0 < 0: above) */
    uint16_t adv16;    /* advance, 1/16 pixel */
    uint32_t off;      /* first byte in the bitmap: rows of (w+1)/2 bytes, high nibble first */
} font_glyph_t;

typedef struct {
    uint8_t ascent, descent, line_h;
    uint16_t nglyphs;
    const font_glyph_t *glyphs;
    const uint8_t *bitmap;
} font_t;

/* The built-in fonts (src/fonts/) */
extern const font_t font_small;    /* Inter Medium 12: labels, hints */
extern const font_t font_body;     /* Inter Regular 15: descriptions */
extern const font_t font_bold;     /* Inter SemiBold 15: values, buttons */
extern const font_t font_title;    /* Inter Display Bold 30: titles */
extern const font_t font_huge;     /* Inter Display Black 52, the letters of FRANK: the welcome logo */
extern const font_t font_kick;     /* Inter Display Black 16, the letters of KICKSTART */
extern const font_t font_count;    /* Inter Display Black 84, 1..4: the start countdown */

#endif
