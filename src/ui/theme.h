/*
 * frank-kickstart - colours and palette layout of the launcher screens
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 *   0..7     flat UI colours
 *   8..15    the ambient glow behind the cover (page .. cover tint)
 *   16..127  seven 16-step ramps for anti-aliased text and shapes
 *   128..143 the header and footer ramp (bar -> white), never faded
 *   144..255 the cover art on screen (112 colours)
 */
#ifndef FK_THEME_H
#define FK_THEME_H

#define C_BG        0
#define C_BAR       1
#define C_CARD      2
#define C_DIVIDER   3
#define C_GREEN     4
#define C_ACCENT    5
#define C_WHITE     6
#define C_BLACK     7
#define C_GLOW      8   /* 8 levels: page colour .. glow at full */
#define N_GLOW      8
#define GLOW_MAX    22  /* /256 (linear light): how far the glow goes towards the tint */

#define RAMP_TEXT   16  /* page -> primary text */
#define RAMP_MUTED  32  /* page -> secondary text */
#define RAMP_ACCENT 48  /* page -> accent */
#define RAMP_SHADOW 64  /* page -> black */
#define RAMP_CARD   80  /* page -> card / key cap */
#define RAMP_ONCARD 96  /* card -> primary text */
#define RAMP_GREEN  112 /* page -> green */

#define RAMP_CHROME 128 /* bar -> white: the header and footer, never faded */

#define ART_BASE    144
#define ART_COLORS  112

#define RGB_PAGE     0x0F1421U  /* the page; the text ramps start from it */
#define RGB_BAR      0x111726U
#define RGB_CARD     0x1F2639U
#define RGB_DIVIDER  0x262E44U
#define RGB_TEXT     0xEEF1F8U
#define RGB_MUTED    0x8B95AC
#define RGB_ACCENT   0xA97CFFU  /* the violet of the welcome screen's grid, lighter */
#define RGB_GREEN    0x3DD68CU
#define RGB_SHADOW   0x000000U

/* Loads the launcher palette into pal_master (the art slice is left alone). */
void theme_load_palette(void);

#endif
