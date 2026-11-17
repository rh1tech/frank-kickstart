/*
 * frank-kickstart - the 256-colour palette: a master copy on core 0, ramps,
 * and fades applied at vblank
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FK_PALETTE_H
#define FK_PALETTE_H

#include <stdint.h>

extern uint32_t pal_master[256]; /* 0xRRGGBB */

void pal_set(int i, uint32_t rgb);

/* 16 entries base..base+15 from `from` to `to`, blended in linear light
 * (the right steps for anti-aliased edges). */
void pal_ramp(int base, uint32_t from, uint32_t to);

/* a + (b - a) * k / 256, per channel in linear light */
uint32_t pal_mix(uint32_t a, uint32_t b, int k);

/* Show the palette: pal_begin() copies the master, pal_fade() mixes a range
 * of that copy towards a colour (k = 256: unchanged, 0: all `to`),
 * pal_end() stages it for the next vblank. */
void pal_begin(void);
void pal_fade(int first, int n, uint32_t to, int k);
void pal_end(void);

/* pal_begin(); pal_end(); */
void pal_show(void);

#endif
