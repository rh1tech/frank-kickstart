/*
 * frank-kickstart - the 256-colour palette
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <math.h>
#include "palette.h"
#include "video.h"

uint32_t pal_master[256];
static uint32_t pal_out[256];

/* sRGB <-> linear, 8-bit in, 12-bit linear */
static uint16_t to_lin[256];
static uint8_t from_lin[4096];
static int lin_ready;

static void lin_init(void) {
    for (int i = 0; i < 256; i++) to_lin[i] = (uint16_t)(powf(i / 255.0f, 2.2f) * 4095.0f + 0.5f);
    for (int i = 0; i < 4096; i++) from_lin[i] = (uint8_t)(powf(i / 4095.0f, 1.0f / 2.2f) * 255.0f + 0.5f);
    lin_ready = 1;
}

uint32_t pal_mix(uint32_t a, uint32_t b, int k) {
    if (!lin_ready) lin_init();
    uint32_t out = 0;
    for (int sh = 0; sh <= 16; sh += 8) {
        int la = to_lin[(a >> sh) & 0xff], lb = to_lin[(b >> sh) & 0xff];
        int l = la + (lb - la) * k / 256;
        out |= (uint32_t)from_lin[l < 0 ? 0 : l > 4095 ? 4095 : l] << sh;
    }
    return out;
}

void pal_set(int i, uint32_t rgb) {
    pal_master[i & 255] = rgb & 0xFFFFFF;
}

void pal_ramp(int base, uint32_t from, uint32_t to) {
    for (int i = 0; i < 16; i++) pal_set(base + i, pal_mix(from, to, i * 256 / 15));
}

void pal_begin(void) {
    for (int i = 0; i < 256; i++) pal_out[i] = pal_master[i];
}

void pal_fade(int first, int n, uint32_t to, int k) {
    if (k >= 256) return;
    for (int i = first; i < first + n && i < 256; i++) pal_out[i] = pal_mix(to, pal_out[i], k);
}

void pal_end(void) {
    video_stage_palette(0, 256, pal_out);
    video_commit_palette();
}

void pal_show(void) {
    pal_begin();
    pal_end();
}
