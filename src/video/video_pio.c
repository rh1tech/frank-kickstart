/*
 * frank-kickstart - video backend for M1/Z0 (and M2 with VIDEO_BACKEND=PIO):
 * PIO HDMI, drivers/hdmi_pio640. Every palette channel is snapped to a
 * level with a DC-balanced TMDS symbol (52 of 256, 16..239).
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "pico/stdlib.h"
#include "hdmi_pio640.h"
#include "video.h"
#include "tmds_balanced.h"

uint8_t video_fb[VIDEO_W * VIDEO_H] __attribute__((aligned(4)));

void video_init(void) {
    hdmi_pio640_init(video_fb);
}

void video_start(void) {
    hdmi_pio640_start();
}

void video_stop(void) {
    hdmi_pio640_stop();
}

void video_stage_palette(int first, int n, const uint32_t *rgb888) {
    for (int i = 0; i < n && first + i < 256; i++) hdmi_pio640_stage(first + i, rgb888[i]);
}

void video_commit_palette(void) {
    hdmi_pio640_commit();
}

bool video_palette_pending(void) {
    return hdmi_pio640_pending();
}

uint32_t video_frame_count(void) {
    return hdmi_pio640_frames;
}

static void (*frame_hook)(void);

void video_set_frame_hook(void (*hook)(void)) {
    frame_hook = hook;
}

void video_wait_vblank(void) {
    uint32_t n = hdmi_pio640_frames;
    while (hdmi_pio640_frames == n) tight_loop_contents();
    if (frame_hook) frame_hook();
}

/* no HDMI sound on the PIO output */
bool video_set_audio_source(video_audio_source_t source) {
    (void)source;
    return false;
}

uint32_t video_irq_max_us(void) {
    uint32_t v = hdmi_pio640_irq_max_us;
    hdmi_pio640_irq_max_us = 0;
    return v;
}

uint32_t video_errors(void) {
    return hdmi_pio640_errors;
}

uint32_t video_quantize(uint32_t c) {
    uint32_t out = 0;
    for (int sh = 0; sh <= 16; sh += 8) out |= (uint32_t)tmds_balanced_snap((uint8_t)(c >> sh)) << sh;
    return out;
}

const char *video_backend_name(void) {
    return "PIO";
}
