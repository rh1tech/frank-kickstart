/*
 * frank-kickstart - video backend for M2: HSTX HDMI (drivers/hdmi/psx_video.c)
 * in its native 640-pixel line mode. Each line is looked up through a
 * 256-entry BGR555 palette in the line IRQ.
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/structs/hstx_ctrl.h"
#include "psx_video.h"
#include "hstx_data_island_queue.h"
#include "video.h"

#define NSLOT 4 /* line buffers: the DMA reads a slot up to two lines after the callback */

uint8_t video_fb[VIDEO_W * VIDEO_H] __attribute__((aligned(4)));

static uint16_t pal[2][256] __attribute__((aligned(4)));
static volatile int pal_front;
static uint16_t pal_stage[256] __attribute__((aligned(4)));
static volatile bool pal_pending;
static uint32_t line_buf[NSLOT][VIDEO_W / 2];
static volatile uint32_t vsync_count;

extern volatile uint32_t psxv_irq_max_us;
extern volatile uint32_t psxv_resyncs;

static void __not_in_flash_func(on_vsync)(void) {
    if (pal_pending) {
        int back = pal_front ^ 1;
        /* a loop, not memcpy: newlib's is in flash, and this IRQ must keep
         * running from RAM while the flash is being written */
        uint32_t *d = (uint32_t *)pal[back];
        const uint32_t *s = (const uint32_t *)pal_stage;
        for (int i = 0; i < 128; i++) d[i] = s[i];
        pal_front = back;
        pal_pending = false;
    }
    vsync_count++;
}

static void __not_in_flash_func(on_line)(uint32_t out_line, psxv_line_t *line) {
    uint32_t *dst = line_buf[out_line & (NSLOT - 1)];
    const uint32_t *src = (const uint32_t *)(video_fb + out_line * VIDEO_W);
    const uint16_t *p = pal[pal_front];
    for (int i = 0; i < VIDEO_W / 4; i++) {
        uint32_t s = src[i];
        dst[2 * i] = p[s & 0xff] | (uint32_t)p[(s >> 8) & 0xff] << 16;
        dst[2 * i + 1] = p[(s >> 16) & 0xff] | (uint32_t)p[s >> 24] << 16;
    }
    line->pixels = (const uint16_t *)dst;
    line->mode = PSXV_LINE_NATIVE;
}

void video_init(void) {
    psxv_init();
    psxv_set_line_callback(on_line);
    psxv_set_vsync_callback(on_vsync);
}

void video_start(void) {
    psxv_start();
}

void video_stop(void) {
    irq_set_enabled(DMA_IRQ_0, false);
    dma_channel_abort(0);
    dma_channel_abort(1);
    hstx_ctrl_hw->csr = 0;
}

static inline uint16_t to_bgr555(uint32_t c) {
    uint32_t r = (c >> 16) & 0xff, g = (c >> 8) & 0xff, b = c & 0xff;
    return (uint16_t)((b >> 3) << 10 | (g >> 3) << 5 | r >> 3);
}

uint32_t video_quantize(uint32_t c) {
    uint32_t out = 0;
    for (int sh = 0; sh <= 16; sh += 8) out |= (((c >> sh) & 0xff) >> 3) * 255 / 31 << sh;
    return out;
}

void video_stage_palette(int first, int n, const uint32_t *rgb888) {
    /* pal_stage always holds the latest wanted palette; a commit in flight
     * is copying it, so wait for the vblank to take it first */
    while (pal_pending) tight_loop_contents();
    for (int i = 0; i < n && first + i < 256; i++) pal_stage[first + i] = to_bgr555(rgb888[i]);
}

void video_commit_palette(void) {
    pal_pending = true;
}

bool video_palette_pending(void) {
    return pal_pending;
}

uint32_t video_frame_count(void) {
    return vsync_count;
}

static void (*frame_hook)(void);

void video_set_frame_hook(void (*hook)(void)) {
    frame_hook = hook;
}

void video_wait_vblank(void) {
    uint32_t n = vsync_count;
    while (vsync_count == n) tight_loop_contents();
    if (frame_hook) frame_hook();
}

bool video_set_audio_source(video_audio_source_t source) {
    hstx_di_queue_set_audio_source_full(source);
    return true;
}

uint32_t video_irq_max_us(void) {
    uint32_t v = psxv_irq_max_us;
    psxv_irq_max_us = 0;
    return v;
}

uint32_t video_errors(void) {
    return psxv_resyncs;
}

const char *video_backend_name(void) {
    return "HSTX";
}
