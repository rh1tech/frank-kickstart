/*
 * frank-kickstart - 640x480@60 8-bit indexed video, one API over two backends:
 *   video_hstx.c  M2: HSTX HDMI (drivers/hdmi/psx_video.c), BGR555 output
 *   video_pio.c   M1/Z0: PIO HDMI (drivers/hdmi_pio640), balanced-TMDS output
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FK_VIDEO_API_H
#define FK_VIDEO_API_H

#include <stdbool.h>
#include <stdint.h>

#define VIDEO_W 640
#define VIDEO_H 480

/* The framebuffer the screen is scanned out of: one palette index per pixel. */
extern uint8_t video_fb[VIDEO_W * VIDEO_H];

/* Claims the DMA channels, PIO/HSTX and builds the tables. Core 0, before
 * anything else claims a DMA channel. */
void video_init(void);

/* Starts the scan-out; its IRQ runs on the calling core (core 1). */
void video_start(void);

/* Stops the scan-out (DMA aborted, state machines/HSTX off, IRQ off). Call
 * on the core that called video_start(). */
void video_stop(void);

/* Stages palette entries [first, first+n) from 0xRRGGBB values; all staged
 * changes take effect together at the next vblank (video_commit_palette). */
void video_stage_palette(int first, int n, const uint32_t *rgb888);
void video_commit_palette(void);
bool video_palette_pending(void);

/* The colour the output really shows for 0xRRGGBB (HSTX: 5 bits per
 * channel; PIO: the DC-balanced levels). Smooth gradients dither between
 * such colours. */
uint32_t video_quantize(uint32_t rgb888);

/* HDMI sound: the source gives stereo frames (left | right << 16) at 44.1
 * kHz, called from the video IRQ (it must be in SRAM). Returns false where
 * the backend has no audio (PIO). NULL: silence. */
typedef uint32_t (*video_audio_source_t)(void);
bool video_set_audio_source(video_audio_source_t source);

/* Called by video_wait_vblank() on its way out, on the waiting core (the
 * music's SD reads); NULL: none. */
void video_set_frame_hook(void (*hook)(void));

/* Frame counter, incremented at the start of every vblank. */
uint32_t video_frame_count(void);
void video_wait_vblank(void);

/* Diagnostics: worst line-IRQ time (us) since the last call, and the
 * number of scan-out resyncs/stalls seen so far. */
uint32_t video_irq_max_us(void);
uint32_t video_errors(void);
const char *video_backend_name(void);

#endif
