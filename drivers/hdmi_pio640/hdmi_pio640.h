/*
 * frank-kickstart - PIO HDMI, 640x480@60 from an 8-bit indexed framebuffer,
 * every pixel its own (see hdmi_pio640.c). clk_sys a multiple of 252 MHz.
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef HDMI_PIO640_H
#define HDMI_PIO640_H

#include <stdbool.h>
#include <stdint.h>

/* Tables, PIO, DMA channels 0 and 1; scans fb (640 x 480 bytes) out. */
void hdmi_pio640_init(const uint8_t *fb);
/* DMA_IRQ_0 on the calling core, then the output starts. */
void hdmi_pio640_start(void);
void hdmi_pio640_stop(void);

/* Palette entry i (0xRRGGBB, each channel snapped to a balanced level) into
 * the staged palette; hdmi_pio640_commit() has the next vblank take it. */
void hdmi_pio640_stage(int i, uint32_t rgb);
void hdmi_pio640_commit(void);
bool hdmi_pio640_pending(void);

extern volatile uint32_t hdmi_pio640_frames;     /* at the start of vblank */
extern volatile uint32_t hdmi_pio640_irq_max_us; /* the worst line IRQ */
extern volatile uint32_t hdmi_pio640_errors;     /* output FIFO underruns */

#endif
