/*
 * frank-kickstart - video bring-up firmware (make frank-kickstart-vtest)
 *
 * Test patterns on the 640x480 8-bit framebuffer; 'n' on the UART (or every
 * 8 s) moves to the next one. Once a second the UART gets
 *   video: <backend> frames=<n> fps=<n> irq_max=<us> errors=<n>
 * "~BOOT~" reboots into BOOTSEL, "~RST~" reboots.
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "board_config.h"
#include "platform.h"
#include "video.h"

static uint32_t pal[256];

static void set_pal(void) {
    video_stage_palette(0, 256, pal);
    video_commit_palette();
}

static void core1_main(void) {
    video_start();
    while (true) __wfe();
}

static void fill(int x, int y, int w, int h, uint8_t c) {
    for (int j = y; j < y + h; j++) memset(&video_fb[j * VIDEO_W + x], c, w);
}

/* 0: colour bars over a 240-step grey ramp */
static void pattern_bars(void) {
    static const uint32_t bars[8] = {0xFFFFFF, 0xFFFF00, 0x00FFFF, 0x00FF00,
                                     0xFF00FF, 0xFF0000, 0x0000FF, 0x000000};
    for (int i = 0; i < 8; i++) pal[i] = bars[i];
    for (int i = 0; i < 240; i++) {
        uint32_t v = i * 255 / 239;
        pal[16 + i] = v << 16 | v << 8 | v;
    }
    set_pal();
    for (int i = 0; i < 8; i++) fill(i * 80, 0, 80, 320, i);
    for (int x = 0; x < VIDEO_W; x++)
        for (int y = 320; y < VIDEO_H; y++) video_fb[y * VIDEO_W + x] = 16 + x * 240 / VIDEO_W;
}

/* 1: 64-step red, green, blue and grey ramps, full width */
static void pattern_ramps(void) {
    for (int i = 0; i < 64; i++) {
        uint32_t v = i * 255 / 63;
        pal[i] = v << 16;
        pal[64 + i] = v << 8;
        pal[128 + i] = v;
        pal[192 + i] = v << 16 | v << 8 | v;
    }
    set_pal();
    for (int band = 0; band < 4; band++)
        for (int y = band * 120; y < band * 120 + 120; y++)
            for (int x = 0; x < VIDEO_W; x++) video_fb[y * VIDEO_W + x] = band * 64 + x * 64 / VIDEO_W;
}

/* 2: the worst cases for the encoder: 1-pixel red/green columns,
 * a black/white checkerboard, 1-pixel white lines on black */
static void pattern_stress(void) {
    pal[0] = 0x000000; pal[1] = 0xFFFFFF; pal[2] = 0xFF0000; pal[3] = 0x00FF00;
    pal[4] = 0x0000FF; pal[5] = 0xFFFF00;
    set_pal();
    for (int y = 0; y < 160; y++)
        for (int x = 0; x < VIDEO_W; x++) video_fb[y * VIDEO_W + x] = (x & 1) ? 3 : 2;
    for (int y = 160; y < 320; y++)
        for (int x = 0; x < VIDEO_W; x++) video_fb[y * VIDEO_W + x] = ((x ^ y) & 1);
    for (int y = 320; y < VIDEO_H; y++)
        for (int x = 0; x < VIDEO_W; x++)
            video_fb[y * VIDEO_W + x] = (y & 1) ? ((x % 8) == 0) : ((x & 1) ? 4 : 5);
    /* a 1-pixel frame round the edge: every corner pixel must be visible */
    fill(0, 0, VIDEO_W, 1, 1); fill(0, VIDEO_H - 1, VIDEO_W, 1, 1);
    fill(0, 0, 1, VIDEO_H, 1); fill(VIDEO_W - 1, 0, 1, VIDEO_H, 1);
}

/* 3: a box moving every frame, drawn right after vblank (tearing check) */
static int box_x = 0, box_dx = 4;
static void pattern_motion_setup(void) {
    pal[0] = 0x101830; pal[1] = 0xF0F0F0; pal[2] = 0xE04040;
    set_pal();
    memset(video_fb, 0, sizeof(video_fb));
}
static void pattern_motion_step(void) {
    fill(box_x, 180, 120, 120, 0);
    box_x += box_dx;
    if (box_x < 0 || box_x + 120 > VIDEO_W) { box_dx = -box_dx; box_x += 2 * box_dx; }
    fill(box_x, 180, 120, 120, 2);
    fill(box_x + 58, 180, 4, 120, 1);
}

int main(void) {
    platform_init_clocks();
    stdio_init_all();
    printf("\nfrank-kickstart video test, board %s, %lu MHz\n", BOARD_NAME,
           (unsigned long)(clock_get_hz(clk_sys) / 1000000));

    memset(video_fb, 0, sizeof(video_fb));
    video_init();
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS |
                BUSCTRL_BUS_PRIORITY_DMA_R_BITS | BUSCTRL_BUS_PRIORITY_DMA_W_BITS);
    multicore_launch_core1(core1_main);
    printf("video: %s started\n", video_backend_name());

    int pattern = -1;
    absolute_time_t next_pattern = get_absolute_time();
    absolute_time_t next_log = make_timeout_time_ms(1000);
    uint32_t last_frames = video_frame_count();
    while (true) {
        bool advance = time_reached(next_pattern);
        int c;
        while ((c = getchar_timeout_us(0)) >= 0) {
            platform_uart_cmd(c);
            if (c == 'n') advance = true;
        }
        if (advance) {
            pattern = (pattern + 1) % 4;
            next_pattern = make_timeout_time_ms(8000);
            switch (pattern) {
            case 0: pattern_bars(); break;
            case 1: pattern_ramps(); break;
            case 2: pattern_stress(); break;
            case 3: pattern_motion_setup(); break;
            }
            printf("pattern %d\n", pattern);
        }
        if (pattern == 3) {
            video_wait_vblank();
            pattern_motion_step();
        } else {
            sleep_ms(5);
        }
        if (time_reached(next_log)) {
            next_log = make_timeout_time_ms(1000);
            uint32_t f = video_frame_count();
            printf("video: %s frames=%lu fps=%lu irq_max=%lu errors=%lu\n", video_backend_name(),
                   (unsigned long)f, (unsigned long)(f - last_frames),
                   (unsigned long)video_irq_max_us(), (unsigned long)video_errors());
            last_frames = f;
        }
    }
}
