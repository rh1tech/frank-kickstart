/*
 * frank-kickstart - background music over HDMI (see music.h)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "ff.h"
#include "music.h"
#include "video.h"

#define BLOCK 1024                   /* IMA ADPCM block (stereo) */
#define BLOCK_FRAMES 1017            /* 1 + (BLOCK - 8) / 8 * 8 */
#define CBLOCKS 16                   /* compressed ring: 16 KB, 0.37 s */
#define PCM_FRAMES 2048              /* decoded ring (a power of 2) */
#define VOLUME 20000                 /* of 32768 */
#define FADE_STEP 1                  /* per sample: 0.45 s to VOLUME */

static FIL file;
static bool open_;
static uint32_t data_start, data_blocks, next_block;

/* core 0 writes cwr, core 1 cread; core 1 writes pwr, the IRQ (core 1) prd */
static uint8_t cring[CBLOCKS][BLOCK];
static volatile uint32_t cwr, crd;
static uint32_t pcm[PCM_FRAMES];
static volatile uint32_t pwr, prd;
static volatile int32_t vol, vol_target;

/* in RAM, not const: core 1 decodes while the flash is written */
static int16_t step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230,
    253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963,
    1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327,
    3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487,
    12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
};
static int8_t index_table[16] = {-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8};

typedef struct {
    int pred, idx;
} ima_t;

static inline int __not_in_flash_func(ima_step)(ima_t *c, int n) {
    int step = step_table[c->idx];
    int diff = ((2 * (n & 7) + 1) * step) >> 3; /* as ffmpeg (the encoder) */
    int p = (n & 8) ? c->pred - diff : c->pred + diff;
    if (p > 32767) p = 32767;
    if (p < -32768) p = -32768;
    c->pred = p;
    int i = c->idx + index_table[n];
    c->idx = i < 0 ? 0 : i > 88 ? 88 : i;
    return p;
}

void __not_in_flash_func(music_core1)(void) {
    while (crd != cwr && pwr - prd <= PCM_FRAMES - BLOCK_FRAMES) {
        const uint8_t *b = cring[crd % CBLOCKS];
        ima_t l = {(int16_t)(b[0] | b[1] << 8), b[2] > 88 ? 88 : b[2]};
        ima_t r = {(int16_t)(b[4] | b[5] << 8), b[6] > 88 ? 88 : b[6]};
        uint32_t w = pwr;
        pcm[w++ % PCM_FRAMES] = (uint16_t)l.pred | (uint32_t)(uint16_t)r.pred << 16;
        for (const uint8_t *g = b + 8; g < b + BLOCK; g += 8) {
            /* 4 bytes left (8 samples, low nibble first), 4 bytes right */
            for (int k = 0; k < 8; k++) {
                int ln = (g[k >> 1] >> ((k & 1) * 4)) & 15;
                int rn = (g[4 + (k >> 1)] >> ((k & 1) * 4)) & 15;
                int ls = ima_step(&l, ln), rs = ima_step(&r, rn);
                pcm[(w + k) % PCM_FRAMES] = (uint16_t)ls | (uint32_t)(uint16_t)rs << 16;
            }
            w += 8;
        }
        __dmb();
        crd = crd + 1;
        pwr = w;
    }
}

/* The HDMI IRQ: one stereo frame (left | right << 16), silence when dry. */
static uint32_t __not_in_flash_func(music_sample)(void) {
    int v = vol;
    if (v != vol_target) vol = v < vol_target ? v + FADE_STEP : v - FADE_STEP;
    if (prd == pwr) return 0;
    uint32_t f = pcm[prd % PCM_FRAMES];
    prd = prd + 1;
    int l = ((int16_t)f * v) >> 15, r = ((int16_t)(f >> 16) * v) >> 15;
    return (uint16_t)l | (uint32_t)(uint16_t)r << 16;
}

/* Up to n blocks from the file into the ring (wrapping at the end). */
static void fill(uint32_t n) {
    while (n && cwr - crd < CBLOCKS) {
        uint32_t slot = cwr % CBLOCKS;
        uint32_t k = n;
        if (k > CBLOCKS - (cwr - crd)) k = CBLOCKS - (cwr - crd);
        if (k > CBLOCKS - slot) k = CBLOCKS - slot;
        if (k > data_blocks - next_block) k = data_blocks - next_block;
        UINT br = 0;
        if (f_read(&file, cring[slot], k * BLOCK, &br) != FR_OK || br != k * BLOCK) {
            printf("music: read failed\n");
            open_ = false;
            f_close(&file);
            return;
        }
        __dmb();
        cwr = cwr + k;
        n -= k;
        next_block += k;
        if (next_block == data_blocks) {
            next_block = 0;
            f_lseek(&file, data_start);
        }
    }
}

void music_pump(void) {
    if (!open_) return;
    /* a block a frame keeps up (60 KB/s against 44); two when it runs low,
     * so a frame never waits long on the card */
    fill(cwr - crd < CBLOCKS / 2 ? 2 : 1);
}

static uint32_t le32(const uint8_t *p) {
    return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

bool music_start(const char *path, bool on) {
    if (open_) return true;
    if (f_open(&file, path, FA_READ) != FR_OK) return false;
    uint8_t h[64];
    UINT br;
    bool fmt_ok = false;
    uint32_t pos = 12;
    if (f_read(&file, h, 12, &br) != FR_OK || br != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4))
        goto bad;
    for (;;) {
        if (f_lseek(&file, pos) != FR_OK || f_read(&file, h, 8, &br) != FR_OK || br != 8) goto bad;
        uint32_t len = le32(h + 4);
        if (!memcmp(h, "fmt ", 4)) {
            if (f_read(&file, h + 8, 16, &br) != FR_OK || br != 16) goto bad;
            /* IMA ADPCM, 2 channels, 44100 Hz, 1024-byte blocks */
            fmt_ok = (h[8] | h[9] << 8) == 0x11 && (h[10] | h[11] << 8) == 2 && le32(h + 12) == 44100 &&
                     (h[20] | h[21] << 8) == BLOCK;
        } else if (!memcmp(h, "data", 4)) {
            if (!fmt_ok) goto bad;
            data_start = pos + 8;
            data_blocks = len / BLOCK;
            break;
        }
        pos += 8 + len + (len & 1);
    }
    if (!data_blocks) goto bad;
    cwr = crd = pwr = prd = 0;
    next_block = 0;
    vol = 0;
    vol_target = on ? VOLUME : 0;
    open_ = true;
    fill(CBLOCKS);
    if (!open_ || !video_set_audio_source(music_sample)) {
        if (open_) f_close(&file);
        open_ = false;
        return false;
    }
    video_set_frame_hook(music_pump);
    printf("music: %s, %lu s\n", path, (unsigned long)(data_blocks * BLOCK_FRAMES / 44100));
    return true;
bad:
    printf("music: %s is not an IMA ADPCM 44.1 kHz stereo WAV\n", path);
    f_close(&file);
    return false;
}

bool music_available(void) {
    return open_;
}

void music_set_on(bool on) {
    vol_target = on ? VOLUME : 0;
}

bool music_is_on(void) {
    return open_ && vol_target != 0;
}

void music_stop(void) {
    if (!open_) return;
    vol_target = 0;
    for (int i = 0; i < 40 && vol; i++) video_wait_vblank();
    video_set_frame_hook(NULL);
    video_set_audio_source(NULL);
    open_ = false;
    f_close(&file);
}
