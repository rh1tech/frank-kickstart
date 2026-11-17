/*
 * frank-kickstart - PIO HDMI, 640x480@60, one 8-bit index a pixel
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * From the frank-quake2 PIO HDMI driver (itself from frank-msx / pico-nes
 * and the HDMI framing of shuichitakano's PicoDVI): the same 10-instruction
 * output program, pins and data islands. What differs is the pixel: that
 * driver sends each index as a symbol and its complement (two output pixels,
 * for DC balance), so a line holds 320 indices. Here every palette channel is
 * snapped to a level whose TMDS symbol is DC-balanced by itself
 * (tmds_balanced.c), so each of the 640 pixels is one precomputed 64-bit
 * word pair and needs nothing after it.
 *
 * The output state machine runs at clk_sys / (clk_sys / 252 MHz): 10
 * instructions a pixel, 25.2 MHz. DMA channel "line" feeds its FIFO, 32 bits
 * at a time, in 5 blocks a line (800 pixels) that channel "ctrl" loads from
 * the line's block list:
 *
 *   0  front porch 8 + data preamble 8       16 px  linear (per VSYNC level)
 *   1  data island                           36 px  linear (null, or the AVI
 *                                                   info frame once a frame)
 *   2  hsync                                 60 px  one symbol repeated (ring)
 *   3  back porch 38 + video preamble 8      48 px  linear on active lines,
 *      + guard band 2                               blank repeated otherwise
 *   4  the pixels                           640 px  the line buffer, or blank
 *
 * The repeated blocks read one 8-byte-aligned symbol through an 8-byte read
 * ring. The IRQ at the end of block 3 (on core 1) points "ctrl" at the next
 * line's list, makes the list of the line after it, and expands the next
 * line's 640 indices into the other line buffer (pal_sym lookups) while this
 * line's pixels are being sent from the first.
 */

#include "hdmi_pio640.h"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hstx_packet.h"
#include "pico.h"
#include "tmds_balanced.h"

#ifndef HDMI_PIO_BASE_PIN
#define HDMI_PIO_BASE_PIN 32
#endif
/* Z0 wires the data lanes first, then the clock, BGR, the pairs not
 * inverted; the others (frank-msx boards) clock first, RGB, inverted */
#ifndef HDMI_PIO_Z0_LAYOUT
#define HDMI_PIO_Z0_LAYOUT 1
#endif
#if HDMI_PIO_Z0_LAYOUT
#define PIN_RGB_NOT_BGR 0
#define PIN_INVERT_DIFFPAIRS 0
#define PIN_DATA (HDMI_PIO_BASE_PIN)
#define PIN_CLK (HDMI_PIO_BASE_PIN + 6)
#else
#define PIN_RGB_NOT_BGR 1
#define PIN_INVERT_DIFFPAIRS 1
#define PIN_DATA (HDMI_PIO_BASE_PIN + 2)
#define PIN_CLK (HDMI_PIO_BASE_PIN)
#endif

#define PIO_VIDEO pio0
#define SM_VIDEO 0
#define DMA_CTRL 0
#define DMA_LINE 1

#define W 640
#define ACTIVE_LINES 480
#define TOTAL_LINES 525
#define VSYNC_FIRST 490 /* 490, 491 */
#define ISLAND 36       /* 2 guard, 32 packet, 2 guard */
#define HEAD 16
#define HSYNC 60
#define TAIL 48
#define NBLOCK 5

typedef struct {
    const void *read;
    volatile void *write;
    uint32_t count;
    uint32_t ctrl;
} block_t;

/* The palette as sent (SCRATCH_Y: read by the expansion, beside the DMA's
 * reads of the line buffers in the main SRAM) and the HDMI symbols. */
static uint64_t pal_sym[256] __attribute__((section(".scratch_y.hdmi")));
static uint64_t head[2][HEAD] __attribute__((section(".scratch_y.hdmi")));
static uint64_t tail_active[TAIL] __attribute__((section(".scratch_y.hdmi")));
static uint64_t isl_null[2][ISLAND] __attribute__((section(".scratch_y.hdmi")));
static uint64_t isl_avi[ISLAND] __attribute__((section(".scratch_y.hdmi")));
static uint64_t sym_blank[2] __attribute__((section(".scratch_y.hdmi")));
static uint64_t sym_hsync[2] __attribute__((section(".scratch_y.hdmi")));
static block_t blocks[2][NBLOCK] __attribute__((section(".scratch_y.hdmi"), aligned(16)));

/* two lines of symbols: one being sent, the next being made */
static uint64_t line_buf[2][W] __attribute__((aligned(8)));

static uint64_t stage[256];
static volatile int stage_pending;

static const uint8_t *frame;
static uint32_t ctrl_lin, ctrl_ring, ctrl_lin_irq, ctrl_ring_irq;

volatile uint32_t hdmi_pio640_frames;
volatile uint32_t hdmi_pio640_irq_max_us;
volatile uint32_t hdmi_pio640_errors;

/* TERC4 (data islands) and the control symbols by CTL bits (c1 << 1 | c0),
 * bit 0 sent first, as in hstx_packet.c and PicoDVI */
static const uint16_t terc4[16] = {0x29c, 0x263, 0x2e4, 0x2e2, 0x171, 0x11e, 0x18e, 0x13c,
                                   0x2cc, 0x139, 0x19c, 0x2c6, 0x28e, 0x271, 0x163, 0x2c3};
static const uint16_t ctl_sym[4] = {0x354, 0x0ab, 0x154, 0x2ab};
#define GUARD_DATA 0x133 /* lanes 1, 2 of a data island guard band */
#define GUARD_VIDEO_0 0x2cc
#define GUARD_VIDEO_1 0x133

/* One lane's 10-bit symbol as the output program shifts it: 10 steps of 6
 * bits (a 2-bit pair a lane, the second half of the word 2 bits on). Lane 0
 * is TMDS channel 0 (blue, the syncs), 1 green, 2 red. */
static uint64_t ser_lane(uint16_t sym, int lane) {
    int shift = lane == 1 ? 2 : (lane == 0) == (PIN_RGB_NOT_BGR != 0) ? 0 : 4;
    uint64_t out = 0;
    for (int i = 0; i < 10; i++) {
        out <<= 6;
        if (i == 5) out <<= 2;
        uint32_t b = (sym >> (9 - i)) & 1;
        b |= (b ^ 1) << 1;
        if (PIN_INVERT_DIFFPAIRS) b ^= 3;
        out |= (uint64_t)b << shift;
    }
    return out;
}

static uint64_t ser3(uint16_t l0, uint16_t l1, uint16_t l2) {
    return ser_lane(l0, 0) | ser_lane(l1, 1) | ser_lane(l2, 2);
}

/* A data island (36 symbols) from a packet: lane 0 carries the header and
 * the syncs (hv: VSYNC << 1 | HSYNC levels), lanes 1 and 2 the subpackets
 * (laid out as hstx_packet.c does for the HSTX). */
static void encode_island(uint64_t *d, const hstx_packet_t *p, uint32_t hv) {
    uint64_t terc_lane[3][16];
    for (int lane = 0; lane < 3; lane++)
        for (int n = 0; n < 16; n++) terc_lane[lane][n] = ser_lane(terc4[n], lane);
    uint64_t guard12 = ser_lane(GUARD_DATA, 1) | ser_lane(GUARD_DATA, 2);
    uint64_t g = terc_lane[0][0xc | hv] | guard12;
    d[0] = g;
    d[1] = g;
    d[34] = g;
    d[35] = g;
    for (int j = 0; j < 8; j++) {
        uint32_t v = p->subpacket[0][j] | p->subpacket[1][j] << 8 | p->subpacket[2][j] << 16 |
                     (uint32_t)p->subpacket[3][j] << 24;
        uint32_t x = (v ^ (v >> 7)) & 0x00aa00aa;
        v = v ^ x ^ (x << 7);
        x = (v ^ (v >> 14)) & 0x0000cccc;
        v = v ^ x ^ (x << 14);
        for (int k = 0; k < 4; k++) {
            int i = j * 4 + k;                   /* the packet's pixel */
            int sh = (k & 1) * 16 + (k >> 1) * 4; /* lane 1: 0, 16, 4, 20; lane 2: 8 more */
            uint32_t bit = (p->header[i >> 3] >> (i & 7)) & 1;
            uint32_t n0 = bit << 2 | hv | (i ? 8 : 0);
            d[2 + i] = terc_lane[0][n0] | terc_lane[1][(v >> sh) & 0xf] | terc_lane[2][(v >> (sh + 8)) & 0xf];
        }
    }
}

static inline void set_block(block_t *b, const void *read, uint32_t pixels, uint32_t ctrl) {
    b->read = read;
    b->write = &PIO_VIDEO->txf[SM_VIDEO];
    b->count = pixels * 2; /* two words a pixel */
    b->ctrl = ctrl;
}

/* the block list of a line, into slot (the lines' parity would not do: 525
 * a frame); an active line's pixels are line_buf[slot] */
static void __scratch_x("hdmi640") prepare_line(uint32_t line, uint32_t slot) {
    block_t *b = blocks[slot];
    int blank = line >= ACTIVE_LINES;
    int vs = line >= VSYNC_FIRST && line < VSYNC_FIRST + 2;

    set_block(&b[0], head[vs], HEAD, ctrl_lin);
    set_block(&b[1], line == ACTIVE_LINES + 1 ? isl_avi : isl_null[vs], ISLAND, ctrl_lin);
    set_block(&b[2], &sym_hsync[vs], HSYNC, ctrl_ring);
    if (!blank) {
        set_block(&b[3], tail_active, TAIL, ctrl_lin_irq);
        set_block(&b[4], line_buf[slot], W, ctrl_lin);
    } else {
        set_block(&b[3], &sym_blank[vs], TAIL, ctrl_ring_irq);
        set_block(&b[4], &sym_blank[vs], W, ctrl_ring);
    }
}

/* 640 indices to symbols: the IRQ's whole cost on an active line. Four
 * pixels a pass: four LDRDs from pal_sym, one STM of the eight words; in
 * integer registers (no FPU state in the IRQ). */
static void __scratch_x("hdmi640") expand_line(uint64_t *dst, const uint8_t *src) {
    const uint32_t *s = (const uint32_t *)src;
    const uint32_t *end = s + W / 4;
    const uint64_t *p = pal_sym;
    __asm volatile(
        "1:\n"
        "ldr    r3, [%[s]], #4\n"
        "uxtb   r6, r3\n"
        "ubfx   r8, r3, #8, #8\n"
        "ubfx   r10, r3, #16, #8\n"
        "lsr    r12, r3, #24\n"
        "add    r6, %[p], r6, lsl #3\n"
        "add    r8, %[p], r8, lsl #3\n"
        "add    r10, %[p], r10, lsl #3\n"
        "add    r12, %[p], r12, lsl #3\n"
        "ldrd   r6, r7, [r6]\n"
        "ldrd   r8, r9, [r8]\n"
        "ldrd   r10, r11, [r10]\n"
        "ldrd   r12, lr, [r12]\n"
        "stmia  %[d]!, {r6-r12, lr}\n"
        "cmp    %[s], %[e]\n"
        "bne    1b\n"
        : [s] "+r"(s), [d] "+r"(dst)
        : [p] "r"(p), [e] "r"(end)
        : "r3", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "lr", "cc", "memory");
}

static void __scratch_x("hdmi640") dma_irq(void) {
    static uint32_t line, slot; /* being sent: its block 3 is done */
    uint32_t t0 = timer_hw->timerawl;
    dma_hw->ints0 = 1u << DMA_LINE;
    /* "ctrl" is loading this line's last block: let it finish, then point
     * it at the next line's list */
    while (dma_hw->ch[DMA_CTRL].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) tight_loop_contents();
    dma_hw->ch[DMA_CTRL].read_addr = (uintptr_t)blocks[slot ^ 1];

    uint32_t next = line + 1 == TOTAL_LINES ? 0 : line + 1;
    uint32_t after = next + 1 == TOTAL_LINES ? 0 : next + 1;
    if (line == ACTIVE_LINES + 1) {
        hdmi_pio640_frames++;
        if (stage_pending) { /* the palette staged by core 0 */
            /* volatile: word copies, not FPU doubles, in the IRQ */
            volatile uint32_t *d = (volatile uint32_t *)pal_sym;
            const uint32_t *src = (const uint32_t *)stage;
            for (int i = 0; i < 512; i++) d[i] = src[i];
            stage_pending = 0;
        }
    }
    prepare_line(after, slot); /* this line's list: all loaded */
    if (next < ACTIVE_LINES) expand_line(line_buf[slot ^ 1], frame + next * W);

    /* an empty FIFO with the state machine waiting: an underrun */
    const uint32_t stall = 1u << (PIO_FDEBUG_TXSTALL_LSB + SM_VIDEO);
    if (PIO_VIDEO->fdebug & stall) {
        PIO_VIDEO->fdebug = stall;
        if (hdmi_pio640_frames > 1) hdmi_pio640_errors++;
    }
    line = next;
    slot ^= 1;
    uint32_t dt = timer_hw->timerawl - t0;
    if (dt > hdmi_pio640_irq_max_us) hdmi_pio640_irq_max_us = dt;
}

static uint64_t pixel_symbol(uint32_t rgb) {
    uint8_t r = tmds_balanced_snap((uint8_t)(rgb >> 16));
    uint8_t g = tmds_balanced_snap((uint8_t)(rgb >> 8));
    uint8_t b = tmds_balanced_snap((uint8_t)rgb);
    return ser3(tmds_balanced_symbol(b), tmds_balanced_symbol(g), tmds_balanced_symbol(r));
}

void hdmi_pio640_stage(int i, uint32_t rgb) {
    while (stage_pending) tight_loop_contents();
    stage[i & 0xff] = pixel_symbol(rgb);
}

void hdmi_pio640_commit(void) {
    __dmb();
    stage_pending = 1;
}

bool hdmi_pio640_pending(void) {
    return stage_pending != 0;
}

static void make_tables(void) {
    tmds_balanced_init();
    for (int i = 0; i < 256; i++) stage[i] = pal_sym[i] = pixel_symbol(0);
    for (int vs = 0; vs < 2; vs++) {
        uint32_t v = vs ? 0 : 2; /* VSYNC level (low when asserted) */
        sym_blank[vs] = ser3(ctl_sym[v | 1], ctl_sym[0], ctl_sym[0]);
        sym_hsync[vs] = ser3(ctl_sym[v], ctl_sym[0], ctl_sym[0]);
        uint64_t pre_data = ser3(ctl_sym[v | 1], ctl_sym[1], ctl_sym[1]);
        for (int i = 0; i < 8; i++) {
            head[vs][i] = sym_blank[vs];
            head[vs][8 + i] = pre_data;
        }
        hstx_packet_t null;
        hstx_packet_set_null(&null);
        encode_island(isl_null[vs], &null, v);
    }
    /* 640x480 is VIC 1 */
    hstx_packet_t avi;
    hstx_packet_set_avi_infoframe(&avi, 1, 0);
    encode_island(isl_avi, &avi, 2);
#if HDMI_PIO640_DVI
    /* plain DVI: no data islands, the head all blanking, the island's
     * pixels part of the sync pulse */
    for (int vs = 0; vs < 2; vs++) {
        for (int i = 0; i < HEAD; i++) head[vs][i] = sym_blank[vs];
        for (int i = 0; i < ISLAND; i++) isl_null[vs][i] = sym_hsync[vs];
    }
    for (int i = 0; i < ISLAND; i++) isl_avi[i] = sym_hsync[0];
#endif
    /* back porch 38, the video preamble 8, the guard band 2 */
    for (int i = 0; i < 38; i++) tail_active[i] = sym_blank[0];
    for (int i = 38; i < 46; i++) tail_active[i] = ser3(ctl_sym[3], ctl_sym[1], ctl_sym[0]);
    tail_active[46] = tail_active[47] = ser3(GUARD_VIDEO_0, GUARD_VIDEO_1, GUARD_VIDEO_0);
}

void hdmi_pio640_init(const uint8_t *fb) {
    frame = fb;
    make_tables();

    dma_channel_claim(DMA_CTRL);
    dma_channel_claim(DMA_LINE);
    pio_sm_claim(PIO_VIDEO, SM_VIDEO);
#if HDMI_PIO_BASE_PIN >= 16
    pio_set_gpio_base(PIO_VIDEO, 16); /* 0 or 16 only: GPIO 16..47 */
#endif
    /* the output: a pixel in 10 steps on the 3 data pairs, the clock by side-set */
    static const uint16_t video_instr[] = {
        0x7006, 0x7006, 0x7006, 0x7006, 0x7006, /* out pins, 6  side 2 */
        0x6806, 0x6806, 0x6806, 0x6806, 0x6806, /* out pins, 6  side 1 */
    };
    static const struct pio_program video_program = {.instructions = video_instr, .length = 10, .origin = -1};
    uint off_video = pio_add_program(PIO_VIDEO, &video_program);

    pio_sm_config cc = pio_get_default_sm_config();
    sm_config_set_wrap(&cc, off_video, off_video + video_program.length - 1);
    sm_config_set_sideset_pins(&cc, PIN_CLK);
    sm_config_set_sideset(&cc, 2, false, false);
    for (int i = 0; i < 2; i++) {
        pio_gpio_init(PIO_VIDEO, PIN_CLK + i);
        gpio_set_drive_strength(PIN_CLK + i, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(PIN_CLK + i, GPIO_SLEW_RATE_FAST);
    }
    for (int i = 0; i < 6; i++) {
        pio_gpio_init(PIO_VIDEO, PIN_DATA + i);
        gpio_set_drive_strength(PIN_DATA + i, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(PIN_DATA + i, GPIO_SLEW_RATE_FAST);
    }
    uint64_t clk_mask = (uint64_t)3u << PIN_CLK;
    pio_sm_set_pins_with_mask64(PIO_VIDEO, SM_VIDEO, clk_mask, clk_mask);
    pio_sm_set_pindirs_with_mask64(PIO_VIDEO, SM_VIDEO, clk_mask, clk_mask);
    pio_sm_set_consecutive_pindirs(PIO_VIDEO, SM_VIDEO, PIN_DATA, 6, true);
    sm_config_set_out_pins(&cc, PIN_DATA, 6);
    sm_config_set_out_shift(&cc, true, true, 30);
    sm_config_set_fifo_join(&cc, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&cc, (float)clock_get_hz(clk_sys) / 252000000.0f);
    pio_sm_init(PIO_VIDEO, SM_VIDEO, off_video, &cc);

    /* "line": the ctrl words its blocks load (32-bit words into the output
     * FIFO, chained to "ctrl"; the IRQ after block 3); "ring" repeats the
     * 8-byte symbol it starts at */
    dma_channel_config d = dma_channel_get_default_config(DMA_LINE);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, true);
    channel_config_set_write_increment(&d, false);
    channel_config_set_dreq(&d, pio_get_dreq(PIO_VIDEO, SM_VIDEO, true));
    channel_config_set_chain_to(&d, DMA_CTRL);
    channel_config_set_irq_quiet(&d, true);
    ctrl_lin = channel_config_get_ctrl_value(&d);
    channel_config_set_irq_quiet(&d, false);
    ctrl_lin_irq = channel_config_get_ctrl_value(&d);
    channel_config_set_ring(&d, false, 3);
    ctrl_ring_irq = channel_config_get_ctrl_value(&d);
    channel_config_set_irq_quiet(&d, true);
    ctrl_ring = channel_config_get_ctrl_value(&d);

    /* "ctrl": a block (4 words) into the registers of "line", the last
     * write (ctrl_trig) starts it */
    d = dma_channel_get_default_config(DMA_CTRL);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, true);
    channel_config_set_write_increment(&d, true);
    channel_config_set_ring(&d, true, 4);
    dma_channel_configure(DMA_CTRL, &d, &dma_hw->ch[DMA_LINE].read_addr, blocks[0], 4, false);

    prepare_line(0, 0);
    prepare_line(1, 1);
    expand_line(line_buf[0], frame);
}

void hdmi_pio640_start(void) {
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq);
    irq_set_priority(DMA_IRQ_0, 0);
    dma_channel_acknowledge_irq0(DMA_LINE);
    dma_channel_set_irq0_enabled(DMA_LINE, true);
    irq_set_enabled(DMA_IRQ_0, true);
    pio_sm_set_enabled(PIO_VIDEO, SM_VIDEO, true);
    dma_channel_start(DMA_CTRL);
}

void hdmi_pio640_stop(void) {
    irq_set_enabled(DMA_IRQ_0, false);
    dma_channel_set_irq0_enabled(DMA_LINE, false);
    /* "line" chains to "ctrl", which restarts "line": stop both, twice */
    for (int i = 0; i < 2; i++) {
        dma_channel_abort(DMA_CTRL);
        dma_channel_abort(DMA_LINE);
    }
    pio_sm_set_enabled(PIO_VIDEO, SM_VIDEO, false);
    pio_sm_clear_fifos(PIO_VIDEO, SM_VIDEO);
}
