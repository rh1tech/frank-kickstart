/*
 * frank-kickstart - DC-balanced TMDS data symbols (see tmds_balanced.h)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "tmds_balanced.h"

static uint8_t snap_tab[256];
static uint16_t sym_tab[256];

/* the control tokens: a data symbol must never look like one */
static const uint16_t ctl_tokens[4] = {0x354, 0x0ab, 0x154, 0x2ab};

/* q_m[8:0] of d in the XOR (xnor = 0) or XNOR form */
static uint32_t tmds_qm(uint32_t d, int xnor) {
    uint32_t q = d & 1;
    for (int i = 1; i < 8; i++) {
        uint32_t b = ((q >> (i - 1)) ^ (d >> i)) & 1;
        if (xnor) b ^= 1;
        q |= b << i;
    }
    return q | (uint32_t)(xnor ? 0 : 1) << 8;
}

static int ones8(uint32_t v) {
    int n = 0;
    for (int i = 0; i < 8; i++) n += (v >> i) & 1;
    return n;
}

/* the balanced 10-bit symbol of a q_m: bits 7:0 inverted when q_m[8] is 0 */
static uint16_t tmds_out(uint32_t qm) {
    uint32_t q8 = (qm >> 8) & 1;
    uint32_t low = q8 ? (qm & 0xff) : (~qm & 0xff);
    return (uint16_t)(low | q8 << 8 | (q8 ^ 1) << 9);
}

static uint16_t balanced(uint32_t d) {
    /* Only the form the DVI rule picks. The other one decodes too, but it
     * does not minimise transitions: those symbols carry up to 9 (control
     * tokens have 7..9), and receivers lose the word alignment on them
     * (seen: no signal at all on a capture card). 52 levels, 16..239. */
    int n1 = ones8(d);
    int xnor = n1 > 4 || (n1 == 4 && !(d & 1));
    uint32_t qm = tmds_qm(d, xnor);
    if (ones8(qm) != 4) return 0;
    uint16_t s = tmds_out(qm);
    for (int i = 0; i < 4; i++)
        if (s == ctl_tokens[i]) return 0;
    return s;
}

void tmds_balanced_init(void) {
    for (int d = 0; d < 256; d++) sym_tab[d] = balanced((uint32_t)d);
    for (int v = 0; v < 256; v++) {
        int best = -1;
        for (int dist = 0; dist < 256 && best < 0; dist++) {
            if (v - dist >= 0 && sym_tab[v - dist]) best = v - dist;
            else if (v + dist < 256 && sym_tab[v + dist]) best = v + dist;
        }
        snap_tab[v] = (uint8_t)best;
    }
}

uint8_t tmds_balanced_snap(uint8_t v) {
    return snap_tab[v];
}

uint16_t tmds_balanced_symbol(uint8_t level) {
    return sym_tab[level];
}
