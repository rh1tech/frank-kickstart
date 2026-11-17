/*
 * frank-kickstart - DC-balanced TMDS data symbols
 *
 * A TMDS data symbol whose q_m[7:0] has four ones carries five ones in its
 * ten bits: it leaves the running disparity where it was, so it is the same
 * symbol whatever came before. Sending only such symbols, every pixel is
 * encoded on its own (a palette lookup) and needs no complement after it.
 * Only the form the DVI rule picks is used: the other one would decode as
 * well, but its symbols are not transition-minimised and receivers lose
 * the word alignment on them. 52 of the 256 levels have such a symbol,
 * 16..239, in pairs one apart (about 26 distinct steps); colours are
 * snapped to those.
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef TMDS_BALANCED_H
#define TMDS_BALANCED_H

#include <stdint.h>

/* Builds the tables (call once before the others). */
void tmds_balanced_init(void);

/* The nearest level that has a balanced symbol. */
uint8_t tmds_balanced_snap(uint8_t v);

/* The 10-bit symbol (bit 0 sent first) for a level that tmds_balanced_snap
 * returned; 0 for a level without one. */
uint16_t tmds_balanced_symbol(uint8_t level);

#endif
