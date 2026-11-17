/*
 * frank-kickstart - input: NES/SNES pad, PS/2 and USB keyboards, USB
 * gamepads and the debug UART, merged into one button state
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FK_INPUT_H
#define FK_INPUT_H

#include <stdint.h>

#define IN_LEFT  0x01
#define IN_RIGHT 0x02
#define IN_UP    0x04
#define IN_DOWN  0x08
#define IN_OK    0x10   /* A, Start, Enter */
#define IN_BACK  0x20   /* B, Esc */
#define IN_BOOTSEL_COMBO 0x40 /* Select+Start held: BOOTSEL after 3 s (main.c) */
#define IN_MUSIC 0x80   /* M: music on/off */

void input_init(void);

/* The buttons held now (polls every source; feeds UART bytes to
 * platform_uart_cmd). */
uint32_t input_poll(void);

#endif
