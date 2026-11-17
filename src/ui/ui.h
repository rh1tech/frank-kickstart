/*
 * frank-kickstart - the launcher screens
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FK_UI_H
#define FK_UI_H

#include <stdbool.h>
#include <stdint.h>

/* The entry flashed last (shown as installed; launching it only reboots), -1. */
extern int ui_installed;

/* Off-screen buffers in PSRAM (after pmem_init). */
void ui_init(void);

/* The animated welcome screen; returns on a key (input_poll) or after
 * ~9 s, faded to black. */
void ui_welcome(void);

/* The firmware browser, entry `sel`, faded in from black. */
void ui_browser_show(int sel);

/* Animated move to entry `to` (dir -1: came from the right, +1: from the left). */
void ui_browser_move(int to, int dir);

/* Footer hint for a second press (wrong board), or back to normal. */
void ui_browser_confirm(int sel, bool confirm);

/* The footer drawn again (the music hint changed). */
void ui_browser_footer(int sel, bool confirm);

/* A full-screen message (fades in); line2 may be NULL. */
void ui_message(const char *title, const char *line1, const char *line2);

/* The install screen for entry sel, then progress updates. */
void ui_flash_begin(int sel, bool reboot_only);
void ui_flash_progress(uint32_t done, uint32_t total);

/* The start screen after a reset: "Loading <entry sel>" (-1: unknown) and
 * a countdown of `seconds`; true if a key was pressed (open the menu),
 * false when it ran out. Either way it ends faded to black. */
bool ui_countdown(int sel, int seconds);

/* Fades the whole screen to black. */
void ui_fade_out(void);

#endif
