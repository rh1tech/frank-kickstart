/*
 * frank-kickstart - platform helpers (reboot, UART debug commands)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdbool.h>

/* Core voltage, flash timing and clk_sys = CPU_CLOCK_MHZ (a multiple of
 * 252 MHz, for the 25.2 MHz pixel clock). Call first thing in main(). */
void platform_init_clocks(void);

/* Back to 150 MHz before flash programming (the ROM routines reset the
 * QMI timing; XIP must stay in spec without it). */
void platform_slow_clocks(void);

/* Every software reboot goes through here (see platform.c). The watchdog
 * scratch registers survive, so SCRATCH_MAGIC_BOOT still reaches before_main(). */
void __attribute__((noreturn)) platform_reboot(bool bootsel);

/* Feed every byte received on the debug console: "~BOOT~" reboots into
 * BOOTSEL, "~RST~" reboots. */
void platform_uart_cmd(int ch);

#endif
