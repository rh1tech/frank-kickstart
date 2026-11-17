/*
 * frank-kickstart - board configuration (M1 / M2 / Z0)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The pins come from CMakeLists.txt (BOARD_VARIANT); this header only keeps
 * the defaults and the run-time PSRAM chip-select choice.
 *
 *          HDMI              SD (SPI)              PS/2   NES (CLK/LATCH/DATA)
 *   M1     PIO, GPIO 6-13    SPI0 SCK2 MOSI3 MISO4 CS5    0/1   14/15/16
 *   M2     HSTX, GPIO 12-19  SPI0 SCK6 MOSI7 MISO4 CS5    2/3   20/21/26
 *   Z0     PIO, GPIO 32-39   SPI1 SCK30 MOSI31 MISO40 CS43 2/3  4/5/7
 *
 * PSRAM chip select: GPIO47 on an RP2350B; on an RP2350A GPIO19 (M1) or GPIO8 (M2).
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "hardware/structs/sysinfo.h"
#include "hardware/vreg.h"

#if !defined(BOARD_M1) && !defined(BOARD_M2) && !defined(BOARD_Z0)
#define BOARD_M2
#endif

#ifndef BOARD_NAME
#define BOARD_NAME "M2"
#endif

#ifndef CPU_CLOCK_MHZ
#define CPU_CLOCK_MHZ 252
#endif

#ifndef CPU_VOLTAGE
#define CPU_VOLTAGE VREG_VOLTAGE_1_50
#endif

#ifndef PSRAM_MAX_FREQ_MHZ
#define PSRAM_MAX_FREQ_MHZ 133
#endif

#ifndef PS2_PIN_CLK
#if defined(BOARD_M1)
#define PS2_PIN_CLK 0
#else
#define PS2_PIN_CLK 2
#endif
#endif
#ifndef PS2_PIN_DATA
#define PS2_PIN_DATA (PS2_PIN_CLK + 1)
#endif

#ifdef BOARD_M1
#define PSRAM_PIN_RP2350A 19
#else
#define PSRAM_PIN_RP2350A 8
#endif
#define PSRAM_PIN_RP2350B 47

// Run-time PSRAM pin, from the chip package
static inline uint get_psram_pin(void) {
    uint32_t package_sel = *((io_ro_32*)(SYSINFO_BASE + SYSINFO_PACKAGE_SEL_OFFSET));
    return (package_sel & 1) ? PSRAM_PIN_RP2350A : PSRAM_PIN_RP2350B;
}

#endif // BOARD_CONFIG_H
