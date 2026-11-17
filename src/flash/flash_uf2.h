/*
 * frank-kickstart - writing a UF2 image from the SD card into the flash
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FK_FLASH_UF2_H
#define FK_FLASH_UF2_H

#include <stdbool.h>
#include <stdint.h>

#define ZERO_BLOCK_OFFSET   ((16ul << 20) - (256ul << 10) - (4ul << 10))
#define FLASH_MAGIC_OVER    0x3836d91au
#define SCRATCH_MAGIC_BOOT  0x383da910u
#define SCRATCH_MAGIC_UI    0x17F00FFFu

/* Set before main() when a firmware asked for the launcher (fw_boot_redirect.c). */
extern bool boot_ui_requested;

typedef void (*flash_progress_cb)(uint32_t done, uint32_t total);

/* Writes path (a UF2 for the RP2350) into the user area of the flash; the
 * video keeps running on core 1 (all its code and data are in SRAM).
 * Returns false if the file cannot be read; on success the caller reboots
 * into it (flash_boot_into). */
bool flash_uf2(const char *path, flash_progress_cb progress);

/* Reboots into the flashed firmware (the launcher's before_main() jumps to
 * it from ZERO_BLOCK). */
void __attribute__((noreturn)) flash_boot_into(void);

#endif
