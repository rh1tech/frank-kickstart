/*
 * frank-kickstart - writing a UF2 image from the SD card into the flash
 * (the sector-0 scheme from pico-launcher's m1p2-uf2 branch)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The image's sector 0 is kept in ZERO_BLOCK (marked with FLASH_MAGIC_OVER)
 * and written to the flash with its stack pointer and reset vector the
 * launcher's,
 * so every power-on starts the launcher; before_main() (fw_boot_redirect.c)
 * then jumps into the image when the watchdog scratch says so.
 *
 * The video runs on through it all: core 1 only runs from SRAM, and each
 * flash operation (frank-quake2's q2_assets_flash.c pattern) runs from RAM
 * with core 0's interrupts off, putting the launcher's QMI timing back
 * before anything is fetched from the flash again.
 */
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/structs/qmi.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "hardware/xip_cache.h"
#include "ff.h"
#include "flash_uf2.h"
#include "platform.h"

typedef struct {
    uint32_t magicStart0;
    uint32_t magicStart1;
    uint32_t flags;
    uint32_t targetAddr;
    uint32_t payloadSize;
    uint32_t blockNo;
    uint32_t numBlocks;
    uint32_t fileSize;
    uint8_t data[476];
    uint32_t magicEnd;
} uf2_block_t;

#define ZERO_BLOCK_ADDRESS (XIP_BASE + ZERO_BLOCK_OFFSET)

static uint8_t sector[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));

static void __no_inline_not_in_flash_func(flash_op)(uint32_t offs, const uint8_t *data, uint32_t size) {
    uint32_t timing = qmi_hw->m[0].timing, rcmd = qmi_hw->m[0].rcmd, rfmt = qmi_hw->m[0].rfmt;
    uint32_t s = save_and_disable_interrupts();
    if (data)
        flash_range_program(offs, data, size);
    else
        flash_range_erase(offs, size);
    qmi_hw->m[0].timing = timing;
    qmi_hw->m[0].rcmd = rcmd;
    qmi_hw->m[0].rfmt = rfmt;
    __dsb();
    __isb();
    restore_interrupts(s);
}

/* Writes one sector if it differs from what the flash holds. */
static void write_sector(uint32_t offs, const uint8_t *data) {
    if (!memcmp(data, (const void *)(XIP_BASE + offs), FLASH_SECTOR_SIZE)) return;
    /* the ROM drops the XIP cache: PSRAM writes still in it must land first */
    xip_cache_clean_all();
    flash_op(offs, NULL, FLASH_SECTOR_SIZE);
    flash_op(offs, data, FLASH_SECTOR_SIZE);
}

static void flush_sector(uint32_t base) {
    if (base == 0) {
        /* sector 0: the original into ZERO_BLOCK, the launcher's reset
         * vector into the flash copy */
        uint32_t *v = (uint32_t *)sector;
        uint32_t orig = v[1023];
        v[1023] = FLASH_MAGIC_OVER;
        write_sector(ZERO_BLOCK_OFFSET, sector);
        /* the launcher's stack (the boot ROM sets it from here: with the
         * firmware's, the launcher's stack ran into SCRATCH_Y) and entry */
        extern uint32_t __StackTop;
        v[0] = (uint32_t)&__StackTop;
        v[1] = *(const uint32_t *)(XIP_BASE + 4);
        v[1023] = orig;
    }
    write_sector(base, sector);
}

bool flash_uf2(const char *path, flash_progress_cb progress) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return false;
    uint32_t total = (uint32_t)(f_size(&f) / sizeof(uf2_block_t));
    uint32_t done = 0;
    int32_t base = -1;
    static uf2_block_t blk;
    UINT br;
    while (f_read(&f, &blk, sizeof blk, &br) == FR_OK && br == sizeof blk) {
        done++;
        if (progress && (done & 63) == 0) progress(done, total);
        if (blk.magicStart0 != 0x0A324655u || blk.magicStart1 != 0x9E5D5157u) continue;
        if (blk.flags & 0x1) continue; /* "not main flash" */
        uint32_t off = blk.targetAddr - XIP_BASE;
        /* nothing at or above ZERO_BLOCK: that is the launcher's (the
         * RP2350-E10 workaround block lands at 0x10FFFF00) */
        if (blk.targetAddr < XIP_BASE || off + blk.payloadSize > ZERO_BLOCK_OFFSET) continue;
        if (blk.payloadSize > 256 || (off & 255)) continue;
        uint32_t sbase = off & ~(FLASH_SECTOR_SIZE - 1);
        if ((int32_t)sbase != base) {
            if (base >= 0) flush_sector((uint32_t)base);
            base = (int32_t)sbase;
            memset(sector, 0xFF, sizeof sector);
        }
        memcpy(sector + (off - sbase), blk.data, blk.payloadSize);
    }
    if (base >= 0) flush_sector((uint32_t)base);
    f_close(&f);
    if (progress) progress(total, total);
    return base >= 0;
}

void flash_boot_into(void) {
    watchdog_hw->scratch[0] = SCRATCH_MAGIC_BOOT;
    platform_reboot(false);
}
