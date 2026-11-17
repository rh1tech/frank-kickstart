/*
 * frank-kickstart - fsio on FatFs (the SD card)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <string.h>
#include "ff.h"
#include "fsio.h"

static FATFS fs;
static DIR dir;

bool fsio_mount(void) {
    return f_mount(&fs, "", 1) == FR_OK;
}

void fsio_unmount(void) {
    f_unmount("");
}

int fsio_read(const char *path, uint32_t offset, void *buf, int max) {
    FIL f;
    UINT br = 0;
    if (f_open(&f, path, FA_READ) != FR_OK) return -1;
    if (offset && f_lseek(&f, offset) != FR_OK) {
        f_close(&f);
        return -1;
    }
    FRESULT r = f_read(&f, buf, (UINT)max, &br);
    f_close(&f);
    return r == FR_OK ? (int)br : -1;
}

bool fsio_write(const char *path, const void *buf, int n) {
    FIL f;
    UINT bw = 0;
    if (f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return false;
    FRESULT r = f_write(&f, buf, (UINT)n, &bw);
    f_close(&f);
    return r == FR_OK && (int)bw == n;
}

bool fsio_dir_open(const char *path) {
    return f_opendir(&dir, path) == FR_OK;
}

bool fsio_dir_next(char *name, int max, uint32_t *size, uint32_t *stamp) {
    FILINFO fno;
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        if (fno.fattrib & (AM_DIR | AM_HID | AM_SYS)) continue;
        strncpy(name, fno.fname, (size_t)max - 1);
        name[max - 1] = 0;
        if (size) *size = (uint32_t)fno.fsize;
        if (stamp) *stamp = (uint32_t)fno.fdate << 16 | fno.ftime;
        return true;
    }
    return false;
}

void fsio_dir_close(void) {
    f_closedir(&dir);
}
