/*
 * frank-kickstart - the few file operations the catalog needs (FatFs on the
 * board, stdio on the host for tools/host_render)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FK_FSIO_H
#define FK_FSIO_H

#include <stdbool.h>
#include <stdint.h>

bool fsio_mount(void);
void fsio_unmount(void);

/* Up to max bytes of the file from offset; returns the count, -1 if absent. */
int fsio_read(const char *path, uint32_t offset, void *buf, int max);

/* Replaces the file with n bytes. */
bool fsio_write(const char *path, const void *buf, int n);

/* Directory listing: names of the regular files, one per call, with their
 * size and modification stamp (any value that changes when the file does). */
bool fsio_dir_open(const char *path);
bool fsio_dir_next(char *name, int max, uint32_t *size, uint32_t *stamp);
void fsio_dir_close(void);

#endif
