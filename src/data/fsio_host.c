/*
 * frank-kickstart - fsio on stdio, for the host renderer (tools/host_render)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "fsio.h"

/* paths are "/kickstart/..."; the host maps "/" to $FK_SD (default "sdcard") */
static const char *root(void) {
    const char *r = getenv("FK_SD");
    return r ? r : "sdcard";
}

static void host_path(char *out, int n, const char *path) {
    snprintf(out, (size_t)n, "%s%s", root(), path);
}

bool fsio_mount(void) { return true; }
void fsio_unmount(void) {}

int fsio_read(const char *path, uint32_t offset, void *buf, int max) {
    char p[512];
    host_path(p, sizeof p, path);
    FILE *f = fopen(p, "rb");
    if (!f) return -1;
    fseek(f, (long)offset, SEEK_SET);
    int n = (int)fread(buf, 1, (size_t)max, f);
    fclose(f);
    return n;
}

bool fsio_write(const char *path, const void *buf, int n) {
    (void)path; (void)buf; (void)n;
    return true;
}

static DIR *d;
static char dpath[512];

bool fsio_dir_open(const char *path) {
    host_path(dpath, sizeof dpath, path);
    d = opendir(dpath);
    return d != NULL;
}

bool fsio_dir_next(char *name, int max, uint32_t *size, uint32_t *stamp) {
    struct dirent *e;
    while (d && (e = readdir(d))) {
        if (e->d_name[0] == '.') continue;
        char p[1024];
        struct stat st;
        snprintf(p, sizeof p, "%s/%s", dpath, e->d_name);
        if (stat(p, &st) || !S_ISREG(st.st_mode)) continue;
        strncpy(name, e->d_name, (size_t)max - 1);
        name[max - 1] = 0;
        if (size) *size = (uint32_t)st.st_size;
        if (stamp) *stamp = (uint32_t)st.st_mtime;
        return true;
    }
    return false;
}

void fsio_dir_close(void) {
    if (d) closedir(d);
    d = NULL;
}
