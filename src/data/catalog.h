/*
 * frank-kickstart - the firmware catalog: the .uf2 files in /uf2/<board>,
 * /uf2 and /kickstart (in that order of precedence), their X.xml metadata
 * and X.bmp cover art from /kickstart, kept in PSRAM. The scan is cached in
 * /kickstart/.catalog, reused while the folders are unchanged.
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FK_CATALOG_H
#define FK_CATALOG_H

#include <stdbool.h>
#include <stdint.h>

#define CAT_DIR      "/kickstart"
#define CAT_MAX      128
#define ART_MAX_W    240
#define ART_MAX_H    320
#define ART_NCOLORS  112 /* = ART_COLORS in src/ui/theme.h */

/* board bits (cat_entry_t.boards; 0 = not known) */
#define BOARD_BIT_M1 1
#define BOARD_BIT_M2 2
#define BOARD_BIT_Z0 4

typedef enum {
    UF2_OK = 0,
    UF2_UNREADABLE,  /* not a UF2 file */
    UF2_RP2040,      /* built for the RP2040 */
    UF2_RISCV,       /* RP2350 RISC-V image: the launcher's boot redirect is ARM-only */
    UF2_TOO_BIG,     /* reaches the launcher's flash area (0x10FBF000) */
} uf2_status_t;

typedef struct {
    char filename[64];
    char basename[60];
    char dir[12];       /* where the UF2 is: "/uf2/m2", "/uf2" or "/kickstart" */
    char title[64];
    char version[32];
    char author[64];
    char website[96];
    char controls[256];
    char description[1024];
    uint8_t boards;
    uint8_t status;     /* uf2_status_t (UF2_OK until cat_check() has looked) */
    bool checked;       /* the UF2 header has been read */
    bool has_art;       /* X.bmp exists */
    bool art_failed;    /* ... but could not be read */
    uint32_t image_bytes; /* flash the image takes */
} cat_entry_t;

/* Transparency: the BMP palette entry 0xFF00FF (magenta) is see-through.
 * Such a cover is drawn as a cut-out on the page, with a soft shadow made
 * from its shape (half resolution, ART_SHADOW_R pixels of blur around it). */
#define ART_KEY_RGB   0xFF00FFu
#define ART_SHADOW_R  6
#define ART_SHADOW_W  (ART_MAX_W / 2 + 2 * ART_SHADOW_R)
#define ART_SHADOW_H  (ART_MAX_H / 2 + 2 * ART_SHADOW_R)

typedef struct {
    uint16_t w, h;
    uint16_t ncolors;
    int16_t key;               /* the transparent index, -1 if none */
    uint32_t tint;             /* its dominant colour (0xRRGGBB), for the glow */
    uint32_t pal[ART_NCOLORS]; /* 0xRRGGBB */
    uint8_t px[ART_MAX_W * ART_MAX_H];
    uint16_t sw, sh;           /* the shadow: levels 0..15, half resolution */
    uint8_t shadow[ART_SHADOW_W * ART_SHADOW_H];
} art_t;

extern cat_entry_t *cat;
extern int cat_count;

/* PSRAM: the 8 MB at 0x11000000 (mandatory on every supported board).
 * Returns false if it does not answer. */
bool pmem_init(void);
void *pmem_alloc(uint32_t size);
void *pmem_scratch(uint32_t size);

/* Lists /kickstart and reads every entry's metadata and UF2 header; entries
 * are sorted by title. Returns the count. */
int cat_scan(void);

/* Index of the entry with this path ("/uf2/m2/nes.uf2") or, for an old
 * .last, file name; -1 if none. */
int cat_find(const char *path);

/* The entry's UF2 path. */
void cat_path(int i, char *out, int max);

/* Reads entry i's UF2 header (its status), once. */
void cat_check(int i);

/* The cover art of entry i, loaded on first use and cached (LRU); NULL if
 * the entry has none. */
const art_t *cat_art(int i);

/* This board's bit, and whether entry i says it is built for another board. */
uint8_t cat_board_bit(void);
bool cat_wrong_board(int i);

#endif
