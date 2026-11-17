/*
 * frank-kickstart - the firmware catalog
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "catalog.h"
#include "fsio.h"
#ifndef FK_HOST
#include "pico/time.h"
#endif

cat_entry_t *cat;
int cat_count;

/* ------------------------------------------------------------------ */
/* PSRAM                                                               */
/* ------------------------------------------------------------------ */

#define PSRAM_SIZE (8u * 1024 * 1024)

#ifdef FK_HOST
static uint8_t *psram_base;
bool pmem_init(void) {
    psram_base = calloc(1, PSRAM_SIZE);
    return psram_base != NULL;
}
#else
#define psram_base ((uint8_t *)0x11000000)
bool pmem_init(void) {
    volatile uint8_t *p = psram_base;
    volatile uint32_t *w = (volatile uint32_t *)psram_base;
    p[0] = 0xA5; p[1] = 0x5A; p[2] = 0x42;
    if (p[0] != 0xA5 || p[1] != 0x5A || p[2] != 0x42) return false;
    /* the last word too: an 8 MB chip, not a smaller one aliasing */
    w[PSRAM_SIZE / 4 - 1] = 0x12345678u;
    w[0] = 0;
    return w[PSRAM_SIZE / 4 - 1] == 0x12345678u && w[0] == 0;
}
#endif

static uint32_t pmem_used;

void *pmem_alloc(uint32_t size) {
    size = (size + 15) & ~15u;
    if (pmem_used + size > PSRAM_SIZE) return NULL;
    void *p = psram_base + pmem_used;
    pmem_used += size;
    memset(p, 0, size);
    return p;
}

/* A scratch buffer in PSRAM, reused (grown when asked for more). */
void *pmem_scratch(uint32_t size) {
    static uint8_t *buf;
    static uint32_t cap;
    if (size > cap) {
        buf = pmem_alloc(size);
        cap = buf ? size : 0;
    }
    return buf;
}

/* ------------------------------------------------------------------ */
/* Boards                                                              */
/* ------------------------------------------------------------------ */

uint8_t cat_board_bit(void) {
#if defined(BOARD_M1)
    return BOARD_BIT_M1;
#elif defined(BOARD_Z0)
    return BOARD_BIT_Z0;
#else
    return BOARD_BIT_M2;
#endif
}

bool cat_wrong_board(int i) {
    return cat[i].boards && !(cat[i].boards & cat_board_bit());
}

/* Board names in a release file name or a <boards> list: "frank-nes_m2p2_hstx",
 * "m1 m2 z0". Tokens are split on anything that is not a letter or digit;
 * m1/m1p2 (Pico 2 builds) count, m1p1 (RP2040) does not. */
static uint8_t boards_in(const char *s) {
    uint8_t b = 0;
    char tok[16];
    while (*s) {
        while (*s && !isalnum((unsigned char)*s)) s++;
        int n = 0;
        while (*s && isalnum((unsigned char)*s)) {
            if (n < (int)sizeof(tok) - 1) tok[n++] = (char)tolower((unsigned char)*s);
            s++;
        }
        tok[n] = 0;
        if (n < 2 || tok[0] == 0) continue;
        uint8_t bit = 0;
        if (!strncmp(tok, "m1", 2)) bit = BOARD_BIT_M1;
        else if (!strncmp(tok, "m2", 2)) bit = BOARD_BIT_M2;
        else if (!strncmp(tok, "z0", 2)) bit = BOARD_BIT_Z0;
        if (bit && (tok[2] == 0 || !strcmp(tok + 2, "p2"))) b |= bit;
    }
    return b;
}

/* ------------------------------------------------------------------ */
/* XML metadata                                                        */
/* ------------------------------------------------------------------ */

/* The text of <tag>...</tag>, entities decoded, white space runs
 * (line breaks, indentation) collapsed to one space. */
static void xml_extract(const char *xml, const char *tag, char *out, int max) {
    char open[32], close[32];
    out[0] = 0;
    snprintf(open, sizeof open, "<%s>", tag);
    snprintf(close, sizeof close, "</%s>", tag);
    const char *s = strstr(xml, open);
    if (!s) return;
    s += strlen(open);
    const char *e = strstr(s, close);
    if (!e) return;
    int n = 0;
    bool space = true; /* drops leading white space */
    while (s < e && n < max - 1) {
        char c = *s++;
        if (c == '&') {
            const char *semi = memchr(s, ';', (size_t)(e - s));
            if (semi && semi - s <= 8) {
                long cp = -1;
                if (!strncmp(s, "amp;", 4)) cp = '&';
                else if (!strncmp(s, "lt;", 3)) cp = '<';
                else if (!strncmp(s, "gt;", 3)) cp = '>';
                else if (!strncmp(s, "quot;", 5)) cp = '"';
                else if (!strncmp(s, "apos;", 5)) cp = '\'';
                else if (s[0] == '#') cp = (s[1] == 'x') ? strtol(s + 2, NULL, 16) : strtol(s + 1, NULL, 10);
                if (cp > 0) {
                    s = semi + 1;
                    /* to UTF-8 */
                    if (cp < 0x80) { out[n++] = (char)cp; }
                    else if (cp < 0x800 && n < max - 2) { out[n++] = (char)(0xC0 | cp >> 6); out[n++] = (char)(0x80 | (cp & 0x3F)); }
                    else if (cp < 0x10000 && n < max - 3) { out[n++] = (char)(0xE0 | cp >> 12); out[n++] = (char)(0x80 | ((cp >> 6) & 0x3F)); out[n++] = (char)(0x80 | (cp & 0x3F)); }
                    space = false;
                    continue;
                }
            }
        }
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            if (!space) out[n++] = ' ';
            space = true;
            continue;
        }
        out[n++] = c;
        space = false;
    }
    while (n > 0 && out[n - 1] == ' ') n--;
    out[n] = 0;
}

static void load_metadata(cat_entry_t *e, bool has_xml) {
    static char xml[4096];
    char path[160];
    snprintf(path, sizeof path, CAT_DIR "/%s.xml", e->basename);
    int n = has_xml ? fsio_read(path, 0, xml, sizeof xml - 1) : -1;
    if (n > 0) {
        xml[n] = 0;
        xml_extract(xml, "title", e->title, sizeof e->title);
        xml_extract(xml, "version", e->version, sizeof e->version);
        xml_extract(xml, "author", e->author, sizeof e->author);
        xml_extract(xml, "website", e->website, sizeof e->website);
        xml_extract(xml, "description", e->description, sizeof e->description);
        xml_extract(xml, "controls", e->controls, sizeof e->controls);
        char boards[64];
        xml_extract(xml, "boards", boards, sizeof boards);
        if (boards[0]) e->boards = boards_in(boards);
    }
    if (!e->title[0]) strncpy(e->title, e->basename, sizeof e->title - 1);
}

/* ------------------------------------------------------------------ */
/* UF2 header                                                          */
/* ------------------------------------------------------------------ */

#define UF2_MAGIC0 0x0A324655u
#define UF2_MAGIC1 0x9E5D5157u
#define UF2_FLAG_FAMILY 0x00002000u
#define FAMILY_RP2040 0xE48BFF56u
#define FAMILY_ABSOLUTE 0xE48BFF57u /* the RP2350-E10 workaround block */
#define FAMILY_RP2350_ARM_S 0xE48BFF59u
#define FAMILY_RP2350_RISCV 0xE48BFF5Au
#define FAMILY_RP2350_ARM_NS 0xE48BFF5Bu
#define LAUNCHER_AREA 0x10FBF000u /* ZERO_BLOCK, then the launcher */

static void check_uf2(cat_entry_t *e) {
    static uint32_t blocks[256];
    char path[160];
    snprintf(path, sizeof path, "%s/%s", e->dir, e->filename);
    e->status = UF2_UNREADABLE;
    e->checked = true;
    /* the first two blocks in one read: the first may be the E10 one */
    int got = fsio_read(path, 0, blocks, sizeof blocks);
    for (uint32_t off = 0; off < 2 * 512; off += 512) {
        if (got < (int)(off + 512)) return;
        uint32_t *blk = blocks + off / 4;
        if (blk[0] != UF2_MAGIC0 || blk[1] != UF2_MAGIC1) return;
        uint32_t family = (blk[2] & UF2_FLAG_FAMILY) ? blk[7] : 0;
        if (family == FAMILY_ABSOLUTE) continue;
        uint32_t addr = blk[3], nblocks = blk[6];
        e->image_bytes = nblocks * 256;
        if (family == FAMILY_RP2040) e->status = UF2_RP2040;
        else if (family == FAMILY_RP2350_RISCV) e->status = UF2_RISCV;
        else if (addr + e->image_bytes > LAUNCHER_AREA) e->status = UF2_TOO_BIG;
        else e->status = UF2_OK;
        return;
    }
}

/* ------------------------------------------------------------------ */
/* Scan                                                                */
/* ------------------------------------------------------------------ */

static int cmp_title(const void *a, const void *b) {
    return strcasecmp(((const cat_entry_t *)a)->title, ((const cat_entry_t *)b)->title);
}

/* ---- the scan, and its cache ---- */

#define CACHE_FILE  CAT_DIR "/.catalog"
#define CACHE_MAGIC 0x31434B46u /* "FKC1" */
#define MAX_FILES   1024

typedef struct {
    uint32_t magic, sig, count, entry_size;
} cache_hdr_t;

typedef struct {
    char name[64];
} fname_t;

static fname_t *meta_names;  /* the .xml and .bmp files in /kickstart */
static int meta_count;

static uint32_t fnv(uint32_t h, const void *p, int n) {
    const uint8_t *b = p;
    while (n--) h = (h ^ *b++) * 16777619u;
    return h;
}

static bool has_meta(const char *base, const char *ext) {
    char want[64];
    snprintf(want, sizeof want, "%s%s", base, ext);
    for (int i = 0; i < meta_count; i++)
        if (!strcasecmp(meta_names[i].name, want)) return true;
    return false;
}

static int find_base(const char *base) {
    for (int i = 0; i < cat_count; i++)
        if (!strcasecmp(cat[i].basename, base)) return i;
    return -1;
}

/* One folder's UF2s into the catalogue (a name already there from a folder
 * of higher precedence is skipped), its .xml/.bmp names noted; every name,
 * size and stamp goes into the signature. */
static uint32_t list_dir(const char *dir, bool meta, uint32_t sig) {
    if (!fsio_dir_open(dir)) return fnv(sig, dir, (int)strlen(dir));
    char name[96];
    uint32_t size, stamp;
    while (fsio_dir_next(name, sizeof name, &size, &stamp)) {
        int len = (int)strlen(name);
        if (len < 5) continue;
        const char *ext = name + len - 4;
        bool uf2 = !strcasecmp(ext, ".uf2");
        bool xmlbmp = meta && (!strcasecmp(ext, ".xml") || !strcasecmp(ext, ".bmp"));
        if (!uf2 && !xmlbmp) continue;
        sig = fnv(sig, dir, (int)strlen(dir));
        sig = fnv(sig, name, len);
        sig = fnv(sig, &size, 4);
        sig = fnv(sig, &stamp, 4);
        if (xmlbmp) {
            if (meta_count < MAX_FILES && len < (int)sizeof meta_names[0].name)
                strcpy(meta_names[meta_count++].name, name);
            continue;
        }
        if (cat_count >= CAT_MAX || len >= (int)sizeof cat[0].filename) continue;
        char base[64];
        memcpy(base, name, (size_t)(len - 4));
        base[len - 4] = 0;
        if (len - 4 >= (int)sizeof cat[0].basename || find_base(base) >= 0) continue;
        cat_entry_t *e = &cat[cat_count++];
        memset(e, 0, sizeof *e);
        strcpy(e->filename, name);
        strcpy(e->basename, base);
        strncpy(e->dir, dir, sizeof e->dir - 1);
    }
    fsio_dir_close();
    return sig;
}

int cat_scan(void) {
    if (!cat) cat = pmem_alloc(sizeof(cat_entry_t) * CAT_MAX);
    if (!meta_names) meta_names = pmem_alloc(sizeof(fname_t) * MAX_FILES);
    cat_count = 0;
    meta_count = 0;
    if (!cat || !meta_names) return 0;
    static const char *const board_dir[] = {"", "/uf2/m1", "/uf2/m2", "", "/uf2/z0"};
    uint32_t sig = 2166136261u;
    sig = list_dir(board_dir[cat_board_bit()], false, sig);
    sig = list_dir("/uf2", false, sig);
    sig = list_dir(CAT_DIR, true, sig);
    sig = fnv(sig, &cat_count, 4);

    /* unchanged since the last scan: the cached entries */
    cache_hdr_t h;
    if (fsio_read(CACHE_FILE, 0, &h, sizeof h) == sizeof h && h.magic == CACHE_MAGIC && h.sig == sig &&
        h.count == (uint32_t)cat_count && h.entry_size == sizeof(cat_entry_t)) {
        int n = (int)(h.count * sizeof(cat_entry_t));
        if (fsio_read(CACHE_FILE, sizeof h, cat, n) == n) {
            for (int i = 0; i < cat_count; i++) cat[i].checked = false, cat[i].status = UF2_OK, cat[i].art_failed = false;
            return cat_count;
        }
    }

    for (int i = 0; i < cat_count; i++) {
        cat_entry_t *e = &cat[i];
        e->boards = boards_in(e->basename);
        e->has_art = has_meta(e->basename, ".bmp");
        load_metadata(e, has_meta(e->basename, ".xml"));
    }
    qsort(cat, (size_t)cat_count, sizeof cat[0], cmp_title);

    /* the cache: header + entries in one write */
    int n = (int)(sizeof h + (size_t)cat_count * sizeof(cat_entry_t));
    uint8_t *buf = pmem_scratch(n);
    if (buf) {
        h.magic = CACHE_MAGIC;
        h.sig = sig;
        h.count = (uint32_t)cat_count;
        h.entry_size = sizeof(cat_entry_t);
        memcpy(buf, &h, sizeof h);
        memcpy(buf + sizeof h, cat, (size_t)cat_count * sizeof(cat_entry_t));
        fsio_write(CACHE_FILE, buf, n);
    }
    return cat_count;
}

void cat_path(int i, char *out, int max) {
    snprintf(out, (size_t)max, "%s/%s", cat[i].dir, cat[i].filename);
}

void cat_check(int i) {
    if (i >= 0 && i < cat_count && !cat[i].checked) check_uf2(&cat[i]);
}

int cat_find(const char *path) {
    char p[160];
    for (int i = 0; i < cat_count; i++) {
        cat_path(i, p, sizeof p);
        if (!strcmp(p, path) || !strcmp(cat[i].filename, path)) return i;
    }
    return -1;
}

/* ------------------------------------------------------------------ */
/* Cover art                                                           */
/* ------------------------------------------------------------------ */

#define ART_SLOTS 16

static art_t *slot_art[ART_SLOTS];
static int slot_entry[ART_SLOTS];
static uint32_t slot_used[ART_SLOTS];
static uint32_t use_clock;

static inline uint32_t rd16(const uint8_t *p) { return p[0] | p[1] << 8; }
static inline uint32_t rd32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

static int dist2(uint32_t a, uint32_t b) {
    int dr = (int)((a >> 16) & 255) - (int)((b >> 16) & 255);
    int dg = (int)((a >> 8) & 255) - (int)((b >> 8) & 255);
    int db = (int)(a & 255) - (int)(b & 255);
    /* weighted towards green, as the eye is */
    return 3 * dr * dr + 4 * dg * dg + 2 * db * db;
}

/* Fewer than ART_NCOLORS colours: merges the closest pair of the used
 * colours (weighted by pixel count) until they fit, then remaps. The
 * transparent index (key, -1: none) is kept out of it and comes last. */
static void reduce_colors(art_t *a, const uint32_t *src_pal, int nsrc, int key) {
    const int max = key >= 0 ? ART_NCOLORS - 1 : ART_NCOLORS;
    static uint32_t count[256], col[256];
    static int map[256];
    memset(count, 0, sizeof count);
    int npx = a->w * a->h;
    for (int i = 0; i < npx; i++) count[a->px[i]]++;
    if (key >= 0) count[key] = 0;
    int n = 0;
    static int idx_of[256];
    for (int i = 0; i < nsrc; i++) {
        idx_of[i] = -1;
        if (count[i]) { col[n] = src_pal[i]; idx_of[i] = n; count[n] = count[i]; n++; }
    }
    for (int i = 0; i < n; i++) map[i] = i;
    /* map[] chases merged colours to their survivor */
    while (n > max) {
        int best = 0x7FFFFFFF, bi = 0, bj = 1;
        for (int i = 0; i < n; i++) {
            if (map[i] != i) continue;
            for (int j = i + 1; j < n; j++) {
                if (map[j] != j) continue;
                /* merging two rare colours costs less than two common ones */
                int64_t d = (int64_t)dist2(col[i], col[j]) *
                            (int64_t)(count[i] < count[j] ? count[i] : count[j]);
                int dd = d > 0x7FFFFFFF ? 0x7FFFFFFF : (int)d;
                if (dd < best) { best = dd; bi = i; bj = j; }
            }
        }
        /* bj into bi, colour weighted by count */
        uint32_t ci = col[bi], cj = col[bj], wi = count[bi], wj = count[bj], w = wi + wj;
        uint32_t r = (((ci >> 16) & 255) * wi + ((cj >> 16) & 255) * wj) / w;
        uint32_t g = (((ci >> 8) & 255) * wi + ((cj >> 8) & 255) * wj) / w;
        uint32_t b = ((ci & 255) * wi + (cj & 255) * wj) / w;
        col[bi] = r << 16 | g << 8 | b;
        count[bi] = w;
        map[bj] = bi;
        int live = 0;
        for (int i = 0; i < n; i++) live += map[i] == i;
        if (live <= max) break;
    }
    /* compact the survivors */
    static int final_idx[256];
    int m = 0;
    for (int i = 0; i < n; i++) {
        int r = i;
        while (map[r] != r) r = map[r];
        map[i] = r;
    }
    for (int i = 0; i < n; i++)
        if (map[i] == i) { final_idx[i] = m; a->pal[m++] = col[i]; }
    a->key = -1;
    if (key >= 0) {
        a->key = (int16_t)m;
        a->pal[m++] = ART_KEY_RGB;
    }
    a->ncolors = (uint16_t)m;
    static uint8_t lut[256];
    for (int i = 0; i < nsrc; i++) lut[i] = idx_of[i] >= 0 ? (uint8_t)final_idx[map[idx_of[i]]] : 0;
    if (key >= 0) lut[key] = (uint8_t)a->key;
    for (int i = 0; i < npx; i++) a->px[i] = lut[a->px[i]];
}

/* The shadow of a cut-out: its shape at half resolution, blurred (two box
 * passes each way), as levels 0..15. */
static void make_shadow(art_t *a) {
    const int R = ART_SHADOW_R;
    int hw = (a->w + 1) / 2, hh = (a->h + 1) / 2;
    int sw = hw + 2 * R, sh = hh + 2 * R;
    a->sw = (uint16_t)sw;
    a->sh = (uint16_t)sh;
    static uint16_t buf[2][ART_SHADOW_W * ART_SHADOW_H];
    uint16_t *m = buf[0], *t = buf[1];
    memset(m, 0, sizeof buf[0]);
    for (int y = 0; y < hh; y++)
        for (int x = 0; x < hw; x++) {
            int n = 0;
            for (int dy = 0; dy < 2; dy++)
                for (int dx = 0; dx < 2; dx++) {
                    int sx = 2 * x + dx, sy = 2 * y + dy;
                    if (sx < a->w && sy < a->h && a->px[sy * a->w + sx] != a->key) n++;
                }
            m[(y + R) * sw + x + R] = (uint16_t)(n * 64); /* 0..256 */
        }
    const int r = R / 2;
    for (int pass = 0; pass < 2; pass++) {
        for (int y = 0; y < sh; y++)
            for (int x = 0; x < sw; x++) {
                int sum = 0;
                for (int k = -r; k <= r; k++) {
                    int xx = x + k;
                    if (xx >= 0 && xx < sw) sum += m[y * sw + xx];
                }
                t[y * sw + x] = (uint16_t)(sum / (2 * r + 1));
            }
        for (int y = 0; y < sh; y++)
            for (int x = 0; x < sw; x++) {
                int sum = 0;
                for (int k = -r; k <= r; k++) {
                    int yy = y + k;
                    if (yy >= 0 && yy < sh) sum += t[yy * sw + x];
                }
                m[y * sw + x] = (uint16_t)(sum / (2 * r + 1));
            }
    }
    for (int i = 0; i < sw * sh; i++) a->shadow[i] = (uint8_t)(m[i] * 11 / 256); /* strength 11 */
}

/* The colour the cover is "about": its pixels averaged, each weighted by
 * how saturated and bright it is (greys and black count for little), then
 * brought up to full brightness. A grey cover gets the accent violet. */
static uint32_t dominant_colour(const art_t *a) {
    static uint32_t count[ART_NCOLORS];
    memset(count, 0, sizeof count);
    for (int i = 0; i < a->w * a->h; i += 2) count[a->px[i]]++;
    uint64_t sr = 0, sg = 0, sb = 0, sw = 0;
    for (int c = 0; c < a->ncolors; c++) {
        if (c == a->key || !count[c]) continue;
        int r = (a->pal[c] >> 16) & 255, g = (a->pal[c] >> 8) & 255, b = a->pal[c] & 255;
        int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
        int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
        uint64_t w = (uint64_t)count[c] * (uint64_t)((mx - mn) * (mx - mn)) * (uint64_t)mx;
        sr += w * r; sg += w * g; sb += w * b; sw += w;
    }
    if (sw < (uint64_t)a->w * a->h * 2000u) return 0xA97CFFu;
    uint32_t r = (uint32_t)(sr / sw), g = (uint32_t)(sg / sw), b = (uint32_t)(sb / sw);
    uint32_t mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    if (!mx) return 0xA97CFFu;
    r = r * 255 / mx; g = g * 255 / mx; b = b * 255 / mx;
    return r << 16 | g << 8 | b;
}

static bool load_bmp(int i, art_t *a) {
    static uint8_t hdr[54 + 1024];
    char path[160];
    snprintf(path, sizeof path, CAT_DIR "/%s.bmp", cat[i].basename);
    int n = fsio_read(path, 0, hdr, sizeof hdr);
    if (n < 54 || hdr[0] != 'B' || hdr[1] != 'M') return false;
    uint32_t pix_off = rd32(hdr + 10), dib = rd32(hdr + 14);
    int32_t w = (int32_t)rd32(hdr + 18), h = (int32_t)rd32(hdr + 22);
    uint32_t bpp = rd16(hdr + 28), comp = rd32(hdr + 30), nused = rd32(hdr + 46);
    bool top_down = h < 0;
    if (top_down) h = -h;
    if (bpp != 8 || comp != 0 || w <= 0 || h <= 0 || w > ART_MAX_W || h > ART_MAX_H) return false;
    int ncol = nused ? (int)nused : 256;
    if (ncol > 256) ncol = 256;
    uint32_t pal_off = 14 + dib;
    if (pal_off + (uint32_t)ncol * 4 > (uint32_t)n) return false;
    static uint32_t src_pal[256];
    for (int c = 0; c < ncol; c++) {
        const uint8_t *p = hdr + pal_off + c * 4;
        src_pal[c] = (uint32_t)p[2] << 16 | (uint32_t)p[1] << 8 | p[0];
    }
    a->w = (uint16_t)w;
    a->h = (uint16_t)h;
    /* all rows in one read to the start of the pixel buffer, packed in
     * place (each row moves down to y * w <= y * stride), then put top
     * first for the usual bottom-up files */
    int stride = (w + 3) & ~3;
    if (stride * h > ART_MAX_W * ART_MAX_H) return false;
    if (fsio_read(path, pix_off, a->px, stride * h) != stride * h) return false;
    for (int y = 1; y < h; y++) memmove(a->px + y * w, a->px + y * stride, (size_t)w);
    if (!top_down) {
        static uint8_t row[ART_MAX_W];
        for (int y = 0; y < h / 2; y++) {
            uint8_t *r0 = a->px + y * w, *r1 = a->px + (h - 1 - y) * w;
            memcpy(row, r0, (size_t)w);
            memcpy(r0, r1, (size_t)w);
            memcpy(r1, row, (size_t)w);
        }
    }
    for (int p = 0; p < w * h; p++)
        if (a->px[p] >= ncol) a->px[p] = 0;
    int key = -1;
    for (int c = 0; c < ncol; c++)
        if (src_pal[c] == ART_KEY_RGB) { key = c; break; }
    if (ncol <= ART_NCOLORS) {
        for (int c = 0; c < ncol; c++) a->pal[c] = src_pal[c];
        a->ncolors = (uint16_t)ncol;
        a->key = (int16_t)key;
    } else {
        reduce_colors(a, src_pal, ncol, key);
    }
    if (a->key >= 0) make_shadow(a);
    a->tint = dominant_colour(a);
    return true;
}

const art_t *cat_art(int i) {
    if (i < 0 || i >= cat_count || !cat[i].has_art || cat[i].art_failed) return NULL;
    use_clock++;
    int victim = 0;
    for (int s = 0; s < ART_SLOTS; s++) {
        if (slot_art[s] && slot_entry[s] == i) {
            slot_used[s] = use_clock;
            return slot_art[s];
        }
        if (!slot_art[s]) {
            slot_art[s] = pmem_alloc(sizeof(art_t));
            slot_entry[s] = -1;
            slot_used[s] = 0;
        }
        if (slot_used[s] < slot_used[victim]) victim = s;
    }
    if (!slot_art[victim]) return NULL;
    slot_entry[victim] = -1;
    if (!load_bmp(i, slot_art[victim])) {
        cat[i].art_failed = true;
        return NULL;
    }
    slot_entry[victim] = i;
    slot_used[victim] = use_clock;
    return slot_art[victim];
}
