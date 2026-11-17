/*
 * frank-kickstart - renders the launcher screens on the host, to PPM files,
 * through the same drawing and UI code as the firmware.
 *
 *   tools/host_render.sh            (builds this and writes PNGs into build-host)
 *
 * FK_SD points at an SD card image directory (with a kickstart/ folder);
 * FK_OUT is the output directory; FK_PIO=1 shows the colours the PIO video
 * driver (M1/Z0) produces instead of the HSTX ones (M2).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "catalog.h"
#include "input.h"
#include "music.h"
#include "tmds_balanced.h"
#include "ui.h"
#include "video.h"

uint8_t video_fb[VIDEO_W * VIDEO_H];
static uint32_t pal_staged[256], pal_shown[256];
static int frame;
#define MAX_DUMPS 16
static int dump_frames[MAX_DUMPS];
static const char *dump_names[MAX_DUMPS];
static int ndumps;
static int input_from = 1 << 30;
static const char *out_dir;
static int pio_colours;

static void write_ppm(const char *name) {
    char path[512];
    snprintf(path, sizeof path, "%s/%s.ppm", out_dir, name);
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); exit(1); }
    fprintf(f, "P6\n%d %d\n255\n", VIDEO_W, VIDEO_H);
    for (int i = 0; i < VIDEO_W * VIDEO_H; i++) {
        uint32_t c = pal_shown[video_fb[i]];
        uint8_t rgb[3] = {(uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c};
        if (pio_colours) {
            for (int k = 0; k < 3; k++) rgb[k] = tmds_balanced_snap(rgb[k]);
        } else {
            /* BGR555 on the HSTX */
            for (int k = 0; k < 3; k++) rgb[k] = (uint8_t)((rgb[k] >> 3) * 255 / 31);
        }
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
    printf("wrote %s\n", path);
}

/* --- the video and input the UI expects --- */
void video_init(void) {}
void video_start(void) {}
void video_stop(void) {}
void video_stage_palette(int first, int n, const uint32_t *rgb) {
    for (int i = 0; i < n; i++) pal_staged[first + i] = rgb[i];
}
void video_commit_palette(void) { memcpy(pal_shown, pal_staged, sizeof pal_shown); }
bool video_palette_pending(void) { return false; }
uint32_t video_frame_count(void) { return (uint32_t)frame; }
void video_wait_vblank(void) {
    frame++;
    for (int i = 0; i < ndumps; i++)
        if (frame == dump_frames[i]) write_ppm(dump_names[i]);
}
uint32_t video_irq_max_us(void) { return 0; }
bool video_set_audio_source(video_audio_source_t s) { (void)s; return false; }
void video_set_frame_hook(void (*hook)(void)) { (void)hook; }
/* as on M2 with music playing: the footer shows the M hint */
bool music_available(void) { return true; }
bool music_is_on(void) { return true; }
uint32_t video_errors(void) { return 0; }
const char *video_backend_name(void) { return "host"; }
uint32_t video_quantize(uint32_t c) {
    uint32_t out = 0;
    for (int sh = 0; sh <= 16; sh += 8) {
        uint8_t v = (uint8_t)(c >> sh);
        out |= (uint32_t)(pio_colours ? tmds_balanced_snap(v) : (v >> 3) * 255 / 31) << sh;
    }
    return out;
}
void input_init(void) {}
uint32_t input_poll(void) { return frame >= input_from ? IN_OK : 0; }

/* write the screen as `name` f frames from now (several may be pending) */
static void dump_at(int f, const char *name) {
    if (ndumps < MAX_DUMPS) {
        dump_frames[ndumps] = frame + f;
        dump_names[ndumps++] = name;
    }
}

int main(void) {
    out_dir = getenv("FK_OUT") ? getenv("FK_OUT") : ".";
    pio_colours = getenv("FK_PIO") && atoi(getenv("FK_PIO"));
    tmds_balanced_init();
    pmem_init();
    ui_init();
    cat_scan();
    printf("%d entries\n", cat_count);
    ui_installed = cat_find("nes.uf2");

    /* the start countdown: frames in its second and fourth second */
    dump_at(12 + 70, "00_countdown_3");
    dump_at(12 + 185, "00_countdown_1");
    ui_countdown(ui_installed, 4);

    /* welcome: a frame mid-animation, then a key */
    dump_at(200, "01_welcome");
    input_from = frame + 260;
    ui_welcome();
    input_from = 1 << 30;

    int sel = ui_installed >= 0 ? ui_installed : 0;
    ui_browser_show(sel);
    write_ppm("02_browser_installed");

    /* a move, mid-way and at the end */
    dump_at(5, "03_move_mid");
    ui_browser_move((sel + 1) % cat_count, 1);
    write_ppm("04_browser_next");
    sel = (sel + 1) % cat_count;

    /* every entry */
    for (int i = 0; i < cat_count; i++) {
        char name[96];
        ui_browser_move(i, 1);
        snprintf(name, sizeof name, "10_entry_%02d_%s", i, cat[i].basename);
        write_ppm(name);
        if (cat_wrong_board(i)) {
            ui_browser_confirm(i, true);
            snprintf(name, sizeof name, "11_confirm_%s", cat[i].basename);
            write_ppm(name);
            ui_browser_confirm(i, false);
        }
    }

    ui_flash_begin(0, false);
    ui_flash_progress(420, 1000);
    write_ppm("20_installing");

    ui_message("No firmware found", "Copy .uf2 files into /kickstart on the SD card", "and restart the board.");
    write_ppm("21_message");
    return 0;
}
