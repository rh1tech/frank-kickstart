/*
 * Frank-Kickstart - UF2 firmware launcher for RP2350 boards (M1, M2, Z0)
 *
 * Lists the .uf2 files in /kickstart on the SD card with their cover art
 * (X.bmp) and metadata (X.xml) on 640x480 HDMI, and writes the chosen one
 * into the flash. PSRAM is required.
 *
 * Copyright (c) 2025-2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/watchdog.h"
#include "board_config.h"
#include "catalog.h"
#include "flash_uf2.h"
#include "fsio.h"
#include "input.h"
#include "music.h"
#include "platform.h"
#include "psram_init.h"
#include "ui.h"
#include "video.h"

#define LAST_FILE CAT_DIR "/.last"
#define MUSIC_FILE CAT_DIR "/music.wav"
#define CONFIG_FILE CAT_DIR "/.config"
#define COUNTDOWN_S 4

/* Core 1 only scans the video out and decodes the music. It must not touch
 * the flash after the start (the flash is written with the video running),
 * so its loop is in RAM (and not inlined into core1_main, which is in flash).
 * The line IRQs wake it often enough. */
static void __no_inline_not_in_flash_func(core1_idle)(void) {
    while (true) {
        music_core1();
        __wfe();
    }
}

static void core1_main(void) {
    video_start();
    core1_idle();
}

/* The name in /kickstart/.last: the firmware in the flash now. */
static int read_last(void) {
    char name[96];
    int n = fsio_read(LAST_FILE, 0, name, sizeof name - 1);
    if (n <= 0) return -1;
    name[n] = 0;
    while (n > 0 && (name[n - 1] == '\n' || name[n - 1] == '\r' || name[n - 1] == ' ')) name[--n] = 0;
    return cat_find(name);
}

/* /kickstart/.config: "music=0" when the music was switched off. */
static bool config_music(void) {
    char s[64];
    int n = fsio_read(CONFIG_FILE, 0, s, sizeof s - 1);
    if (n <= 0) return true;
    s[n] = 0;
    return !strstr(s, "music=0");
}

static void config_save(void) {
    const char *s = music_is_on() ? "music=1\n" : "music=0\n";
    fsio_write(CONFIG_FILE, s, (int)strlen(s));
}

/* Shows the message and keeps polling (the UART still takes ~BOOT~). */
static void __attribute__((noreturn)) halt(const char *title, const char *l1, const char *l2) {
    ui_message(title, l1, l2);
    printf("%s: %s %s\n", title, l1 ? l1 : "", l2 ? l2 : "");
    while (true) {
        input_poll();
        sleep_ms(16);
    }
}

static void launch(int sel) {
    const cat_entry_t *e = &cat[sel];
    if (sel == ui_installed) {
        printf("Starting %s (already in the flash)\n", e->filename);
        music_stop();
        ui_flash_begin(sel, true);
        sleep_ms(300);
        flash_boot_into();
    }
    char path[160];
    cat_path(sel, path, sizeof path);
    printf("Installing %s\n", path);
    music_stop();
    ui_flash_begin(sel, false);
    absolute_time_t t0 = get_absolute_time();
    if (!flash_uf2(path, ui_flash_progress)) {
        halt("Install failed", "The file could not be read from the SD card.", "Restart the board to try again.");
    }
    printf("Installed in %lld ms\n", absolute_time_diff_us(t0, get_absolute_time()) / 1000);
    fsio_write(LAST_FILE, path, (int)strlen(path));
    fsio_unmount();
    sleep_ms(200);
    flash_boot_into();
}

/* Boot progress: the stage in watchdog scratch[1] (it survives a reset, so
 * a hang can be read back over SWD), and with FK_BOOT_TRACE a white square
 * per stage at the top left of the screen. */
static void boot_stage(int n) {
    watchdog_hw->scratch[1] = 0xB0070000u | (uint32_t)n;
    printf("[%6lu ms] stage %d\n", (unsigned long)to_ms_since_boot(get_absolute_time()), n);
#ifdef FK_BOOT_TRACE
    for (int y = 8; y < 24; y++) memset(video_fb + y * VIDEO_W + 8 + n * 20, 255, 16);
#endif
}

int main(void) {
    platform_init_clocks();

    /* the video before anything else: whatever fails later can still say so
     * on the screen (it also takes fixed DMA channels) */
    memset(video_fb, 0, sizeof video_fb);
    video_init();
    {
        static const uint32_t white = 0xFFFFFF;
        video_stage_palette(255, 1, &white);
        video_commit_palette();
    }
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS |
                BUSCTRL_BUS_PRIORITY_DMA_R_BITS | BUSCTRL_BUS_PRIORITY_DMA_W_BITS);
    multicore_launch_core1(core1_main);
    boot_stage(1);

    stdio_init_all();
    printf("\nFrank-Kickstart v%d.%02d, board %s, %lu MHz, video %s\n", FK_VERSION_MAJOR, FK_VERSION_MINOR,
           BOARD_NAME, (unsigned long)(clock_get_hz(clk_sys) / 1000000), video_backend_name());
    boot_stage(2);

    /* whatever state the PSRAM was left in (another firmware's QPI mode,
     * a warm reset), back to its power-on SPI mode first */
    gpio_set_function(get_psram_pin(), GPIO_FUNC_XIP_CS1);
    psram_reset_to_spi();
    psram_init(get_psram_pin());
    boot_stage(3);
    bool psram_ok = pmem_init();
    boot_stage(4);

    input_init();
    boot_stage(5);

    if (!psram_ok) {
        printf("No PSRAM\n");
        halt("PSRAM required", "This launcher needs the board's 8 MB PSRAM chip.", "Fit the PSRAM and restart the board.");
    }
    ui_init();
    boot_stage(6);

    bool sd_ok = fsio_mount();
    boot_stage(7);
    int sel = 0;
    /* a firmware in the flash: ZERO_BLOCK holds its sector 0 (flashing the
     * launcher clears it) */
    bool installed = ((const uint32_t *)(XIP_BASE + ZERO_BLOCK_OFFSET))[1023] == FLASH_MAGIC_OVER;
    if (sd_ok) {
        uint32_t ts = to_ms_since_boot(get_absolute_time());
        cat_scan();
        printf("%d firmware file(s), scanned in %lu ms\n", cat_count,
               (unsigned long)(to_ms_since_boot(get_absolute_time()) - ts));
        for (int i = 0; i < cat_count; i++)
            printf("  %s/%s: %s (boards 0x%x)\n", cat[i].dir, cat[i].filename, cat[i].title, cat[i].boards);
        /* .last names the firmware in the flash */
        if (installed) ui_installed = read_last();
        if (ui_installed >= 0) sel = ui_installed;
        /* the first covers, before the welcome screen ends */
        cat_art(sel);
    }
    boot_stage(8);

    if (installed && !boot_ui_requested) {
        /* a reset or power-on with a firmware in the flash: it starts after
         * the countdown, unless a key asks for the menu */
        printf("Starting %s in %d s (any key: menu)\n", ui_installed >= 0 ? cat[ui_installed].filename : "the firmware",
               COUNTDOWN_S);
        if (!ui_countdown(ui_installed, COUNTDOWN_S)) {
            printf("Starting the firmware\n");
            if (sd_ok) fsio_unmount();
            flash_boot_into();
        }
        if (sd_ok) music_start(MUSIC_FILE, config_music());
    } else {
        /* background music, where the video carries sound (M2) */
        if (sd_ok) music_start(MUSIC_FILE, config_music());
        ui_welcome();
    }

    if (!sd_ok) halt("No SD card", "Insert an SD card with a /kickstart folder", "and restart the board.");
    if (!cat_count) halt("No firmware found", "Copy .uf2 files into /kickstart on the SD card", "and restart the board.");

    ui_browser_show(sel);
    cat_art((sel + 1) % cat_count);
    cat_art((sel + cat_count - 1) % cat_count);

    /* nothing until every key has been let go once (no launch from a key
     * still held, or PS/2 noise, after the reboot) */
    while (input_poll()) sleep_ms(16);

    uint32_t prev = 0, repeat = 0, combo_frames = 0;
    bool confirm = false;
    while (true) {
        video_wait_vblank();
        uint32_t in = input_poll();
        uint32_t pressed = in & ~prev;
        if (in && in == prev) {
            if (++repeat > 20 && repeat % 5 == 0) pressed = in & (IN_LEFT | IN_RIGHT | IN_UP | IN_DOWN);
        } else {
            repeat = 0;
        }
        prev = in;

        /* Select+Start held for 3 s: BOOTSEL (to reflash the launcher) */
        combo_frames = (in & IN_BOOTSEL_COMBO) ? combo_frames + 1 : 0;
        if (combo_frames > 180) {
            ui_message("USB boot mode", "Connect the board's USB port to a computer", "to flash new firmware.");
            platform_reboot(true);
        }

        int step = 0;
        if (pressed & (IN_LEFT | IN_UP)) step = -1;
        if (pressed & (IN_RIGHT | IN_DOWN)) step = 1;
        if (step && cat_count > 1) {
            sel = (sel + step + cat_count) % cat_count;
            confirm = false;
            ui_browser_move(sel, step);
            /* the neighbours' covers, while nothing moves */
            cat_art((sel + step + cat_count) % cat_count);
            printf("Selected %s (%d/%d)\n", cat[sel].filename, sel + 1, cat_count);
            continue;
        }
        if ((pressed & IN_MUSIC) && music_available()) {
            music_set_on(!music_is_on());
            ui_browser_footer(sel, confirm);
            config_save();
            printf("Music %s\n", music_is_on() ? "on" : "off");
        }
        if (pressed & IN_BACK) {
            confirm = false;
            ui_browser_confirm(sel, false);
        }
        if (pressed & IN_OK) {
            cat_check(sel);
            if (cat[sel].status != UF2_OK) continue;
            if (cat_wrong_board(sel) && sel != ui_installed && !confirm) {
                confirm = true;
                ui_browser_confirm(sel, true);
                continue;
            }
            launch(sel);
        }
    }
}
