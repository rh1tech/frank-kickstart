/*
 * Kickstarter - BMP Image Viewer for RP2350
 *
 * Loads an 8-bit indexed BMP from SD card and displays it
 * centered on the HDMI screen (320x240).
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"

#include "board_config.h"
#include "HDMI.h"
#include "ff.h"

#define SCREEN_W 320
#define SCREEN_H 240
#define BMP_PATH "/kickstarter/heretic.bmp"

// Single framebuffer used by the HDMI driver via graphics_set_buffer()
static uint8_t framebuffer[SCREEN_W * SCREEN_H];
static FATFS fs;

// Load 8-bit indexed BMP and blit centered
static bool load_bmp(const char *path) {
    FIL file;
    UINT br;

    if (f_open(&file, path, FA_READ) != FR_OK) {
        printf("Failed to open %s\n", path);
        return false;
    }

    uint8_t hdr[54];
    if (f_read(&file, hdr, 54, &br) != FR_OK || br != 54) {
        printf("Failed to read BMP header\n");
        f_close(&file);
        return false;
    }

    if (hdr[0] != 'B' || hdr[1] != 'M') {
        printf("Not a BMP file\n");
        f_close(&file);
        return false;
    }

    uint32_t pixel_offset = hdr[10] | (hdr[11] << 8) | (hdr[12] << 16) | (hdr[13] << 24);
    int32_t  bmp_w        = hdr[18] | (hdr[19] << 8) | (hdr[20] << 16) | (hdr[21] << 24);
    int32_t  bmp_h        = hdr[22] | (hdr[23] << 8) | (hdr[24] << 16) | (hdr[25] << 24);
    uint16_t bpp          = hdr[28] | (hdr[29] << 8);

    bool top_down = (bmp_h < 0);
    if (top_down) bmp_h = -bmp_h;

    printf("BMP: %dx%d, %d bpp, offset=%lu\n", bmp_w, bmp_h, bpp, (unsigned long)pixel_offset);

    if (bpp != 8) {
        printf("Only 8-bit indexed BMP supported (got %d bpp)\n", bpp);
        f_close(&file);
        return false;
    }

    // Read palette (4 bytes each: B, G, R, reserved)
    uint8_t pal[256 * 4];
    uint32_t pal_size = (pixel_offset - 54);
    if (pal_size > sizeof(pal)) pal_size = sizeof(pal);

    if (f_read(&file, pal, pal_size, &br) != FR_OK || br != pal_size) {
        printf("Failed to read palette\n");
        f_close(&file);
        return false;
    }

    f_lseek(&file, pixel_offset);

    int row_stride = (bmp_w + 3) & ~3;
    uint8_t row_buf[512];

    if (row_stride > (int)sizeof(row_buf)) {
        printf("Image too wide (%d)\n", bmp_w);
        f_close(&file);
        return false;
    }

    // Center on screen
    int x_off = (SCREEN_W - bmp_w) / 2;
    int y_off = (SCREEN_H - bmp_h) / 2;
    if (x_off < 0) x_off = 0;
    if (y_off < 0) y_off = 0;

    // Clear framebuffer and write pixel data
    memset(framebuffer, 0, sizeof(framebuffer));

    for (int row = 0; row < bmp_h && row < SCREEN_H; row++) {
        if (f_read(&file, row_buf, row_stride, &br) != FR_OK || br != (UINT)row_stride)
            break;

        int screen_y = top_down ? (y_off + row) : (y_off + bmp_h - 1 - row);
        if (screen_y < 0 || screen_y >= SCREEN_H) continue;

        int copy_w = bmp_w;
        if (x_off + copy_w > SCREEN_W) copy_w = SCREEN_W - x_off;

        memcpy(&framebuffer[screen_y * SCREEN_W + x_off], row_buf, copy_w);
    }

    f_close(&file);

    // Set palette LAST (BMP stores BGR)
    int num_colors = pal_size / 4;
    for (int i = 0; i < num_colors && i < 240; i++) {
        uint8_t b = pal[i * 4 + 0];
        uint8_t g = pal[i * 4 + 1];
        uint8_t r = pal[i * 4 + 2];
        graphics_set_palette(i, (r << 16) | (g << 8) | b);
    }

    printf("Image loaded (%dx%d at +%d,+%d)\n", bmp_w, bmp_h, x_off, y_off);
    return true;
}

int main(void) {
    // 252 MHz — required by HDMI driver
    set_sys_clock_khz(252000, true);
    sleep_ms(10);

    stdio_init_all();

    // Wait for USB serial console
    for (int i = 5; i > 0; i--) {
        printf("Starting in %d...\n", i);
        sleep_ms(1000);
    }

    printf("Kickstarter - BMP Image Viewer\n");
    printf("Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);

    // Clear framebuffer with index 1
    memset(framebuffer, 1, sizeof(framebuffer));

    // Initialize HDMI on Core 0
    graphics_init(g_out_HDMI);
    graphics_set_buffer(framebuffer);
    graphics_set_res(SCREEN_W, SCREEN_H);

    // Set index 0 = black, index 1 = dark grey (keeps monitor awake)
    graphics_set_palette(0, 0x000000);
    graphics_set_palette(1, 0x101010);
    graphics_set_bgcolor(0x000000);
    printf("HDMI initialized.\n");

    // Quick driver test: solid red rectangle in center
    graphics_set_palette(2, 0xFF0000); // bright red
    for (int y = 80; y < 160; y++)
        memset(&framebuffer[y * SCREEN_W + 80], 2, 160);
    printf("Red box test. Waiting 5s...\n");
    sleep_ms(5000);

    // Mount SD card
    printf("Mounting SD card...\n");
    FRESULT res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        printf("SD mount failed: %d\n", res);
        while (1) sleep_ms(1000);
    }
    printf("SD card mounted.\n");

    // Load and display image
    if (!load_bmp(BMP_PATH)) {
        printf("Failed to load image.\n");
    }

    while (1) {
        tight_loop_contents();
    }

    return 0;
}
