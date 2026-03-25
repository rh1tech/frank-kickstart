/*
 * Kickstarter - BMP Image Viewer for RP2350
 *
 * 800x600 HDMI (400x300 content with pixel doubling).
 * Uses PICO-BK's patched PicoDVI libdvi at 400 MHz.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/structs/bus_ctrl.h"
#if !PICO_RP2040
#include "hardware/structs/qmi.h"
#endif

#include "board_config.h"
#include "dvi.h"
#include "dvi_timing.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "tmds_encode.h"
#include "ff.h"

#define SCREEN_W 400
#define SCREEN_H 300
#define DVI_TIMING dvi_timing_800x600p_60hz
#define BMP_PATH "/kickstarter/heretic.bmp"

struct dvi_inst dvi0;
struct semaphore dvi_start_sem;

static uint8_t framebuffer[SCREEN_W * SCREEN_H];

// Palette: RGB888 + RGB332 for TMDS encoding
static uint32_t palette_rgb888[256];
static uint8_t palette_rgb332[256];
static uint8_t scanline_rgb332[SCREEN_W] __attribute__((aligned(4)));

static FATFS fs;

static inline uint8_t rgb888_to_rgb332(uint32_t rgb888) {
    uint8_t r = (rgb888 >> 16) & 0xFF;
    uint8_t g = (rgb888 >> 8) & 0xFF;
    uint8_t b = rgb888 & 0xFF;
    return ((r >> 5) << 5) | ((g >> 5) << 2) | (b >> 6);
}

static void palette_set(uint8_t index, uint32_t rgb888) {
    palette_rgb888[index] = rgb888 & 0x00FFFFFF;
    palette_rgb332[index] = rgb888_to_rgb332(rgb888);
}

static void __not_in_flash_func(flash_timings)(void) {
#if !PICO_RP2040
    const int max_flash_freq = 66 * 1000000;
    const int clock_hz = DVI_TIMING.bit_clk_khz * 1000;
    int divisor = (clock_hz + max_flash_freq - 1) / max_flash_freq;
    if (divisor == 1 && clock_hz > 100000000) divisor = 2;
    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000) rxdelay += 1;
    qmi_hw->m[0].timing = 0x60007000 |
                        rxdelay << QMI_M0_TIMING_RXDELAY_LSB |
                        divisor << QMI_M0_TIMING_CLKDIV_LSB;
#endif
    sleep_ms(100);
    set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
}

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
    uint32_t compression  = hdr[30] | (hdr[31] << 8) | (hdr[32] << 16) | (hdr[33] << 24);

    bool top_down = (bmp_h < 0);
    if (top_down) bmp_h = -bmp_h;

    printf("BMP: %dx%d, %d bpp, compression=%lu\n",
           bmp_w, bmp_h, bpp, (unsigned long)compression);

    if (bpp != 8 || compression != 0) {
        printf("Only uncompressed 8-bit indexed BMP supported\n");
        f_close(&file);
        return false;
    }

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

    int x_off = (SCREEN_W - bmp_w) / 2;
    int y_off = (SCREEN_H - bmp_h) / 2;
    if (x_off < 0) x_off = 0;
    if (y_off < 0) y_off = 0;

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

    int num_colors = pal_size / 4;
    for (int i = 0; i < num_colors && i < 256; i++) {
        uint8_t b = pal[i * 4 + 0];
        uint8_t g = pal[i * 4 + 1];
        uint8_t r = pal[i * 4 + 2];
        palette_set(i, (r << 16) | (g << 8) | b);
    }

    printf("Image loaded (%dx%d at +%d,+%d)\n", bmp_w, bmp_h, x_off, y_off);
    return true;
}

// Core 1: DVI output + scanline encode (PICO-BK pattern)
void __not_in_flash_func(core1_main)(void) {
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_1);
    dvi_start(&dvi0);
    sem_acquire_blocking(&dvi_start_sem);

    uint pixwidth = dvi0.timing->h_active_pixels;
    uint words_per_channel = pixwidth / DVI_SYMBOLS_PER_WORD;
    uint32_t *tmdsbuf;

    while (true) {
        for (uint y = 0; y < SCREEN_H; y++) {
            queue_remove_blocking_u32(&dvi0.q_tmds_free, &tmdsbuf);

            // Palette lookup: indexed -> RGB332
            const uint8_t *src = &framebuffer[y * SCREEN_W];
            for (int x = 0; x < SCREEN_W; x++)
                scanline_rgb332[x] = palette_rgb332[src[x]];

            // Encode 3 channels with pixel doubling (400 -> 800)
            const uint32_t *pixbuf = (const uint32_t *)scanline_rgb332;
            tmds_encode_data_channel_8bpp(pixbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_8BPP_BLUE_MSB,  DVI_8BPP_BLUE_LSB);
            tmds_encode_data_channel_8bpp(pixbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_8BPP_GREEN_MSB, DVI_8BPP_GREEN_LSB);
            tmds_encode_data_channel_8bpp(pixbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_8BPP_RED_MSB,   DVI_8BPP_RED_LSB);

            queue_add_blocking_u32(&dvi0.q_tmds_valid, &tmdsbuf);
        }
    }
}

int main(void) {
    // 400 MHz for 800x600@60Hz TMDS bit clock
    vreg_disable_voltage_limit();
    vreg_set_voltage(VREG_VOLTAGE_1_60);
    flash_timings();
    sleep_ms(10);

    stdio_init_all();

    for (int i = 5; i > 0; i--) {
        printf("Starting in %d...\n", i);
        sleep_ms(1000);
    }

    printf("Kickstarter - BMP Image Viewer (800x600)\n");
    printf("Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);

    for (int i = 0; i < 256; i++) palette_set(i, 0x000000);
    memset(framebuffer, 0, sizeof(framebuffer));

    // Init DVI
    dvi0.timing = &DVI_TIMING;
    dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
    dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

    // Mount SD and load BMP
    printf("Mounting SD card...\n");
    FRESULT res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        printf("SD mount failed: %d\n", res);
        while (1) sleep_ms(1000);
    }
    printf("SD card mounted.\n");
    load_bmp(BMP_PATH);

    // Launch Core 1
    sem_init(&dvi_start_sem, 0, 1);
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);
    multicore_launch_core1(core1_main);
    sem_release(&dvi_start_sem);
    printf("DVI started on Core 1 (800x600, 400x300 content).\n");

    while (1) tight_loop_contents();
}
