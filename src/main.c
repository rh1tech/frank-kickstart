/*
 * Kickstarter - HDMI Test Pattern for RP2350
 *
 * Displays a color bar test pattern on HDMI output.
 * Uses the HDMI driver from MurmSNES project.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"

#include "board_config.h"
#include "HDMI.h"

// Screen double-buffer expected by HDMI driver
// 256 pixels wide x 224 lines, palette-indexed (8-bit)
uint8_t SCREEN[2][256 * 224];
volatile uint32_t current_buffer = 0;

// Palette colors for the test pattern (RGB888)
static const uint32_t test_colors[] = {
    0xFFFFFF, // White
    0xFFFF00, // Yellow
    0x00FFFF, // Cyan
    0x00FF00, // Green
    0xFF00FF, // Magenta
    0xFF0000, // Red
    0x0000FF, // Blue
    0x000000, // Black
};

#define NUM_TEST_COLORS (sizeof(test_colors) / sizeof(test_colors[0]))

// Generate classic SMPTE-style color bar test pattern
static void generate_test_pattern(void) {
    const int width = 256;
    const int height = 224;
    const int bar_width = width / NUM_TEST_COLORS;

    // Set up palette: index 0-7 = test colors, index 0 also = black background
    for (int i = 0; i < (int)NUM_TEST_COLORS; i++) {
        graphics_set_palette(i + 1, test_colors[i]);
    }
    // Index 0 = black (default after init)
    graphics_set_palette(0, 0x000000);

    // Fill both screen buffers with the test pattern
    for (int buf = 0; buf < 2; buf++) {
        uint8_t *screen = SCREEN[buf];

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int bar = x / bar_width;
                if (bar >= (int)NUM_TEST_COLORS) bar = NUM_TEST_COLORS - 1;

                if (y < height * 2 / 3) {
                    // Top 2/3: color bars
                    screen[y * width + x] = bar + 1;
                } else if (y < height * 3 / 4) {
                    // Middle band: reversed/complementary bars
                    int rev_bar = (NUM_TEST_COLORS - 1) - bar;
                    screen[y * width + x] = rev_bar + 1;
                } else {
                    // Bottom: grayscale ramp
                    int gray_val = (x * 255) / (width - 1);
                    // Map to palette index 9..9+31 (32 grey levels)
                    int gray_idx = 9 + (gray_val >> 3); // 0-31 range
                    screen[y * width + x] = gray_idx;
                }
            }
        }
    }

    // Set up grayscale palette entries (indices 9..40)
    for (int i = 0; i < 32; i++) {
        uint8_t g = i * 255 / 31;
        graphics_set_palette(9 + i, (g << 16) | (g << 8) | g);
    }
}

// Core 1: runs the HDMI output
static void core1_main(void) {
    graphics_init(g_out_HDMI);
    graphics_set_mode(GRAPHICSMODE_DEFAULT);
    graphics_set_res(256, 224);

    // Spin forever — HDMI runs from DMA interrupts
    while (1) {
        tight_loop_contents();
    }
}

int main(void) {
    // Init stdio at default clock so USB CDC enumerates reliably
    stdio_init_all();

    // Wait for USB serial console to connect
    for (int i = 5; i > 0; i--) {
        printf("Starting in %d...\n", i);
        sleep_ms(1000);
    }

    printf("Kickstarter - HDMI Test Pattern\n");

    // Set 252 MHz — required by HDMI driver for pixel clock
    set_sys_clock_khz(252000, true);
    sleep_ms(10);
    printf("Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);

    // Clear screen buffers
    memset(SCREEN, 0, sizeof(SCREEN));

    // Launch HDMI on core 1
    multicore_launch_core1(core1_main);
    sleep_ms(100); // Let HDMI initialize

    // Generate and display the test pattern
    generate_test_pattern();

    printf("Test pattern active.\n");

    while (1) {
        tight_loop_contents();
    }

    return 0;
}
