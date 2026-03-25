/*
 * Frank-Kickstart — Boot redirect (from pico-launcher m1p2-uf2)
 *
 * Runs before main() via __attribute__((constructor)).
 * Checks SRAM magic and ZERO_BLOCK to decide: run firmware or frank-kickstart.
 */

#include <stdint.h>
#include "pico.h"
#include "hardware/regs/m33.h"

#define ZERO_BLOCK_OFFSET   ((16ul << 20) - (256ul << 10) - (4ul << 10))
#define ZERO_BLOCK_ADDRESS  (XIP_BASE + ZERO_BLOCK_OFFSET)
#define FLASH_MAGIC_OVER    0x3836d91au
#define SRAM_MAGIC_BOOT     0x383da910u
#define SRAM_MAGIC_UI       0x17F00FFFu

#define SRAM_TOP            (0x20000000 + (512 << 10))

__attribute__((constructor))
static void before_main(void) {
    volatile uint32_t *sram_ui   = (volatile uint32_t *)(SRAM_TOP - 4);
    volatile uint32_t *sram_boot = (volatile uint32_t *)(SRAM_TOP - 8);

    // If UI magic is set (e.g., from SELECT button), skip redirect
    if (*sram_ui == SRAM_MAGIC_UI) {
        *sram_ui = 0;
        return;  // run frank-kickstart
    }

    // If boot magic is set (from watchdog reboot after flash), jump to firmware
    if (*sram_boot == SRAM_MAGIC_BOOT) {
        *sram_boot = 0;

        if (((uint32_t *)ZERO_BLOCK_ADDRESS)[1023] == FLASH_MAGIC_OVER) {
            // VTOR stays at 0x10000000 to preserve firmware IRQ vectors
            // Jump to firmware's original Reset from ZERO_BLOCK
            __asm volatile (
                "ldr r0, =%[zb_addr]\n"
                "ldmia r0, {r0, r1}\n"
                "msr msp, r0\n"
                "bx r1\n"
                :: [zb_addr] "X" (ZERO_BLOCK_ADDRESS)
            );
            __builtin_unreachable();
        }
    }

    // No valid firmware or no magic — run frank-kickstart
}

// ZERO_BLOCK placeholder in flash (linker places at ERASE region)
const uint8_t erase_block[4096]
    __attribute__((aligned(4096), section(".erase_block"), used)) = { 0 };
