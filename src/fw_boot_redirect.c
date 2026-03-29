/*
 * Frank-Kickstart — Boot redirect (from pico-launcher m1p2-uf2)
 *
 * Runs before main() via __attribute__((constructor)).
 * Checks watchdog scratch regs and ZERO_BLOCK to decide: run firmware or frank-kickstart.
 *
 * Uses watchdog scratch registers instead of SRAM magic — SRAM near the top
 * can be corrupted by the RP2350 boot ROM's stack during reboot (varies by
 * firmware binary layout).
 */

#include <stdint.h>
#include "pico.h"
#include "hardware/watchdog.h"

#define ZERO_BLOCK_OFFSET   ((16ul << 20) - (256ul << 10) - (4ul << 10))
#define ZERO_BLOCK_ADDRESS  (XIP_BASE + ZERO_BLOCK_OFFSET)
#define FLASH_MAGIC_OVER    0x3836d91au
#define SCRATCH_MAGIC_BOOT  0x383da910u
#define SCRATCH_MAGIC_UI    0x17F00FFFu

// scratch[0] = boot magic (SCRATCH_MAGIC_BOOT after flash, SCRATCH_MAGIC_UI for escape)
// scratch[1-3] = reserved for debug

__attribute__((constructor))
static void before_main(void) {
    uint32_t magic = watchdog_hw->scratch[0];

    // If UI magic is set (e.g., from SELECT button), skip redirect
    if (magic == SCRATCH_MAGIC_UI) {
        watchdog_hw->scratch[0] = 0;
        return;  // run frank-kickstart
    }

    // If boot magic is set (from watchdog reboot after flash), jump to firmware
    if (magic == SCRATCH_MAGIC_BOOT) {
        watchdog_hw->scratch[0] = 0;

        if (((uint32_t *)ZERO_BLOCK_ADDRESS)[1023] == FLASH_MAGIC_OVER) {
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
