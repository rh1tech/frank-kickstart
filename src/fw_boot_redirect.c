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

#include <stdbool.h>
#include <stdint.h>
#include "pico.h"
#include "hardware/watchdog.h"
#include "hardware/structs/scb.h"
#include "hardware/regs/intctrl.h"

#define ZERO_BLOCK_OFFSET   ((16ul << 20) - (256ul << 10) - (4ul << 10))
#define ZERO_BLOCK_ADDRESS  (XIP_BASE + ZERO_BLOCK_OFFSET)
#define FLASH_MAGIC_OVER    0x3836d91au
#define SCRATCH_MAGIC_BOOT  0x383da910u
#define SCRATCH_MAGIC_UI    0x17F00FFFu

/* The launcher was asked for by a firmware (SCRATCH_MAGIC_UI): straight to
 * the menu, no start countdown (main.c). */
bool boot_ui_requested;

// scratch[0] = boot magic (SCRATCH_MAGIC_BOOT after flash, SCRATCH_MAGIC_UI for escape)
// scratch[1-3] = reserved for debug

/* The firmware's sector 0 sits in the flash with its reset vector pointing
 * here (flash_uf2.c), and its stack pointer replaced by ours: the boot ROM
 * and crt0 take both from 0x10000000. Its other vectors stay the firmware's. */
/* runtime_init copied the vector table from 0x10000000, the firmware's when
 * one is installed: its exception handlers are put back to ours. */
extern void isr_nmi(void), isr_hardfault(void), isr_memmanage(void), isr_busfault(void),
    isr_usagefault(void), isr_securefault(void), isr_svcall(void), isr_debugmonitor(void),
    isr_pendsv(void), isr_systick(void), isr_invalid(void), __unhandled_user_irq(void);

static void own_vectors(void) {
    const uint32_t *flash_vt = (const uint32_t *)XIP_BASE;
    void (*const sys[16])(void) = {
        0, 0, isr_nmi, isr_hardfault, isr_memmanage, isr_busfault, isr_usagefault, isr_securefault,
        isr_invalid, isr_invalid, isr_invalid, isr_svcall, isr_debugmonitor, isr_invalid,
        isr_pendsv, isr_systick,
    };
    uint32_t *vt = (uint32_t *)scb_hw->vtor;
    for (int i = 2; i < 16 + NUM_IRQS; i++)
        if (vt[i] == flash_vt[i]) vt[i] = (uint32_t)(i < 16 ? sys[i] : __unhandled_user_irq);
}

__attribute__((constructor))
static void before_main(void) {
    uint32_t magic = watchdog_hw->scratch[0];

    // If UI magic is set (e.g., from SELECT button), skip redirect
    if (magic == SCRATCH_MAGIC_UI) {
        watchdog_hw->scratch[0] = 0;
        boot_ui_requested = true;
        own_vectors();
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
    own_vectors();
}

// ZERO_BLOCK placeholder in flash (linker places at ERASE region)
const uint8_t erase_block[4096]
    __attribute__((aligned(4096), section(".erase_block"), used)) = { 0 };
