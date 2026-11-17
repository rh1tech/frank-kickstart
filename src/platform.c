/*
 * frank-kickstart - platform helpers (reboot, UART debug commands)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/structs/psm.h"
#include "hardware/structs/qmi.h"
#include "board_config.h"
#include "platform.h"
#include "psram_init.h"

#define FLASH_MAX_MHZ 66

/* Flash QMI divider and read delay for clk_sys = cpu_mhz. Runs from RAM:
 * the flash is being retimed. */
static void __no_inline_not_in_flash_func(set_flash_timings)(int cpu_mhz) {
    const int clock_hz = cpu_mhz * 1000000;
    const int max_flash_freq = FLASH_MAX_MHZ * 1000000;
    int divisor = (clock_hz + max_flash_freq - 1) / max_flash_freq;
    if (divisor == 1 && clock_hz > 100000000) divisor = 2;
    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000) rxdelay += 1;
    qmi_hw->m[0].timing = 0x60007000 |
                          rxdelay << QMI_M0_TIMING_RXDELAY_LSB |
                          divisor << QMI_M0_TIMING_CLKDIV_LSB;
}

void platform_init_clocks(void) {
    vreg_disable_voltage_limit();
    vreg_set_voltage(CPU_VOLTAGE);
    sleep_ms(10);
    /* slow the flash down first, then raise the clock */
    set_flash_timings(CPU_CLOCK_MHZ);
    set_sys_clock_khz(CPU_CLOCK_MHZ * 1000, true);
}

void platform_slow_clocks(void) {
    /* the boot ROM's flash routines reset the QMI timing; at 150 MHz the
     * default (divider 2) is still in spec */
    set_sys_clock_khz(150000, true);
    set_flash_timings(150);
}

/* The PSRAM must leave QPI mode first, or the boot ROM cannot read the
 * flash and the chip hangs until a power cycle (see psram_reset_to_spi). */
void __attribute__((noreturn)) platform_reboot(bool bootsel) {
    stdio_flush();
    if (get_core_num() == 0) {
        /* stop core 1 (it may be reading PSRAM). Not multicore_reset_core1():
         * that waits for core 1's boot ROM to answer on the FIFO; we are
         * rebooting anyway, so just force it off. */
        hw_set_bits(&psm_hw->frce_off, PSM_FRCE_OFF_PROC1_BITS);
        while (!(psm_hw->frce_off & PSM_FRCE_OFF_PROC1_BITS)) tight_loop_contents();
        busy_wait_us(10);
        /* release it again: it restarts into the boot ROM, which waits for a
         * launch and does not touch PSRAM. Left forced off, the bit survives
         * the watchdog reboot and the chip does not come back. */
        hw_clear_bits(&psm_hw->frce_off, PSM_FRCE_OFF_PROC1_BITS);
        busy_wait_us(10);
    }
    save_and_disable_interrupts();
    psram_reset_to_spi();
    if (bootsel) reset_usb_boot(0, 0);
    watchdog_reboot(0, 0, 10);
    while (1) tight_loop_contents();
}

void platform_uart_cmd(int ch) {
    static const char boot[] = "~BOOT~";
    static unsigned bpos;
    bpos = (ch == boot[bpos]) ? bpos + 1 : (ch == boot[0]);
    if (bpos == sizeof(boot) - 1) {
        printf("\nrebooting to BOOTSEL\n");
        sleep_ms(20);
        platform_reboot(true);
    }
    static const char rst[] = "~RST~";
    static unsigned rpos;
    rpos = (ch == rst[rpos]) ? rpos + 1 : (ch == rst[0]);
    if (rpos == sizeof(rst) - 1) {
        printf("\nrebooting\n");
        sleep_ms(20);
        platform_reboot(false);
    }
}
