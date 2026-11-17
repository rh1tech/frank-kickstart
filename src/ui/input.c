/*
 * frank-kickstart - input from every source, merged
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "board_config.h"
#include "input.h"
#include "nespad.h"
#include "platform.h"
#include "ps2kbd_wrapper.h"
#ifdef USB_HID_ENABLED
#include "usbhid.h"
#endif

void input_init(void) {
    nespad_begin(clock_get_hz(clk_sys) / 1000, NESPAD_GPIO_CLK, NESPAD_GPIO_DATA, NESPAD_GPIO_LATCH);
    ps2kbd_init();
#ifdef USB_HID_ENABLED
    usbhid_init();
#endif
}

uint32_t input_poll(void) {
    uint32_t in = 0;
    nespad_read();
    uint32_t p = nespad_state;
    if (p & DPAD_LEFT) in |= IN_LEFT;
    if (p & DPAD_RIGHT) in |= IN_RIGHT;
    if (p & DPAD_UP) in |= IN_UP;
    if (p & DPAD_DOWN) in |= IN_DOWN;
    if (p & (DPAD_A | DPAD_START)) in |= IN_OK;
    if (p & DPAD_B) in |= IN_BACK;
    if ((p & (DPAD_SELECT | DPAD_START)) == (DPAD_SELECT | DPAD_START)) in |= IN_BOOTSEL_COMBO;

    ps2kbd_tick();
    uint16_t k = ps2kbd_get_state();
#ifdef USB_HID_ENABLED
    usbhid_task();
    k |= usbhid_get_kbd_state();
    usbhid_gamepad_state_t g;
    usbhid_get_gamepad_state(&g);
    if (g.connected) {
        if ((g.dpad & 0x04) || g.axis_x < -64) in |= IN_LEFT;
        if ((g.dpad & 0x08) || g.axis_x > 64) in |= IN_RIGHT;
        if ((g.dpad & 0x01) || g.axis_y < -64) in |= IN_UP;
        if ((g.dpad & 0x02) || g.axis_y > 64) in |= IN_DOWN;
        if (g.buttons & (0x01 | 0x40)) in |= IN_OK;
        if (g.buttons & 0x02) in |= IN_BACK;
        if ((g.buttons & 0xC0) == 0xC0) in |= IN_BOOTSEL_COMBO;
    }
#endif
    if (k & KBD_STATE_LEFT) in |= IN_LEFT;
    if (k & KBD_STATE_RIGHT) in |= IN_RIGHT;
    if (k & KBD_STATE_UP) in |= IN_UP;
    if (k & KBD_STATE_DOWN) in |= IN_DOWN;
    if (k & (KBD_STATE_A | KBD_STATE_START)) in |= IN_OK;
    if (k & (KBD_STATE_B | KBD_STATE_ESC)) in |= IN_BACK;
    if (k & KBD_STATE_M) in |= IN_MUSIC;

    /* the debug console: l/h left, r right, m music, Enter launches */
    int c;
    while ((c = getchar_timeout_us(0)) >= 0) {
        platform_uart_cmd(c);
        if (c == 'l' || c == 'h') in |= IN_LEFT;
        if (c == 'r') in |= IN_RIGHT;
        if (c == 'm') in |= IN_MUSIC;
        if (c == '\r' || c == '\n') in |= IN_OK;
    }
    return in;
}
