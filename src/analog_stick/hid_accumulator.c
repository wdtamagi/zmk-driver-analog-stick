/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * HID report accumulator — sums mouse movement and scroll contributions
 * from all analog stick instances, then emits one combined report per
 * scan cycle. Prevents HID report races when multiple sticks are in
 * mouse or scroll mode simultaneously.
 */

#include <zephyr/kernel.h>

#include <analog-stick/hid_accumulator.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>

static int32_t acc_dx, acc_dy;
static int32_t acc_sx, acc_sy;
static bool acc_dirty;

static inline int32_t sat_add32(int32_t a, int32_t b) {
    int64_t r = (int64_t)a + b;
    if (r > INT32_MAX) return INT32_MAX;
    if (r < INT32_MIN) return INT32_MIN;
    return (int32_t)r;
}

void zmk_analog_stick_hid_move_add(int32_t dx, int32_t dy) {
    acc_dx = sat_add32(acc_dx, dx);
    acc_dy = sat_add32(acc_dy, dy);
    acc_dirty = true;
}

void zmk_analog_stick_hid_scroll_add(int32_t sx, int32_t sy) {
    acc_sx = sat_add32(acc_sx, sx);
    acc_sy = sat_add32(acc_sy, sy);
    acc_dirty = true;
}

void zmk_analog_stick_hid_flush(void) {
    if (!acc_dirty) {
        return;
    }

    zmk_hid_mouse_movement_set(acc_dx, acc_dy);
    zmk_hid_mouse_scroll_set(acc_sx, acc_sy);
    zmk_endpoint_send_mouse_report();

    acc_dx = 0;
    acc_dy = 0;
    acc_sx = 0;
    acc_sy = 0;
    acc_dirty = false;
}
