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
#include <zephyr/sys/util.h>

#include <analog-stick/hid_accumulator.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>

/*
 * Thread safety: all accumulators below are accessed exclusively from the
 * analog stick work queue (single-threaded contract). The scan coordinator
 * work handler calls hid_move_add, hid_scroll_add, and hid_flush in sequence
 * without yielding between them. If multi-queue access is ever introduced,
 * protect these globals with a k_spinlock.
 */
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

    zmk_hid_mouse_movement_update(
        (int16_t)CLAMP(acc_dx, INT16_MIN, INT16_MAX),
        (int16_t)CLAMP(acc_dy, INT16_MIN, INT16_MAX));
    zmk_hid_mouse_scroll_set(
        (int8_t)CLAMP(acc_sx, INT8_MIN, INT8_MAX),
        (int8_t)CLAMP(acc_sy, INT8_MIN, INT8_MAX));
    zmk_endpoints_send_mouse_report();

    acc_dx = 0;
    acc_dy = 0;
    acc_sx = 0;
    acc_sy = 0;
    acc_dirty = false;
}
