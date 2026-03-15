/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * HID report accumulator for multi-stick configurations.
 * Accumulates mouse movement and scroll from all sticks, then
 * emits one combined HID report per scan cycle.
 */

#ifndef ANALOG_STICK_HID_ACCUMULATOR_H
#define ANALOG_STICK_HID_ACCUMULATOR_H

#include <stdint.h>

/**
 * Accumulate mouse movement delta (pixels).
 * Values are summed across all sticks; one HID report is sent on flush.
 */
void zmk_analog_stick_hid_move_add(int32_t dx, int32_t dy);

/**
 * Accumulate scroll delta (ticks).
 * Values are summed across all sticks; one HID report is sent on flush.
 */
void zmk_analog_stick_hid_scroll_add(int32_t sx, int32_t sy);

/**
 * Flush accumulated mouse movement and scroll into a single HID report.
 * No-op if nothing has been accumulated since the last flush.
 * Called by the scan coordinator after all sticks have been processed.
 */
void zmk_analog_stick_hid_flush(void);

#endif /* ANALOG_STICK_HID_ACCUMULATOR_H */
