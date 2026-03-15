/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Packed split transport format constants. Retained for use
 * by the future packed-ble-gatt-transport change.
 */

#ifndef ANALOG_STICK_SPLIT_PACKED_H
#define ANALOG_STICK_SPLIT_PACKED_H

#include <stdint.h>

#define ANALOG_STICK_PACKED_VERSION 1
#define ANALOG_STICK_MAX_STICKS     8

struct analog_stick_packed_entry {
    uint8_t reg;
    int8_t x;
    int8_t y;
} __packed;

/* Maximum packed buffer: version(1) + count(1) + entries(8*3) = 26 bytes */
#define ANALOG_STICK_PACKED_MAX_SIZE (2 + ANALOG_STICK_MAX_STICKS * 3)

#endif /* ANALOG_STICK_SPLIT_PACKED_H */
