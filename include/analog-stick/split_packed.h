/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Packed split transport format for analog stick events.
 * Coalesces all stick axis changes into one BLE notification per scan cycle.
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

/**
 * Update coalescing state for a stick (peripheral side).
 * Called by the input event handler instead of immediate BLE send.
 */
void analog_stick_split_update_state(uint8_t reg, int8_t x, int8_t y);

/**
 * Pack all dirty stick states into one buffer and send over BLE.
 * Called by the scan coordinator at end of each scan cycle.
 */
void analog_stick_split_coalesce_and_send(void);

#endif /* ANALOG_STICK_SPLIT_PACKED_H */
