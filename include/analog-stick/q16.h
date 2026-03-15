/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Q16.16 fixed-point arithmetic helpers shared between the analog stick
 * driver and its input listener.
 */

#ifndef ANALOG_STICK_Q16_H
#define ANALOG_STICK_Q16_H

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

BUILD_ASSERT(((int64_t)(-1) >> 1) == (int64_t)(-1),
             "q16_mul requires arithmetic right-shift for int64_t");

typedef int32_t q16_t;

#define Q16_SHIFT 16
#define Q16_ONE   ((q16_t)(1U << Q16_SHIFT))
#define Q16_MAX   INT32_MAX
#define Q16_MIN   INT32_MIN

static inline q16_t q16_from_int(int32_t v) {
    if (v > 32767) return (q16_t)(32767U << Q16_SHIFT);
    if (v < -32767) return (q16_t)((uint32_t)(int32_t)-32767 << Q16_SHIFT);
    return (q16_t)((uint32_t)(int32_t)v << Q16_SHIFT);
}

static inline q16_t q16_mul(q16_t a, q16_t b) {
    int64_t r = ((int64_t)a * b) >> Q16_SHIFT;
    if (r > INT32_MAX) return INT32_MAX;
    if (r < INT32_MIN) return INT32_MIN;
    return (q16_t)r;
}

static inline q16_t q16_div(q16_t a, q16_t b) {
    if (b == 0) {
        return (a >= 0) ? INT32_MAX : INT32_MIN;
    }
    int64_t r = ((int64_t)a << Q16_SHIFT) / b;
    if (r > INT32_MAX) return INT32_MAX;
    if (r < INT32_MIN) return INT32_MIN;
    return (q16_t)r;
}

static inline q16_t q16_neg(q16_t x) {
    return (x == Q16_MIN) ? Q16_MAX : -x;
}

static inline q16_t q16_sat_add(q16_t a, q16_t b) {
    int64_t r = (int64_t)a + b;
    if (r > INT32_MAX) return INT32_MAX;
    if (r < INT32_MIN) return INT32_MIN;
    return (q16_t)r;
}

#endif /* ANALOG_STICK_Q16_H */
