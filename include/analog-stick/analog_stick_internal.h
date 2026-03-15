/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Internal API for the analog stick driver, used by the scan coordinator.
 * Not part of the public module API.
 */

#ifndef ANALOG_STICK_INTERNAL_H
#define ANALOG_STICK_INTERNAL_H

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/**
 * Read ADC values for a stick instance. Stores results in internal
 * data fields (raw_x, raw_y). Does NOT assert/deassert enable GPIO —
 * that is the coordinator's responsibility.
 *
 * @return 0 on success, negative errno on failure.
 */
int analog_stick_read_adc(const struct device *dev);

/**
 * Process a stick instance: run IIR filter, rescale, detect changes,
 * and emit input_report() events. Must be called after analog_stick_read_adc().
 */
void analog_stick_process_and_emit(const struct device *dev);

/**
 * Access the driver's enable GPIO spec for GPIO grouping.
 * Returns NULL if no enable GPIO is configured.
 */
const struct gpio_dt_spec *analog_stick_get_enable_gpio(const struct device *dev);

/** Returns true if the stick has an enable GPIO configured. */
bool analog_stick_has_enable_gpio(const struct device *dev);

/** Returns true if pulse-read is configured for this stick. */
bool analog_stick_has_pulse_read(const struct device *dev);

/** Returns the settling time in microseconds. */
uint32_t analog_stick_get_settling_us(const struct device *dev);

/** Returns true if the stick is currently active (non-zero output). */
bool analog_stick_is_active(const struct device *dev);

/** Returns the configured active poll period in ms. */
int32_t analog_stick_get_wait_period_active(const struct device *dev);

/** Returns the configured idle poll period in ms. */
int32_t analog_stick_get_wait_period_idle(const struct device *dev);

/** The shared work queue used by the analog stick module. */
extern struct k_work_q analog_stick_wq;

/** Ensure the shared work queue is started. */
void analog_stick_ensure_wq(void);

#endif /* ANALOG_STICK_INTERNAL_H */
