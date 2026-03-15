/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Split transport for zmk,analog-stick.
 * Peripheral: subscribes to analog stick input events and forwards
 *             via ZMK split transport (per-event).
 * Central:    receives forwarded events and re-emits on a proxy device.
 */

#define DT_DRV_COMPAT zmk_analog_stick_split

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk_analog_stick, CONFIG_ZMK_ANALOG_STICK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

/* -------------------------------------------------------------------------- */
/* Central role: receive events from peripheral, re-emit on proxy device      */
/* -------------------------------------------------------------------------- */

struct split_proxy_entry {
    uint8_t reg;
    const struct device *dev;
};

#define SPLIT_PROXY_ENTRY(n) \
    { .reg = DT_INST_REG_ADDR(n), .dev = DEVICE_DT_GET(DT_DRV_INST(n)) },

static const struct split_proxy_entry proxy_inputs[] = {
    DT_INST_FOREACH_STATUS_OKAY(SPLIT_PROXY_ENTRY)
};

int zmk_analog_stick_split_report_peripheral_event(uint8_t reg, uint8_t type,
                                                    uint16_t code, int32_t value,
                                                    bool sync) {
    for (size_t i = 0; i < ARRAY_SIZE(proxy_inputs); i++) {
        if (reg == proxy_inputs[i].reg) {
            return input_report(proxy_inputs[i].dev, type, code, value, sync,
                                K_NO_WAIT);
        }
    }
    return -ENODEV;
}

#define SPLIT_INST(n) \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL, \
                          CONFIG_ZMK_ANALOG_STICK_INIT_PRIORITY, NULL);

#else

/* -------------------------------------------------------------------------- */
/* Peripheral role: subscribe to input events and forward over BLE            */
/* -------------------------------------------------------------------------- */

#include <zmk/split/peripheral.h>

/* Legacy per-event peripheral handler */
#define SPLIT_INST(n)                                                          \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(n, device),                             \
                 "Peripheral input splits need a `device` property");          \
    static void analog_stick_split_handler_##n(struct input_event *evt,        \
                                                void *user_data) {            \
        struct zmk_split_transport_peripheral_event ev = {                     \
            .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT,     \
            .data = {.input_event = {                                          \
                         .reg = DT_INST_REG_ADDR(n),                           \
                         .type = evt->type,                                    \
                         .code = evt->code,                                    \
                         .value = evt->value,                                  \
                         .sync = evt->sync,                                    \
                     }}};                                                      \
        int ret = zmk_split_peripheral_report_event(&ev);                      \
        if (ret < 0) {                                                         \
            LOG_ERR("Split forward failed: %d", ret);                          \
        }                                                                      \
    }                                                                          \
    INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, device)),           \
                          analog_stick_split_handler_##n, NULL);

#endif /* CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

DT_INST_FOREACH_STATUS_OKAY(SPLIT_INST)
