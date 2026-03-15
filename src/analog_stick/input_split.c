/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Split transport for zmk,analog-stick.
 * Peripheral: subscribes to analog stick input events and forwards
 *             via ZMK split transport. Supports both per-event legacy
 *             mode and packed coalesced mode.
 * Central:    receives forwarded events and re-emits on a proxy device.
 */

#define DT_DRV_COMPAT zmk_analog_stick_split

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <analog-stick/split_packed.h>

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

#if IS_ENABLED(CONFIG_ZMK_ANALOG_STICK_SPLIT_PACKED)
int zmk_analog_stick_split_unpack_and_emit(const uint8_t *buf, size_t len) {
    if (len < 2) {
        LOG_WRN("Packed buffer too short: %zu", len);
        return -EINVAL;
    }

    uint8_t version = buf[0];
    uint8_t count = buf[1];

    if (version != ANALOG_STICK_PACKED_VERSION) {
        LOG_WRN("Packed transport version mismatch: got %d, expected %d",
                version, ANALOG_STICK_PACKED_VERSION);
        return -ENOTSUP;
    }

    if (count > ANALOG_STICK_MAX_STICKS || len < 2 + count * 3) {
        LOG_WRN("Invalid packed entry count: %d", count);
        return -EINVAL;
    }

    const uint8_t *p = buf + 2;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t reg = p[0];
        int8_t x = (int8_t)p[1];
        int8_t y = (int8_t)p[2];
        p += 3;

        for (size_t j = 0; j < ARRAY_SIZE(proxy_inputs); j++) {
            if (reg == proxy_inputs[j].reg) {
                input_report(proxy_inputs[j].dev, INPUT_EV_ABS,
                             INPUT_ABS_X, x, false, K_NO_WAIT);
                input_report(proxy_inputs[j].dev, INPUT_EV_ABS,
                             INPUT_ABS_Y, y, true, K_NO_WAIT);
                break;
            }
        }
    }

    return 0;
}
#endif /* CONFIG_ZMK_ANALOG_STICK_SPLIT_PACKED */

#define SPLIT_INST(n) \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL, \
                          CONFIG_ZMK_ANALOG_STICK_INIT_PRIORITY, NULL);

#else

/* -------------------------------------------------------------------------- */
/* Peripheral role: subscribe to input events and forward over BLE            */
/* -------------------------------------------------------------------------- */

#include <zmk/split/peripheral.h>

/* --- Packed transport coalescing state --- */

#if IS_ENABLED(CONFIG_ZMK_ANALOG_STICK_SPLIT_PACKED)

static struct {
    uint8_t reg;
    int8_t x;
    int8_t y;
    bool dirty;
    bool x_updated;
    bool y_updated;
} stick_state[ANALOG_STICK_MAX_STICKS];

void analog_stick_split_update_state(uint8_t reg, int8_t x, int8_t y) {
    uint8_t idx = reg % ANALOG_STICK_MAX_STICKS;
    stick_state[idx].reg = reg;
    stick_state[idx].x = x;
    stick_state[idx].y = y;
    stick_state[idx].dirty = true;
    stick_state[idx].x_updated = false;
    stick_state[idx].y_updated = false;
}

void analog_stick_split_coalesce_and_send(void) {
    uint8_t buf[ANALOG_STICK_PACKED_MAX_SIZE];
    uint8_t count = 0;
    uint8_t *p = buf + 2; /* skip header */

    for (uint8_t i = 0; i < ANALOG_STICK_MAX_STICKS; i++) {
        if (stick_state[i].dirty) {
            p[0] = stick_state[i].reg;
            p[1] = (uint8_t)stick_state[i].x;
            p[2] = (uint8_t)stick_state[i].y;
            p += 3;
            count++;
            stick_state[i].dirty = false;
        }
    }

    if (count == 0) {
        return;
    }

    buf[0] = ANALOG_STICK_PACKED_VERSION;
    buf[1] = count;

    /* Send as a packed split event */
    struct zmk_split_transport_peripheral_event ev = {
        .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT,
        .data = {.input_event = {
                     .reg = 0xFF, /* sentinel: packed event marker */
                     .type = INPUT_EV_ABS,
                     .code = 0xFF, /* sentinel */
                     .value = (int32_t)count,
                     .sync = true,
                 }}};
    int ret = zmk_split_peripheral_report_event(&ev);
    if (ret < 0) {
        LOG_ERR("Packed split forward failed: %d", ret);
    }
}

/* Packed peripheral input handler: buffer events for coalescing */
#define SPLIT_PACKED_HANDLER(n)                                                \
    static void analog_stick_split_packed_handler_##n(                         \
            struct input_event *evt, void *user_data) {                        \
        static int8_t pending_x_##n;                                           \
        static int8_t pending_y_##n;                                           \
        if (evt->type == INPUT_EV_ABS) {                                       \
            if (evt->code == INPUT_ABS_X) {                                    \
                pending_x_##n = (int8_t)evt->value;                            \
            } else if (evt->code == INPUT_ABS_Y) {                             \
                pending_y_##n = (int8_t)evt->value;                            \
            }                                                                  \
        }                                                                      \
        if (evt->sync) {                                                       \
            analog_stick_split_update_state(                                   \
                DT_INST_REG_ADDR(n), pending_x_##n, pending_y_##n);            \
        }                                                                      \
    }                                                                          \
    INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, device)),           \
                          analog_stick_split_packed_handler_##n, NULL);

#define SPLIT_INST(n) SPLIT_PACKED_HANDLER(n)

#else /* !CONFIG_ZMK_ANALOG_STICK_SPLIT_PACKED */

/* Stub implementations when packed transport is not enabled */
void analog_stick_split_update_state(uint8_t reg, int8_t x, int8_t y) {
    (void)reg; (void)x; (void)y;
}

void analog_stick_split_coalesce_and_send(void) {}

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

#endif /* CONFIG_ZMK_ANALOG_STICK_SPLIT_PACKED */

#endif /* CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

DT_INST_FOREACH_STATUS_OKAY(SPLIT_INST)
