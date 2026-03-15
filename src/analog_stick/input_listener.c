/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Mode-aware input listener for zmk,analog-stick.
 * Routes absolute axis input events to switch (key), mouse pointer,
 * or scroll mode based on the active ZMK keymap layer.
 */

#define DT_DRV_COMPAT zmk_analog_stick_listener

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_macro.h>

#include <zmk/endpoints.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>

#include <dt-bindings/analog-stick/modes.h>

#include <analog-stick/q16.h>
#include <analog-stick/hid_accumulator.h>

LOG_MODULE_REGISTER(zmk_analog_stick_listener, CONFIG_ZMK_ANALOG_STICK_LOG_LEVEL);

/* -------------------------------------------------------------------------- */
/* Per-layer configuration (from devicetree child nodes)                      */
/* -------------------------------------------------------------------------- */

struct layer_config {
    uint32_t layer_mask;
    uint8_t mode;

    /* Switch mode */
    uint32_t up_keycode;
    uint32_t down_keycode;
    uint32_t left_keycode;
    uint32_t right_keycode;
    int32_t switch_activation_point;
    int32_t switch_hysteresis;

    /* Mouse mode */
    int32_t mouse_min_speed;
    int32_t mouse_max_speed;

    /* Scroll mode */
    int32_t scroll_divisor;
};

struct listener_config {
    size_t layer_configs_len;
    const struct layer_config *layer_configs;
};

/* -------------------------------------------------------------------------- */
/* Per-instance runtime state                                                 */
/* -------------------------------------------------------------------------- */

/* Direction bitfield for switch mode */
#define DIR_UP    BIT(0)
#define DIR_DOWN  BIT(1)
#define DIR_LEFT  BIT(2)
#define DIR_RIGHT BIT(3)

struct listener_data {
    /* Current input values (absolute, [-127, 127]) */
    int16_t abs_x;
    int16_t abs_y;

    /* Active mode (cached from layer lookup) */
    uint8_t current_mode;
    const struct layer_config *current_layer_cfg;

    /* Switch mode state */
    uint8_t pressed_dirs; /* bitfield of currently pressed directions */

    /* Mouse mode state — Q16.16 sub-pixel accumulators */
    q16_t mouse_acc_x;
    q16_t mouse_acc_y;

    /* Scroll mode state — Q16.16 sub-tick accumulators */
    q16_t scroll_acc_x;
    q16_t scroll_acc_y;
};

/* -------------------------------------------------------------------------- */
/* Layer → mode resolution                                                    */
/* -------------------------------------------------------------------------- */

static const struct layer_config *resolve_layer_config(
        const struct listener_config *cfg) {
    /* Highest active layer with a matching config wins */
    const struct layer_config *best = NULL;
    int best_layer = -1;

    for (size_t i = 0; i < cfg->layer_configs_len; i++) {
        const struct layer_config *lc = &cfg->layer_configs[i];
        uint32_t mask = lc->layer_mask;
        uint8_t layer = 0;
        while (mask != 0) {
            if ((mask & BIT(0)) && zmk_keymap_layer_active(layer)) {
                if ((int)layer > best_layer) {
                    best_layer = (int)layer;
                    best = lc;
                }
            }
            layer++;
            mask >>= 1;
        }
    }

    return best;
}

/* -------------------------------------------------------------------------- */
/* Mode transition cleanup                                                    */
/* -------------------------------------------------------------------------- */

static void release_all_keys(struct listener_data *data,
                             const struct layer_config *old_cfg) {
    if (!old_cfg) return;

    if (data->pressed_dirs & DIR_UP) {
        zmk_hid_keyboard_release(old_cfg->up_keycode);
    }
    if (data->pressed_dirs & DIR_DOWN) {
        zmk_hid_keyboard_release(old_cfg->down_keycode);
    }
    if (data->pressed_dirs & DIR_LEFT) {
        zmk_hid_keyboard_release(old_cfg->left_keycode);
    }
    if (data->pressed_dirs & DIR_RIGHT) {
        zmk_hid_keyboard_release(old_cfg->right_keycode);
    }
    if (data->pressed_dirs != 0) {
        zmk_endpoint_send_report(HID_USAGE_KEY);
        data->pressed_dirs = 0;
    }
}

static void cleanup_mode(struct listener_data *data,
                         const struct layer_config *old_cfg,
                         uint8_t old_mode) {
    switch (old_mode) {
    case ANALOG_STICK_MODE_SWITCH:
        release_all_keys(data, old_cfg);
        break;
    case ANALOG_STICK_MODE_MOUSE:
        data->mouse_acc_x = 0;
        data->mouse_acc_y = 0;
        break;
    case ANALOG_STICK_MODE_SCROLL:
        data->scroll_acc_x = 0;
        data->scroll_acc_y = 0;
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Switch mode processing                                                     */
/* -------------------------------------------------------------------------- */

static void process_switch_mode(struct listener_data *data,
                                const struct listener_config *cfg_top,
                                const struct layer_config *lcfg,
                                int32_t activation, int32_t hysteresis) {
    int16_t x = data->abs_x;
    int16_t y = data->abs_y;
    int32_t act = activation;
    int32_t deact = activation - hysteresis;
    if (deact < 0) deact = 0;

    uint8_t new_dirs = 0;
    bool changed = false;

    /* Right: positive X */
    if (x > act || ((data->pressed_dirs & DIR_RIGHT) && x > deact)) {
        new_dirs |= DIR_RIGHT;
    }
    /* Left: negative X */
    if (x < -act || ((data->pressed_dirs & DIR_LEFT) && x < -deact)) {
        new_dirs |= DIR_LEFT;
    }
    /* Down: positive Y */
    if (y > act || ((data->pressed_dirs & DIR_DOWN) && y > deact)) {
        new_dirs |= DIR_DOWN;
    }
    /* Up: negative Y */
    if (y < -act || ((data->pressed_dirs & DIR_UP) && y < -deact)) {
        new_dirs |= DIR_UP;
    }

    /* Detect changes and emit key events */
    uint8_t pressed = new_dirs & ~data->pressed_dirs;
    uint8_t released = data->pressed_dirs & ~new_dirs;

    if (pressed & DIR_UP) {
        zmk_hid_keyboard_press(lcfg->up_keycode);
        changed = true;
    }
    if (pressed & DIR_DOWN) {
        zmk_hid_keyboard_press(lcfg->down_keycode);
        changed = true;
    }
    if (pressed & DIR_LEFT) {
        zmk_hid_keyboard_press(lcfg->left_keycode);
        changed = true;
    }
    if (pressed & DIR_RIGHT) {
        zmk_hid_keyboard_press(lcfg->right_keycode);
        changed = true;
    }

    if (released & DIR_UP) {
        zmk_hid_keyboard_release(lcfg->up_keycode);
        changed = true;
    }
    if (released & DIR_DOWN) {
        zmk_hid_keyboard_release(lcfg->down_keycode);
        changed = true;
    }
    if (released & DIR_LEFT) {
        zmk_hid_keyboard_release(lcfg->left_keycode);
        changed = true;
    }
    if (released & DIR_RIGHT) {
        zmk_hid_keyboard_release(lcfg->right_keycode);
        changed = true;
    }

    if (changed) {
        zmk_endpoint_send_report(HID_USAGE_KEY);
    }

    data->pressed_dirs = new_dirs;
}

/* -------------------------------------------------------------------------- */
/* Mouse pointer mode processing                                              */
/* -------------------------------------------------------------------------- */

static void process_mouse_mode(struct listener_data *data,
                               const struct layer_config *lcfg) {
    int16_t x = data->abs_x;
    int16_t y = data->abs_y;

    if (x == 0 && y == 0) {
        data->mouse_acc_x = 0;
        data->mouse_acc_y = 0;
        return;
    }

    /* Compute deflection magnitude in Q16.16
     * magnitude = sqrt(x^2 + y^2), but we use a fast integer approximation:
     * mag ≈ max(|x|, |y|) + min(|x|, |y|) * 3/8
     * Good enough for speed mapping, avoids sqrt entirely. */
    int32_t ax = (x < 0) ? -x : x;
    int32_t ay = (y < 0) ? -y : y;
    int32_t mag = (ax > ay) ? (ax + ((ay * 3) >> 3))
                            : (ay + ((ax * 3) >> 3));

    /* mag is in [0, ~170] for inputs in [-127, 127] range */
    if (mag > 127) mag = 127;

    /* Linear speed ramp: min_speed at mag=1, max_speed at mag=127 */
    int32_t min_spd = lcfg->mouse_min_speed;
    int32_t max_spd = lcfg->mouse_max_speed;
    /* speed in Q16.16 — cast to uint32_t before shifting to avoid
     * signed left-shift UB when value >= 32768 */
    q16_t speed;
    if (mag <= 1) {
        speed = (q16_t)((uint32_t)min_spd << Q16_SHIFT);
    } else {
        /* Linear interpolation */
        speed = (q16_t)((uint32_t)min_spd << Q16_SHIFT) +
                (q16_t)(((int64_t)(max_spd - min_spd) * (mag - 1) << Q16_SHIFT) / 126);
    }

    /* Decompose into X/Y components preserving angle */
    q16_t dx = q16_mul(speed, (q16_t)((int32_t)x << Q16_SHIFT) / (mag > 0 ? mag : 1));
    q16_t dy = q16_mul(speed, (q16_t)((int32_t)y << Q16_SHIFT) / (mag > 0 ? mag : 1));

    /* Accumulate sub-pixel fractions (saturating to prevent overflow) */
    data->mouse_acc_x = q16_sat_add(data->mouse_acc_x, dx);
    data->mouse_acc_y = q16_sat_add(data->mouse_acc_y, dy);

    /* Extract integer pixels */
    int16_t px = (int16_t)(data->mouse_acc_x >> Q16_SHIFT);
    int16_t py = (int16_t)(data->mouse_acc_y >> Q16_SHIFT);

    /* Keep fractional remainder */
    data->mouse_acc_x -= (q16_t)px << Q16_SHIFT;
    data->mouse_acc_y -= (q16_t)py << Q16_SHIFT;

    if (px != 0 || py != 0) {
        zmk_analog_stick_hid_move_add(px, py);
#if IS_ENABLED(CONFIG_ZMK_ANALOG_STICK_HID_FLUSH_PER_STICK)
        zmk_analog_stick_hid_flush();
#endif
    }
}

/* -------------------------------------------------------------------------- */
/* Mouse scroll mode processing                                               */
/* -------------------------------------------------------------------------- */

static void process_scroll_mode(struct listener_data *data,
                                const struct layer_config *lcfg) {
    int16_t x = data->abs_x;
    int16_t y = data->abs_y;

    if (x == 0 && y == 0) {
        data->scroll_acc_x = 0;
        data->scroll_acc_y = 0;
        return;
    }

    int32_t divisor = lcfg->scroll_divisor;
    if (divisor <= 0) divisor = 4;

    /* Scale deflection by divisor to get scroll rate.
     * At max deflection (127), rate = 127/divisor ticks per poll.
     * At min deflection, rate is tiny and accumulates over cycles. */
    q16_t rate_x = ((int32_t)x << Q16_SHIFT) / divisor;
    q16_t rate_y = ((int32_t)y << Q16_SHIFT) / divisor;

    /* Negate Y for natural scroll direction (stick up = scroll up) */
    rate_y = -rate_y;

    data->scroll_acc_x += rate_x;
    data->scroll_acc_y += rate_y;

    int16_t sx = (int16_t)(data->scroll_acc_x >> Q16_SHIFT);
    int16_t sy = (int16_t)(data->scroll_acc_y >> Q16_SHIFT);

    data->scroll_acc_x -= (q16_t)sx << Q16_SHIFT;
    data->scroll_acc_y -= (q16_t)sy << Q16_SHIFT;

    if (sx != 0 || sy != 0) {
        zmk_analog_stick_hid_scroll_add(sx, sy);
#if IS_ENABLED(CONFIG_ZMK_ANALOG_STICK_HID_FLUSH_PER_STICK)
        zmk_analog_stick_hid_flush();
#endif
    }
}

/* -------------------------------------------------------------------------- */
/* Input event handler                                                        */
/* -------------------------------------------------------------------------- */

static void input_handler(const struct listener_config *config,
                          struct listener_data *data,
                          struct input_event *evt) {
    /* Accumulate axis values */
    if (evt->type == INPUT_EV_ABS) {
        switch (evt->code) {
        case INPUT_ABS_X:
            data->abs_x = (int16_t)evt->value;
            break;
        case INPUT_ABS_Y:
            data->abs_y = (int16_t)evt->value;
            break;
        default:
            return;
        }
    }

    /* Only process on sync */
    if (!evt->sync) {
        return;
    }

    /* Resolve current mode from active layers */
    const struct layer_config *new_cfg = resolve_layer_config(config);
    if (!new_cfg) {
        return; /* No matching layer config */
    }

    uint8_t new_mode = new_cfg->mode;

    /* Handle mode transition */
    if (new_mode != data->current_mode || new_cfg != data->current_layer_cfg) {
        cleanup_mode(data, data->current_layer_cfg, data->current_mode);
        data->current_mode = new_mode;
        data->current_layer_cfg = new_cfg;
    }

    /* Dispatch to mode handler */
    switch (data->current_mode) {
    case ANALOG_STICK_MODE_SWITCH:
        process_switch_mode(data, config, new_cfg,
                            new_cfg->switch_activation_point,
                            new_cfg->switch_hysteresis);
        break;
    case ANALOG_STICK_MODE_MOUSE:
        process_mouse_mode(data, new_cfg);
        break;
    case ANALOG_STICK_MODE_SCROLL:
        process_scroll_mode(data, new_cfg);
        break;
    default:
        LOG_WRN("Unknown mode %d", data->current_mode);
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Device instantiation macros                                                */
/* -------------------------------------------------------------------------- */

#define OVERRIDE_LAYER_BIT(node, prop, idx) BIT(DT_PROP_BY_IDX(node, prop, idx))

#define CHILD_LAYER_CONFIG(node)                                               \
    {                                                                          \
        .layer_mask = DT_FOREACH_PROP_ELEM_SEP(                                \
            node, layers, OVERRIDE_LAYER_BIT, (|)),                            \
        .mode = DT_PROP(node, mode),                                           \
        .up_keycode = DT_PROP(node, up_keycode),                               \
        .down_keycode = DT_PROP(node, down_keycode),                           \
        .left_keycode = DT_PROP(node, left_keycode),                           \
        .right_keycode = DT_PROP(node, right_keycode),                         \
        .switch_activation_point = DT_PROP(node, switch_activation_point),   \
        .switch_hysteresis = DT_PROP(node, switch_hysteresis),               \
        .mouse_min_speed = DT_PROP(node, mouse_min_speed),                     \
        .mouse_max_speed = DT_PROP(node, mouse_max_speed),                     \
        .scroll_divisor = DT_PROP(node, scroll_divisor),                       \
    },

#define IL_ONE(...) +1

#define CHILD_LAYER_VALIDATE(node)                                             \
    BUILD_ASSERT(DT_PROP(node, mouse_min_speed) <= 32767,                      \
                 "mouse-min-speed must be <= 32767 to avoid Q16 overflow");    \
    BUILD_ASSERT(DT_PROP(node, mouse_max_speed) <= 32767,                      \
                 "mouse-max-speed must be <= 32767 to avoid Q16 overflow");

#define LISTENER_INST(n)                                                       \
    DT_INST_FOREACH_CHILD(n, CHILD_LAYER_VALIDATE)                                                       \
    static const struct layer_config layer_configs_##n[] = {                    \
        DT_INST_FOREACH_CHILD(n, CHILD_LAYER_CONFIG)                           \
    };                                                                         \
                                                                               \
    static const struct listener_config listener_config_##n = {                \
        .layer_configs_len = (0 DT_INST_FOREACH_CHILD(n, IL_ONE)),             \
        .layer_configs = layer_configs_##n,                                     \
    };                                                                         \
                                                                               \
    static struct listener_data listener_data_##n = {};                        \
                                                                               \
    static void analog_stick_input_handler_##n(struct input_event *evt,        \
                                               void *user_data) {             \
        input_handler(&listener_config_##n, &listener_data_##n, evt);          \
    }                                                                          \
    INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, device)),           \
                          analog_stick_input_handler_##n, NULL);

DT_INST_FOREACH_STATUS_OKAY(LISTENER_INST)
