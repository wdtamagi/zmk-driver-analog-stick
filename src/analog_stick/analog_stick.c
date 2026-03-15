/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Multi-mode analog stick driver for ZMK.
 * Reads analog position sensors via Zephyr ADC io-channels and emits
 * Zephyr input events (INPUT_EV_ABS). Mode-aware processing is
 * handled by a separate input listener.
 *
 * ADC reading and event emission are factored into per-stick functions
 * called by the scan coordinator (scan_coordinator.c).
 */

#define DT_DRV_COMPAT zmk_analog_stick

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <string.h>

#include <analog-stick/q16.h>
#include <analog-stick/analog_stick_internal.h>

LOG_MODULE_REGISTER(zmk_analog_stick, CONFIG_ZMK_ANALOG_STICK_LOG_LEVEL);

BUILD_ASSERT(sizeof(float) == sizeof(uint32_t),
             "Filter coefficient parsing requires 32-bit floats");

BUILD_ASSERT(((-1) >> 1) == -1,
             "This driver requires arithmetic right-shift for signed integers");

/* -------------------------------------------------------------------------- */
/* Q16.16 helpers local to the driver (not shared)                            */
/* -------------------------------------------------------------------------- */

static inline q16_t q16_abs(q16_t x) {
    if (x == Q16_MIN) return Q16_MAX;
    return x < 0 ? -x : x;
}

static inline q16_t q16_clamp(q16_t x, q16_t lo, q16_t hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline q16_t q16_from_float(float f) {
    if (f != f) return 0; /* NaN */
    float product = f * (float)Q16_ONE;
    if (product >= 2147483648.0f) return Q16_MAX;
    if (product < (float)Q16_MIN) return Q16_MIN;
    return (q16_t)product;
}

static inline bool q16_is_valid(q16_t x) {
    return x != Q16_MIN && x != Q16_MAX;
}

/* -------------------------------------------------------------------------- */
/* IIR biquad filter (Direct Form II Transposed, Q16.16)                      */
/* -------------------------------------------------------------------------- */

struct biquad_state {
    q16_t z1;
    q16_t z2;
};

struct biquad_coeffs {
    q16_t b0, b1, b2;
    q16_t a1, a2;
};

static q16_t biquad_process(struct biquad_state *state,
                            const struct biquad_coeffs *c, q16_t input) {
    q16_t y = q16_sat_add(q16_mul(c->b0, input), state->z1);
    state->z1 = q16_sat_add(q16_sat_add(q16_mul(c->b1, input),
                             q16_neg(q16_mul(c->a1, y))), state->z2);
    state->z2 = q16_sat_add(q16_mul(c->b2, input), q16_neg(q16_mul(c->a2, y)));
    if (!q16_is_valid(y)) {
        state->z1 = 0;
        state->z2 = 0;
        return input;
    }
    return y;
}

#define BIQUAD_PRIME_ITERATIONS 20

static void biquad_prime(struct biquad_state *state,
                         const struct biquad_coeffs *c, q16_t value) {
    for (int i = 0; i < BIQUAD_PRIME_ITERATIONS; i++) {
        biquad_process(state, c, value);
    }
}

/* -------------------------------------------------------------------------- */
/* Dedicated work queue (shared with scan coordinator)                        */
/* -------------------------------------------------------------------------- */

static K_THREAD_STACK_DEFINE(analog_stick_wq_stack,
                             CONFIG_ZMK_ANALOG_STICK_WORK_QUEUE_STACK_SIZE);
struct k_work_q analog_stick_wq;
static atomic_t wq_started = ATOMIC_INIT(0);

void analog_stick_ensure_wq(void) {
    if (atomic_cas(&wq_started, 0, 1)) {
        k_work_queue_init(&analog_stick_wq);
        k_work_queue_start(&analog_stick_wq, analog_stick_wq_stack,
                           K_THREAD_STACK_SIZEOF(analog_stick_wq_stack),
                           K_PRIO_COOP(7), NULL);
        atomic_set(&wq_started, 2);
    } else {
        while (atomic_get(&wq_started) != 2) {
            k_yield();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Config and data structures                                                 */
/* -------------------------------------------------------------------------- */

struct axis_config {
    struct adc_dt_spec adc;
    int32_t min;
    int32_t center;
    int32_t max;
    bool invert;
};

struct analog_stick_config {
    struct gpio_dt_spec enable_gpio;
    bool has_enable_gpio;
    bool pulse_read;
    int32_t read_turn_on_time;
    int32_t wait_period_idle;
    int32_t wait_period_active;

    struct axis_config x;
    bool has_y;
    struct axis_config y;

    int32_t deadzone;
    int32_t deadzone_percent;
    bool has_deadzone_percent;
    int32_t inter_channel_settling_us;

    bool has_filter;
    uint32_t filter_coeffs_raw[6];
};

struct analog_stick_data {
    const struct device *dev;

    struct adc_sequence seq_x;
    struct adc_sequence seq_y;

    int32_t adc_buf[2]; /* [0]=X, [1]=Y — inlined, no DMA section needed */

    /* Raw ADC values from last read (Q16.16), set by analog_stick_read_adc() */
    q16_t raw_x;
    q16_t raw_y;
    int read_err; /* 0 on success, negative errno on ADC failure */

    struct biquad_state filter_x;
    struct biquad_state filter_y;
    struct biquad_coeffs coeffs;

    /* Pre-computed normalization scale for filter (computed from ADC resolution) */
    q16_t inv_full_scale;
    q16_t full_scale_q16;

    /* Pre-computed reciprocals for hot path rescaling */
    q16_t inv_range_neg_x;
    q16_t inv_range_pos_x;
    q16_t inv_range_neg_y;
    q16_t inv_range_pos_y;

    /* Previous output for change detection */
    int16_t prev_x;
    int16_t prev_y;

    bool active;
    bool primed;

    int32_t effective_deadzone; /* computed at init; used by rescale pipeline */
};

/* -------------------------------------------------------------------------- */
/* Accessor functions for scan coordinator                                    */
/* -------------------------------------------------------------------------- */

const struct gpio_dt_spec *analog_stick_get_enable_gpio(const struct device *dev) {
    const struct analog_stick_config *cfg = dev->config;
    return cfg->has_enable_gpio ? &cfg->enable_gpio : NULL;
}

bool analog_stick_has_enable_gpio(const struct device *dev) {
    const struct analog_stick_config *cfg = dev->config;
    return cfg->has_enable_gpio;
}

bool analog_stick_has_pulse_read(const struct device *dev) {
    const struct analog_stick_config *cfg = dev->config;
    return cfg->pulse_read;
}

uint32_t analog_stick_get_settling_us(const struct device *dev) {
    const struct analog_stick_config *cfg = dev->config;
    return (uint32_t)cfg->read_turn_on_time;
}

bool analog_stick_is_active(const struct device *dev) {
    struct analog_stick_data *data = dev->data;
    return data->active;
}

int32_t analog_stick_get_wait_period_active(const struct device *dev) {
    const struct analog_stick_config *cfg = dev->config;
    return cfg->wait_period_active;
}

int32_t analog_stick_get_wait_period_idle(const struct device *dev) {
    const struct analog_stick_config *cfg = dev->config;
    return cfg->wait_period_idle;
}

/* -------------------------------------------------------------------------- */
/* Axis rescaling                                                             */
/* -------------------------------------------------------------------------- */

static q16_t rescale_axis(q16_t adc_val, const struct axis_config *ax,
                          int32_t deadzone, q16_t inv_neg, q16_t inv_pos) {
    q16_t center = q16_from_int(ax->center);
    q16_t diff = q16_sat_add(adc_val, q16_neg(center));
    q16_t dz = q16_from_int(deadzone);

    if (q16_abs(diff) < dz) {
        return 0;
    }

    q16_t scaled;
    if (diff < 0) {
        if (inv_neg == 0) return 0;
        scaled = q16_mul(q16_sat_add(diff, dz), inv_neg);
    } else {
        if (inv_pos == 0) return 0;
        scaled = q16_mul(q16_sat_add(diff, q16_neg(dz)), inv_pos);
    }

    scaled = q16_clamp(scaled, -Q16_ONE, Q16_ONE);

    if (ax->invert) {
        scaled = q16_neg(scaled);
    }

    return scaled;
}

/* -------------------------------------------------------------------------- */
/* Rail detection                                                             */
/* -------------------------------------------------------------------------- */

#define ADC_RAIL_MARGIN 15 /* noise margin: 15 LSB avoids false rail on noisy PCBs */

static bool adc_at_rail(int32_t value, uint8_t resolution, uint8_t oversampling) {
    if (resolution == 0 || resolution > 16) {
        return false;
    }
    int32_t norm = (oversampling > 0) ? (value >> oversampling) : value;
    int32_t full_scale = (int32_t)((1U << resolution) - 1);
    int32_t margin = ADC_RAIL_MARGIN;
    bool at_high = norm > (full_scale - margin);
    bool at_low = (norm >= 0) ? (norm < margin)
                              : (norm < (-(int32_t)(1U << resolution) + margin));
    return at_high || at_low;
}

/* -------------------------------------------------------------------------- */
/* Per-stick ADC read (called by scan coordinator)                            */
/* -------------------------------------------------------------------------- */

int analog_stick_read_adc(const struct device *dev) {
    struct analog_stick_data *data = dev->data;
    const struct analog_stick_config *cfg = dev->config;

    data->raw_x = 0;
    data->raw_y = 0;
    data->read_err = 0;

    int err = adc_read(cfg->x.adc.dev, &data->seq_x);
    if (err) {
        LOG_ERR("X-axis ADC read failed: %d", err);
        data->read_err = err;
        return err;
    }

    int32_t adc_x = data->adc_buf[0];
    if (adc_at_rail(adc_x, cfg->x.adc.resolution, cfg->x.adc.oversampling)) {
        LOG_DBG("X-axis at rail (%d)", adc_x);
        adc_x = cfg->x.center;
    }
    data->raw_x = q16_from_int(adc_x);

    if (cfg->has_y) {
        if (cfg->inter_channel_settling_us <= 50) {
            k_busy_wait((uint32_t)cfg->inter_channel_settling_us);
        } else {
            k_sleep(K_USEC((uint32_t)cfg->inter_channel_settling_us));
        }
        err = adc_read(cfg->y.adc.dev, &data->seq_y);
        if (err) {
            LOG_ERR("Y-axis ADC read failed: %d", err);
            data->read_err = err;
            return err;
        }
        int32_t adc_y = data->adc_buf[1];
        if (adc_at_rail(adc_y, cfg->y.adc.resolution, cfg->y.adc.oversampling)) {
            LOG_DBG("Y-axis at rail (%d)", adc_y);
            adc_y = cfg->y.center;
        }
        data->raw_y = q16_from_int(adc_y);
    }

    return 0;
}

/* -------------------------------------------------------------------------- */
/* Per-stick process and emit (called by scan coordinator)                    */
/* -------------------------------------------------------------------------- */

void analog_stick_process_and_emit(const struct device *dev) {
    struct analog_stick_data *data = dev->data;
    const struct analog_stick_config *cfg = dev->config;

    if (data->read_err) {
        return;
    }

    q16_t raw_x = data->raw_x;
    q16_t raw_y = data->raw_y;

    /* --- IIR filter --- */
    if (cfg->has_filter) {
        q16_t norm_x = q16_mul(raw_x, data->inv_full_scale);
        q16_t norm_y = cfg->has_y ? q16_mul(raw_y, data->inv_full_scale) : 0;
        if (!data->primed) {
            biquad_prime(&data->filter_x, &data->coeffs, norm_x);
            if (cfg->has_y) {
                biquad_prime(&data->filter_y, &data->coeffs, norm_y);
            }
            data->primed = true;
        }
        q16_t filtered_norm_x = biquad_process(&data->filter_x, &data->coeffs, norm_x);
        raw_x = q16_mul(filtered_norm_x, data->full_scale_q16);
        if (cfg->has_y) {
            q16_t filtered_norm_y = biquad_process(&data->filter_y, &data->coeffs, norm_y);
            raw_y = q16_mul(filtered_norm_y, data->full_scale_q16);
        }
    }
    /* --- Rescale --- */
    q16_t scaled_x = rescale_axis(raw_x, &cfg->x, data->effective_deadzone,
                                  data->inv_range_neg_x, data->inv_range_pos_x);
    q16_t scaled_y = cfg->has_y
        ? rescale_axis(raw_y, &cfg->y, data->effective_deadzone,
                       data->inv_range_neg_y, data->inv_range_pos_y)
        : 0;

    /* --- Map to output range [-127, 127] --- */
    int16_t out_x = (int16_t)(((int32_t)scaled_x * 127) >> Q16_SHIFT);
    int16_t out_y = (int16_t)(((int32_t)scaled_y * 127) >> Q16_SHIFT);

    /* --- Emit input events on change --- */
    bool x_changed = (out_x != data->prev_x);
    bool y_changed = (out_y != data->prev_y);

    if (x_changed || y_changed) {
        if (cfg->has_y) {
            if (x_changed) {
                input_report_abs(dev, INPUT_ABS_X, out_x, !y_changed, K_NO_WAIT);
            }
            if (y_changed) {
                input_report_abs(dev, INPUT_ABS_Y, out_y, true, K_NO_WAIT);
            }
        } else {
            input_report_abs(dev, INPUT_ABS_X, out_x, true, K_NO_WAIT);
        }
        data->prev_x = out_x;
        data->prev_y = out_y;
    }

    /* --- Activity detection --- */
    data->active = (out_x != 0 || out_y != 0);
}

/* -------------------------------------------------------------------------- */
/* Init helpers                                                               */
/* -------------------------------------------------------------------------- */

static inline float absf(float x) { return x < 0.0f ? -x : x; }

static inline bool float_is_finite_raw(uint32_t raw) {
    return ((raw >> 23) & 0xFF) != 0xFF;
}

static int setup_adc(struct analog_stick_data *data,
                     const struct analog_stick_config *cfg) {
    int err = adc_channel_setup_dt(&cfg->x.adc);
    if (err) {
        LOG_ERR("X-axis ADC channel setup failed: %d", err);
        return err;
    }
    adc_sequence_init_dt(&cfg->x.adc, &data->seq_x);
    data->seq_x.buffer = &data->adc_buf[0];
    data->seq_x.buffer_size = sizeof(data->adc_buf[0]);

    if (cfg->has_y) {
        err = adc_channel_setup_dt(&cfg->y.adc);
        if (err) {
            LOG_ERR("Y-axis ADC channel setup failed: %d", err);
            return err;
        }
        adc_sequence_init_dt(&cfg->y.adc, &data->seq_y);
        data->seq_y.buffer = &data->adc_buf[1];
        data->seq_y.buffer_size = sizeof(data->adc_buf[1]);
    }

    return 0;
}

static int parse_filter_coeffs(struct analog_stick_data *data,
                               const struct analog_stick_config *cfg) {
    if (!cfg->has_filter) {
        return 0;
    }

    float f[6];
    for (int i = 0; i < 6; i++) {
        memcpy(&f[i], &cfg->filter_coeffs_raw[i], sizeof(float));
    }

    if (absf(f[3] - 1.0f) > 1e-6f) {
        LOG_ERR("Filter a0 must be 1.0 (normalized)");
        return -EINVAL;
    }

    for (int i = 0; i < 6; i++) {
        if (!float_is_finite_raw(cfg->filter_coeffs_raw[i])) {
            LOG_ERR("Filter coefficient [%d] is NaN or Inf", i);
            return -EINVAL;
        }
        if (absf(f[i]) > 32767.0f) {
            LOG_ERR("Filter coefficient [%d] out of Q16.16 range", i);
            return -EINVAL;
        }
    }

    /* Jury stability criterion */
    if (absf(f[5]) >= 1.0f ||
        (1.0f + f[4] + f[5]) <= 0.0f ||
        (1.0f - f[4] + f[5]) <= 0.0f) {
        LOG_ERR("Filter coefficients are unstable");
        return -EINVAL;
    }

    data->coeffs.b0 = q16_from_float(f[0]);
    data->coeffs.b1 = q16_from_float(f[1]);
    data->coeffs.b2 = q16_from_float(f[2]);
    data->coeffs.a1 = q16_from_float(f[4]);
    data->coeffs.a2 = q16_from_float(f[5]);

    return 0;
}

static void compute_axis_reciprocals(const struct axis_config *ax, int32_t deadzone,
                                     q16_t *inv_neg, q16_t *inv_pos) {
    q16_t dz = q16_from_int(deadzone);
    q16_t range_neg = q16_sat_add(q16_from_int(ax->center - ax->min), q16_neg(dz));
    q16_t range_pos = q16_sat_add(q16_from_int(ax->max - ax->center), q16_neg(dz));
    *inv_neg = (range_neg > 0) ? q16_div(Q16_ONE, range_neg) : 0;
    *inv_pos = (range_pos > 0) ? q16_div(Q16_ONE, range_pos) : 0;
}

/* -------------------------------------------------------------------------- */
/* Init                                                                       */
/* -------------------------------------------------------------------------- */

#define MAX_TURN_ON_TIME_US 50000

static int analog_stick_init(const struct device *dev) {
    struct analog_stick_data *data = dev->data;
    const struct analog_stick_config *cfg = dev->config;

    data->dev = dev;

    /* Validate calibration */
    if (cfg->x.min >= cfg->x.center || cfg->x.center >= cfg->x.max) {
        LOG_ERR("Invalid X calibration: min=%d center=%d max=%d",
                cfg->x.min, cfg->x.center, cfg->x.max);
        return -EINVAL;
    }
    if (cfg->has_y &&
        (cfg->y.min >= cfg->y.center || cfg->y.center >= cfg->y.max)) {
        LOG_ERR("Invalid Y calibration: min=%d center=%d max=%d",
                cfg->y.min, cfg->y.center, cfg->y.max);
        return -EINVAL;
    }
    /* Determine effective deadzone (precedence: deadzone-percent > deadzone) */
    if (cfg->has_deadzone_percent) {
        if (cfg->deadzone_percent < 1 || cfg->deadzone_percent > 99) {
            LOG_ERR("deadzone-percent %d out of range [1, 99]",
                    cfg->deadzone_percent);
            return -EINVAL;
        }
        /* Warn if both properties were explicitly set in DT */
        if (cfg->deadzone != 50) {
            LOG_WRN("Both deadzone and deadzone-percent specified; "
                    "deadzone-percent takes precedence");
        }
        int32_t half_range = MIN(cfg->x.center - cfg->x.min,
                                 cfg->x.max - cfg->x.center);
        data->effective_deadzone = (half_range * cfg->deadzone_percent) / 100;
    } else {
        data->effective_deadzone = cfg->deadzone;
    }

    /* Validate effective deadzone */
    if (data->effective_deadzone < 0) {
        LOG_ERR("effective deadzone must be non-negative");
        return -EINVAL;
    }
    if (data->effective_deadzone >= (cfg->x.center - cfg->x.min) ||
        data->effective_deadzone >= (cfg->x.max - cfg->x.center)) {
        LOG_ERR("effective deadzone exceeds X axis range");
        return -EINVAL;
    }
    if (cfg->has_y &&
        (data->effective_deadzone >= (cfg->y.center - cfg->y.min) ||
         data->effective_deadzone >= (cfg->y.max - cfg->y.center))) {
        LOG_ERR("effective deadzone exceeds Y axis range");
        return -EINVAL;
    }
    if (cfg->read_turn_on_time < 0 || cfg->read_turn_on_time > MAX_TURN_ON_TIME_US) {
        LOG_ERR("read-turn-on-time %d out of range", cfg->read_turn_on_time);
        return -EINVAL;
    }
    if (cfg->wait_period_idle <= 0 || cfg->wait_period_active <= 0) {
        LOG_ERR("wait periods must be > 0");
        return -EINVAL;
    }

    analog_stick_ensure_wq();

    /* GPIO setup */
    if (cfg->has_enable_gpio) {
        if (!gpio_is_ready_dt(&cfg->enable_gpio)) {
            LOG_ERR("Enable GPIO not ready");
            return -ENODEV;
        }
        int initial = cfg->pulse_read ? 0 : 1;
        int err = gpio_pin_configure_dt(&cfg->enable_gpio,
                                        GPIO_OUTPUT | (initial ? GPIO_OUTPUT_INIT_HIGH
                                                               : GPIO_OUTPUT_INIT_LOW));
        if (err) {
            LOG_ERR("GPIO configure failed: %d", err);
            return err;
        }
        if (!cfg->pulse_read) {
            if (cfg->read_turn_on_time <= 50) {
                k_busy_wait((uint32_t)cfg->read_turn_on_time);
            } else {
                k_sleep(K_USEC(cfg->read_turn_on_time));
            }
        }
    }

    /* ADC setup */
    if (!adc_is_ready_dt(&cfg->x.adc)) {
        LOG_ERR("X-axis ADC not ready");
        return -ENODEV;
    }
    if (cfg->has_y && !adc_is_ready_dt(&cfg->y.adc)) {
        LOG_ERR("Y-axis ADC not ready");
        return -ENODEV;
    }
    int err = setup_adc(data, cfg);
    if (err) {
        return err;
    }

    /* Filter setup */
    err = parse_filter_coeffs(data, cfg);
    if (err) {
        return err;
    }

    /* Pre-compute normalization scale for filter */
    if (cfg->has_filter) {
        uint8_t res = cfg->x.adc.resolution;
        if (res < 1 || res > 16) {
            LOG_ERR("ADC resolution %d out of range [1, 16]", res);
            return -EINVAL;
        }
        int32_t full_scale = (int32_t)((1U << res) - 1U);
        data->full_scale_q16 = q16_from_int(full_scale);
        data->inv_full_scale = q16_div(Q16_ONE, data->full_scale_q16);
    }

    /* Pre-compute reciprocals */
    compute_axis_reciprocals(&cfg->x, data->effective_deadzone,
                             &data->inv_range_neg_x, &data->inv_range_pos_x);
    if (cfg->has_y) {
        compute_axis_reciprocals(&cfg->y, data->effective_deadzone,
                                 &data->inv_range_neg_y, &data->inv_range_pos_y);
    }

    /* Polling is handled by the scan coordinator — no per-stick work item */

    LOG_INF("Analog stick initialized: has_y=%d, filter=%d, pulse=%d",
            cfg->has_y, cfg->has_filter, cfg->pulse_read);

    return 0;
}

/* -------------------------------------------------------------------------- */
/* PM support                                                                 */
/* -------------------------------------------------------------------------- */

#ifdef CONFIG_PM_DEVICE
static int analog_stick_pm_action(const struct device *dev,
                                  enum pm_device_action action) {
    struct analog_stick_data *data = dev->data;
    const struct analog_stick_config *cfg = dev->config;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        /* Scan coordinator handles work cancellation */
        if (cfg->has_enable_gpio) {
            gpio_pin_set_dt(&cfg->enable_gpio, 0);
        }
        return 0;
    case PM_DEVICE_ACTION_RESUME: {
        int err = setup_adc(data, cfg);
        if (err) {
            LOG_ERR("ADC re-setup after resume failed: %d", err);
            return err;
        }
        if (cfg->has_enable_gpio && !cfg->pulse_read) {
            gpio_pin_set_dt(&cfg->enable_gpio, 1);
            if (cfg->read_turn_on_time <= 50) {
                k_busy_wait((uint32_t)cfg->read_turn_on_time);
            } else {
                k_sleep(K_USEC(cfg->read_turn_on_time));
            }
        }
        data->primed = false;
        data->prev_x = INT16_MIN;
        data->prev_y = INT16_MIN;
        return 0;
    }
    default:
        return -ENOTSUP;
    }
}
#endif

/* -------------------------------------------------------------------------- */
/* Device instantiation                                                       */
/* -------------------------------------------------------------------------- */

#define HAS_PROP(n, prop) DT_INST_NODE_HAS_PROP(n, prop)

#ifndef IMPLIES
#define IMPLIES(a, b) (!(a) || (b))
#endif

/* Y-axis init: ADC_DT_SPEC_GET_BY_IDX(node, 1) eagerly expands on Zephyr 3.5
 * even inside COND_CODE_1 false branches, causing a token-pasting error when
 * io-channels has only 1 entry. Work around this by always using index 0 as a
 * dummy when has_y is false — the Y ADC spec is never actually used in that
 * case, so the value doesn't matter. */
#define ANALOG_STICK_Y_ADC_IDX(n)                                              \
    COND_CODE_1(DT_INST_PROP_HAS_IDX(n, io_channels, 1), (1), (0))

#define ANALOG_STICK_Y_AXIS_INIT(n)                                            \
    .y = {                                                                     \
        .adc = ADC_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n),                        \
                                       ANALOG_STICK_Y_ADC_IDX(n)),            \
        .min = DT_INST_PROP(n, y_min),                                         \
        .center = DT_INST_PROP(n, y_center),                                   \
        .max = DT_INST_PROP(n, y_max),                                         \
        .invert = DT_INST_PROP(n, invert_y),                                   \
    },

#define ANALOG_STICK_FILTER_COEFFS(n)                                          \
    COND_CODE_1(HAS_PROP(n, filter_coefficients),                              \
        ({                                                                     \
            DT_INST_FOREACH_PROP_ELEM_SEP(n, filter_coefficients,              \
                                          DT_PROP_BY_IDX, (, ))               \
        }),                                                                    \
        ({0, 0, 0, 0, 0, 0}))

#define ANALOG_STICK_INIT(n)                                                   \
    BUILD_ASSERT(IMPLIES(DT_INST_PROP(n, pulse_read),                          \
                         HAS_PROP(n, enable_gpios)),                           \
                 "pulse-read requires enable-gpios");                          \
    COND_CODE_1(HAS_PROP(n, filter_coefficients),                              \
        (BUILD_ASSERT(DT_INST_PROP_LEN(n, filter_coefficients) == 6,           \
                      "filter-coefficients must have exactly 6 elements")),    \
        ());                                                                   \
    BUILD_ASSERT(DT_INST_PROP(n, wait_period_active) > 0,                      \
                 "wait-period-active must be positive");                       \
    BUILD_ASSERT(DT_INST_PROP(n, wait_period_idle) > 0,                        \
                 "wait-period-idle must be positive");                         \
                                                                               \
    static struct analog_stick_data analog_stick_data_##n = {                  \
        .prev_x = INT16_MIN,                                                   \
        .prev_y = INT16_MIN,                                                   \
    };                                                                         \
                                                                               \
    static const struct analog_stick_config analog_stick_config_##n = {        \
        .has_enable_gpio = HAS_PROP(n, enable_gpios),                          \
        .enable_gpio = COND_CODE_1(HAS_PROP(n, enable_gpios),                 \
            (GPIO_DT_SPEC_INST_GET(n, enable_gpios)),                          \
            ({0})),                                                            \
        .pulse_read = DT_INST_PROP(n, pulse_read),                             \
        .read_turn_on_time = DT_INST_PROP(n, read_turn_on_time),              \
        .wait_period_idle = DT_INST_PROP(n, wait_period_idle),                 \
        .wait_period_active = DT_INST_PROP(n, wait_period_active),             \
                                                                               \
        .x = {                                                                 \
            .adc = ADC_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), 0),                \
            .min = DT_INST_PROP(n, x_min),                                     \
            .center = DT_INST_PROP(n, x_center),                               \
            .max = DT_INST_PROP(n, x_max),                                     \
            .invert = DT_INST_PROP(n, invert_x),                               \
        },                                                                     \
                                                                               \
        .has_y = DT_INST_PROP_HAS_IDX(n, io_channels, 1),                     \
        ANALOG_STICK_Y_AXIS_INIT(n)                                            \
                                                                               \
        .deadzone = DT_INST_PROP(n, deadzone),                                 \
        .deadzone_percent = DT_INST_PROP(n, deadzone_percent),                 \
        .has_deadzone_percent = HAS_PROP(n, deadzone_percent),                 \
        .inter_channel_settling_us = DT_INST_PROP(n, inter_channel_settling_time), \
                                                                               \
        .has_filter = HAS_PROP(n, filter_coefficients),                        \
        .filter_coeffs_raw = ANALOG_STICK_FILTER_COEFFS(n),                    \
    };                                                                         \
                                                                               \
    PM_DEVICE_DT_INST_DEFINE(n, analog_stick_pm_action);                       \
                                                                               \
    DEVICE_DT_INST_DEFINE(n,                                                   \
                          analog_stick_init,                                    \
                          PM_DEVICE_DT_INST_GET(n),                            \
                          &analog_stick_data_##n,                              \
                          &analog_stick_config_##n,                            \
                          POST_KERNEL,                                         \
                          CONFIG_ZMK_ANALOG_STICK_INIT_PRIORITY,               \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(ANALOG_STICK_INIT)
