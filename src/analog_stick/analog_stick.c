/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Multi-mode analog stick driver for ZMK.
 * Reads hall-effect sensors via Zephyr ADC io-channels and emits
 * Zephyr input events (INPUT_EV_ABS). Mode-aware processing is
 * handled by a separate input listener.
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

LOG_MODULE_REGISTER(zmk_analog_stick, CONFIG_ZMK_ANALOG_STICK_LOG_LEVEL);

BUILD_ASSERT(sizeof(float) == sizeof(uint32_t),
             "Filter coefficient parsing requires 32-bit floats");

BUILD_ASSERT(((-1) >> 1) == -1,
             "This driver requires arithmetic right-shift for signed integers");

/* -------------------------------------------------------------------------- */
/* Q16.16 fixed-point arithmetic                                              */
/* -------------------------------------------------------------------------- */

typedef int32_t q16_t;
#define Q16_SHIFT  16
#define Q16_ONE    ((q16_t)(1U << Q16_SHIFT))

static inline q16_t q16_from_int(int32_t v) {
    if (v > 32767) return (q16_t)(32767U << Q16_SHIFT);
    if (v < -32767) return (q16_t)((uint32_t)(int32_t)-32767 << Q16_SHIFT);
    return (q16_t)((uint32_t)(int32_t)v << Q16_SHIFT);
}

static inline q16_t q16_mul(q16_t a, q16_t b) {
    return (q16_t)(((int64_t)a * b) >> Q16_SHIFT);
}

static inline q16_t q16_div(q16_t a, q16_t b) {
    if (b == 0) {
        return (a >= 0) ? INT32_MAX : INT32_MIN;
    }
    return (q16_t)(((int64_t)a << Q16_SHIFT) / b);
}

static inline q16_t q16_abs(q16_t x) {
    if (x == INT32_MIN) return INT32_MAX;
    return x < 0 ? -x : x;
}

static inline q16_t q16_neg(q16_t x) {
    return (x == INT32_MIN) ? INT32_MAX : -x;
}

static inline q16_t q16_clamp(q16_t x, q16_t lo, q16_t hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline q16_t q16_sat_add(q16_t a, q16_t b) {
    int64_t r = (int64_t)a + b;
    if (r > INT32_MAX) return INT32_MAX;
    if (r < INT32_MIN) return INT32_MIN;
    return (q16_t)r;
}

static inline q16_t q16_from_float(float f) {
    if (f != f) return 0; /* NaN */
    float product = f * (float)Q16_ONE;
    if (product > (float)INT32_MAX) return INT32_MAX;
    if (product < (float)INT32_MIN) return INT32_MIN;
    return (q16_t)product;
}

static inline bool q16_is_valid(q16_t x) {
    return x != INT32_MIN && x != INT32_MAX;
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
/* Dedicated work queue                                                       */
/* -------------------------------------------------------------------------- */

static K_THREAD_STACK_DEFINE(analog_stick_wq_stack,
                             CONFIG_ZMK_ANALOG_STICK_WORK_QUEUE_STACK_SIZE);
static struct k_work_q analog_stick_wq;
static atomic_t wq_started = ATOMIC_INIT(0);

static void ensure_wq(void) {
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
    int32_t switch_activation_point;
    int32_t switch_hysteresis;

    bool has_filter;
    uint32_t filter_coeffs_raw[6];
};

struct analog_stick_data {
    const struct device *dev;
    struct k_work_delayable poll_work;

    struct adc_sequence seq_x;
    struct adc_sequence seq_y;

    int32_t adc_buf[2]; /* [0]=X, [1]=Y — inlined, no DMA section needed */

    struct biquad_state filter_x;
    struct biquad_state filter_y;
    struct biquad_coeffs coeffs;

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

#ifdef CONFIG_PM_DEVICE
    struct k_work_sync work_sync;
#endif
};

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
        scaled = q16_mul(diff + dz, inv_neg);
    } else {
        if (inv_pos == 0) return 0;
        scaled = q16_mul(diff - dz, inv_pos);
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

#define ADC_RAIL_MARGIN 5

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
/* Poll work handler                                                          */
/* -------------------------------------------------------------------------- */

static void analog_stick_poll_work(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct analog_stick_data *data = CONTAINER_OF(dwork, struct analog_stick_data, poll_work);
    const struct device *dev = data->dev;
    const struct analog_stick_config *cfg = dev->config;

    /* --- Power on --- */
    if (cfg->pulse_read && cfg->has_enable_gpio) {
        gpio_pin_set_dt(&cfg->enable_gpio, 1);
        if (cfg->read_turn_on_time <= 1000) {
            k_busy_wait((uint32_t)cfg->read_turn_on_time);
        } else {
            k_sleep(K_USEC(cfg->read_turn_on_time));
        }
    }

    /* --- Read ADC --- */
    int err = adc_read(cfg->x.adc.dev, &data->seq_x);
    if (err) {
        LOG_ERR("X-axis ADC read failed: %d", err);
        goto power_off;
    }

    q16_t raw_x = 0;
    q16_t raw_y = 0;

    int32_t adc_x = data->adc_buf[0];
    if (adc_at_rail(adc_x, cfg->x.adc.resolution, cfg->x.adc.oversampling)) {
        LOG_DBG("X-axis at rail (%d)", adc_x);
        adc_x = cfg->x.center;
    }
    raw_x = q16_from_int(adc_x);

    if (cfg->has_y) {
        k_busy_wait(5); /* inter-channel settling */
        err = adc_read(cfg->y.adc.dev, &data->seq_y);
        if (err) {
            LOG_ERR("Y-axis ADC read failed: %d", err);
            goto power_off;
        }
        int32_t adc_y = data->adc_buf[1];
        if (adc_at_rail(adc_y, cfg->y.adc.resolution, cfg->y.adc.oversampling)) {
            LOG_DBG("Y-axis at rail (%d)", adc_y);
            adc_y = cfg->y.center;
        }
        raw_y = q16_from_int(adc_y);
    }

power_off:
    if (cfg->pulse_read && cfg->has_enable_gpio) {
        gpio_pin_set_dt(&cfg->enable_gpio, 0);
    }

    if (err) {
        goto reschedule;
    }

    /* --- IIR filter --- */
    if (cfg->has_filter && data->active) {
        if (!data->primed) {
            biquad_prime(&data->filter_x, &data->coeffs, raw_x);
            if (cfg->has_y) {
                biquad_prime(&data->filter_y, &data->coeffs, raw_y);
            }
            data->primed = true;
            goto skip_filter;
        }
        raw_x = biquad_process(&data->filter_x, &data->coeffs, raw_x);
        if (cfg->has_y) {
            raw_y = biquad_process(&data->filter_y, &data->coeffs, raw_y);
        }
    } else if (cfg->has_filter && !data->active) {
        data->primed = false;
    }

skip_filter:;
    /* --- Rescale --- */
    q16_t scaled_x = rescale_axis(raw_x, &cfg->x, cfg->deadzone,
                                  data->inv_range_neg_x, data->inv_range_pos_x);
    q16_t scaled_y = cfg->has_y
        ? rescale_axis(raw_y, &cfg->y, cfg->deadzone,
                       data->inv_range_neg_y, data->inv_range_pos_y)
        : 0;

    /* --- Circular deadzone for dual-axis --- */
    if (cfg->has_y) {
        int64_t dx = (int64_t)scaled_x;
        int64_t dy = (int64_t)scaled_y;
        q16_t dz = q16_from_int(cfg->deadzone);
        /* Convert deadzone to Q16.16 fraction of range for comparison
         * with scaled values. Since scaled values are in [-Q16_ONE, Q16_ONE],
         * we need deadzone as a fraction. Use a simple threshold on the
         * squared magnitude vs squared deadzone-fraction. */
        /* For circular deadzone: if both individual per-axis deadzones
         * already zeroed the values, the result is already (0,0).
         * The per-axis linear deadzone in rescale_axis handles most cases.
         * This additional check catches the diagonal case where each axis
         * is individually above its linear deadzone but the combined
         * magnitude is still inside the circular zone. */
        if (scaled_x != 0 || scaled_y != 0) {
            /* Use the smaller of the two per-axis deadzone fractions as
             * the circular radius, expressed in the scaled output space */
            q16_t range_x_min = q16_from_int(cfg->x.center - cfg->x.min);
            q16_t range_y_min = q16_from_int(cfg->y.center - cfg->y.min);
            q16_t frac_x = (range_x_min > 0) ? q16_div(dz, range_x_min) : 0;
            q16_t frac_y = (range_y_min > 0) ? q16_div(dz, range_y_min) : 0;
            /* Use average fraction as circular radius in output space */
            q16_t circ_dz = (frac_x + frac_y) / 2;
            int64_t circ_dz2 = (int64_t)circ_dz * circ_dz;
            int64_t mag2 = (dx * dx + dy * dy) >> Q16_SHIFT;
            if (mag2 < (circ_dz2 >> Q16_SHIFT)) {
                scaled_x = 0;
                scaled_y = 0;
            }
        }
    }

    /* --- Map to output range [-127, 127] --- */
    int16_t out_x = (int16_t)(((int32_t)scaled_x * 127) >> Q16_SHIFT);
    int16_t out_y = (int16_t)(((int32_t)scaled_y * 127) >> Q16_SHIFT);

    /* --- Emit input events on change --- */
    if (out_x != data->prev_x || out_y != data->prev_y) {
        if (cfg->has_y) {
            if (out_x != data->prev_x) {
                input_report_abs(dev, INPUT_ABS_X, out_x, false, K_NO_WAIT);
            }
            if (out_y != data->prev_y) {
                input_report_abs(dev, INPUT_ABS_Y, out_y,
                                 out_x == data->prev_x, K_NO_WAIT);
            }
            /* If only X changed, send sync */
            if (out_x != data->prev_x && out_y == data->prev_y) {
                input_report_abs(dev, INPUT_ABS_X, out_x, true, K_NO_WAIT);
            }
        } else {
            input_report_abs(dev, INPUT_ABS_X, out_x, true, K_NO_WAIT);
        }
        data->prev_x = out_x;
        data->prev_y = out_y;
    }

    /* --- Activity detection --- */
    data->active = (out_x != 0 || out_y != 0);

reschedule: {
    int32_t period = data->active ? cfg->wait_period_active : cfg->wait_period_idle;
#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (data->active && period < CONFIG_ZMK_ANALOG_STICK_BLE_POLL_FLOOR) {
        period = CONFIG_ZMK_ANALOG_STICK_BLE_POLL_FLOOR;
    }
#endif
    k_work_schedule_for_queue(&analog_stick_wq, &data->poll_work, K_MSEC(period));
    }
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
    q16_t range_neg = q16_from_int(ax->center - ax->min) - dz;
    q16_t range_pos = q16_from_int(ax->max - ax->center) - dz;
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
    if (cfg->deadzone < 0) {
        LOG_ERR("deadzone must be non-negative");
        return -EINVAL;
    }
    if (cfg->deadzone >= (cfg->x.center - cfg->x.min) ||
        cfg->deadzone >= (cfg->x.max - cfg->x.center)) {
        LOG_ERR("deadzone exceeds X axis range");
        return -EINVAL;
    }
    if (cfg->has_y &&
        (cfg->deadzone >= (cfg->y.center - cfg->y.min) ||
         cfg->deadzone >= (cfg->y.max - cfg->y.center))) {
        LOG_ERR("deadzone exceeds Y axis range");
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

    ensure_wq();

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
            if (cfg->read_turn_on_time <= 1000) {
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

    /* Pre-compute reciprocals */
    compute_axis_reciprocals(&cfg->x, cfg->deadzone,
                             &data->inv_range_neg_x, &data->inv_range_pos_x);
    if (cfg->has_y) {
        compute_axis_reciprocals(&cfg->y, cfg->deadzone,
                                 &data->inv_range_neg_y, &data->inv_range_pos_y);
    }

    /* Start polling */
    k_work_init_delayable(&data->poll_work, analog_stick_poll_work);
    k_work_schedule_for_queue(&analog_stick_wq, &data->poll_work,
                              K_MSEC(cfg->wait_period_idle));

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
        k_work_cancel_delayable_sync(&data->poll_work, &data->work_sync);
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
            if (cfg->read_turn_on_time <= 1000) {
                k_busy_wait((uint32_t)cfg->read_turn_on_time);
            } else {
                k_sleep(K_USEC(cfg->read_turn_on_time));
            }
        }
        data->primed = false;
        data->prev_x = INT16_MIN;
        data->prev_y = INT16_MIN;
        k_work_schedule_for_queue(&analog_stick_wq, &data->poll_work,
                                  K_MSEC(cfg->wait_period_idle));
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
    BUILD_ASSERT(!HAS_PROP(n, filter_coefficients) ||                          \
                 DT_INST_PROP_LEN(n, filter_coefficients) == 6,                \
                 "filter-coefficients must have exactly 6 elements");          \
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
        .y = COND_CODE_1(DT_INST_PROP_HAS_IDX(n, io_channels, 1),            \
            ({                                                                 \
                .adc = ADC_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), 1),            \
                .min = DT_INST_PROP(n, y_min),                                 \
                .center = DT_INST_PROP(n, y_center),                           \
                .max = DT_INST_PROP(n, y_max),                                 \
                .invert = DT_INST_PROP(n, invert_y),                           \
            }),                                                                \
            ({0})),                                                            \
                                                                               \
        .deadzone = DT_INST_PROP(n, deadzone),                                 \
        .switch_activation_point = DT_INST_PROP(n, switch_activation_point),   \
        .switch_hysteresis = DT_INST_PROP(n, switch_hysteresis),               \
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
