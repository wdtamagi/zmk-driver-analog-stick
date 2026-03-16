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

BUILD_ASSERT(((int64_t)(-1) >> 1) == (int64_t)(-1),
             "q16_mul requires arithmetic right-shift for int64_t");

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

    /* Runtime axis configs (copied from cfg at init; center may be overridden by auto-center) */
    struct axis_config effective_x;
    struct axis_config effective_y;

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
/* ADC result sanitization                                                    */
/* -------------------------------------------------------------------------- */

/**
 * Extract the low N bits of a raw ADC buffer value and clamp negative readings
 * to 0.  The nRF SAADC is inherently signed; in single-ended mode a near-GND
 * input can produce small negative results that land in the int32_t buffer
 * without sign extension (e.g. -1 → 0x0000FFFF).  We detect the ADC sign bit
 * and clamp to 0 — no shift-based sign extension needed, avoiding UB in C99.
 */
static inline int32_t adc_sanitize(int32_t raw, uint8_t resolution) {
    if (resolution == 0 || resolution > 16) {
        return 0;
    }
    int32_t value = raw & ((1 << resolution) - 1); /* extract low N bits */
    if (value & (1 << (resolution - 1))) {          /* ADC sign bit set → negative */
        return 0;
    }
    return value;
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
        LOG_ERR("[%s] X-axis ADC read failed: %d", dev->name, err);
        data->read_err = err;
        return err;
    }

    int32_t adc_x = adc_sanitize(data->adc_buf[0], cfg->x.adc.resolution);
    LOG_DBG("[%s] raw X=%d", dev->name, adc_x);
    data->raw_x = q16_from_int(adc_x);

    if (cfg->has_y) {
        if (cfg->inter_channel_settling_us <= 100) {
            k_busy_wait((uint32_t)cfg->inter_channel_settling_us);
        } else {
            k_sleep(K_USEC((uint32_t)cfg->inter_channel_settling_us));
        }
        err = adc_read(cfg->y.adc.dev, &data->seq_y);
        if (err) {
            LOG_ERR("[%s] Y-axis ADC read failed: %d", dev->name, err);
            data->read_err = err;
            return err;
        }
        int32_t adc_y = adc_sanitize(data->adc_buf[1], cfg->y.adc.resolution);
        LOG_DBG("[%s] raw Y=%d", dev->name, adc_y);
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
    q16_t scaled_x = rescale_axis(raw_x, &data->effective_x, data->effective_deadzone,
                                  data->inv_range_neg_x, data->inv_range_pos_x);
    q16_t scaled_y = cfg->has_y
        ? rescale_axis(raw_y, &data->effective_y, data->effective_deadzone,
                       data->inv_range_neg_y, data->inv_range_pos_y)
        : 0;

    /* --- Map to output range [-127, 127] --- */
    int16_t out_x = (int16_t)(((int32_t)scaled_x * 127) >> Q16_SHIFT);
    int16_t out_y = (int16_t)(((int32_t)scaled_y * 127) >> Q16_SHIFT);

    /* --- Emit input events on change OR while deflected --- */
    bool x_changed = (out_x != data->prev_x);
    bool y_changed = (out_y != data->prev_y);
    bool deflected = (out_x != 0 || out_y != 0);

    if (x_changed || y_changed || deflected) {
        if (cfg->has_y) {
            input_report_abs(dev, INPUT_ABS_X, out_x, false, K_NO_WAIT);
            input_report_abs(dev, INPUT_ABS_Y, out_y, true, K_NO_WAIT);
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

static int setup_adc(const struct device *dev,
                     struct analog_stick_data *data,
                     const struct analog_stick_config *cfg) {
    int err = adc_channel_setup_dt(&cfg->x.adc);
    if (err) {
        LOG_ERR("[%s] X-axis ADC channel setup failed: %d", dev->name, err);
        return err;
    }
    adc_sequence_init_dt(&cfg->x.adc, &data->seq_x);
    data->seq_x.buffer = &data->adc_buf[0];
    data->seq_x.buffer_size = sizeof(data->adc_buf[0]);

    if (cfg->has_y) {
        err = adc_channel_setup_dt(&cfg->y.adc);
        if (err) {
            LOG_ERR("[%s] Y-axis ADC channel setup failed: %d", dev->name, err);
            return err;
        }
        adc_sequence_init_dt(&cfg->y.adc, &data->seq_y);
        data->seq_y.buffer = &data->adc_buf[1];
        data->seq_y.buffer_size = sizeof(data->adc_buf[1]);
    }

    return 0;
}

static int parse_filter_coeffs(const struct device *dev,
                               struct analog_stick_data *data,
                               const struct analog_stick_config *cfg) {
    if (!cfg->has_filter) {
        return 0;
    }

    float f[6];
    for (int i = 0; i < 6; i++) {
        memcpy(&f[i], &cfg->filter_coeffs_raw[i], sizeof(float));
    }

    if (absf(f[3] - 1.0f) > 1e-6f) {
        LOG_ERR("[%s] Filter a0 must be 1.0 (normalized)", dev->name);
        return -EINVAL;
    }

    for (int i = 0; i < 6; i++) {
        if (!float_is_finite_raw(cfg->filter_coeffs_raw[i])) {
            LOG_ERR("[%s] Filter coefficient [%d] is NaN or Inf", dev->name, i);
            return -EINVAL;
        }
        if (absf(f[i]) > 32767.0f) {
            LOG_ERR("[%s] Filter coefficient [%d] out of Q16.16 range", dev->name, i);
            return -EINVAL;
        }
    }

    /* Jury stability criterion */
    if (absf(f[5]) >= 1.0f ||
        (1.0f + f[4] + f[5]) <= 0.0f ||
        (1.0f - f[4] + f[5]) <= 0.0f) {
        LOG_ERR("[%s] Filter coefficients are unstable", dev->name);
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

    /* Validate DT calibration (sanity check regardless of auto-center) */
    if (cfg->x.min >= cfg->x.center || cfg->x.center >= cfg->x.max) {
        LOG_ERR("[%s] Invalid X calibration: min=%d center=%d max=%d",
                dev->name, cfg->x.min, cfg->x.center, cfg->x.max);
        return -EINVAL;
    }
    if (cfg->has_y &&
        (cfg->y.min >= cfg->y.center || cfg->y.center >= cfg->y.max)) {
        LOG_ERR("[%s] Invalid Y calibration: min=%d center=%d max=%d",
                dev->name, cfg->y.min, cfg->y.center, cfg->y.max);
        return -EINVAL;
    }
    if (cfg->has_deadzone_percent) {
        if (cfg->deadzone_percent < 1 || cfg->deadzone_percent > 99) {
            LOG_ERR("[%s] deadzone-percent %d out of range [1, 99]",
                    dev->name, cfg->deadzone_percent);
            return -EINVAL;
        }
        if (cfg->deadzone != 50) {
            LOG_WRN("[%s] Both deadzone and deadzone-percent specified; "
                    "deadzone-percent takes precedence", dev->name);
        }
    }
    if (cfg->read_turn_on_time < 0 || cfg->read_turn_on_time > MAX_TURN_ON_TIME_US) {
        LOG_ERR("[%s] read-turn-on-time %d out of range", dev->name, cfg->read_turn_on_time);
        return -EINVAL;
    }
    if (cfg->wait_period_idle <= 0 || cfg->wait_period_active <= 0) {
        LOG_ERR("[%s] wait periods must be > 0", dev->name);
        return -EINVAL;
    }

    analog_stick_ensure_wq();

    /* GPIO setup */
    if (cfg->has_enable_gpio) {
        if (!gpio_is_ready_dt(&cfg->enable_gpio)) {
            LOG_ERR("[%s] Enable GPIO not ready", dev->name);
            return -ENODEV;
        }
        int initial = cfg->pulse_read ? 0 : 1;
        int err = gpio_pin_configure_dt(&cfg->enable_gpio,
                                        GPIO_OUTPUT | (initial ? GPIO_OUTPUT_INIT_HIGH
                                                               : GPIO_OUTPUT_INIT_LOW));
        if (err) {
            LOG_ERR("[%s] GPIO configure failed: %d", dev->name, err);
            return err;
        }
        if (!cfg->pulse_read) {
            if (cfg->read_turn_on_time <= 100) {
                k_busy_wait((uint32_t)cfg->read_turn_on_time);
            } else {
                k_sleep(K_USEC(cfg->read_turn_on_time));
            }
        }
    }

    /* ADC setup */
    if (!adc_is_ready_dt(&cfg->x.adc)) {
        LOG_ERR("[%s] X-axis ADC not ready", dev->name);
        return -ENODEV;
    }
    if (cfg->has_y && !adc_is_ready_dt(&cfg->y.adc)) {
        LOG_ERR("[%s] Y-axis ADC not ready", dev->name);
        return -ENODEV;
    }
    int err = setup_adc(dev, data, cfg);
    if (err) {
        return err;
    }

    /* Copy axis configs into mutable data (center may be overridden below) */
    data->effective_x = cfg->x;
    if (cfg->has_y) {
        data->effective_y = cfg->y;
    }

#ifdef CONFIG_ZMK_ANALOG_STICK_AUTO_CENTER
    {
        int32_t sum_x = 0;
        int32_t sum_y = 0;
        for (int i = 0; i < CONFIG_ZMK_ANALOG_STICK_AUTO_CENTER_SAMPLES; i++) {
            int ac_err = adc_read(cfg->x.adc.dev, &data->seq_x);
            if (ac_err) {
                LOG_ERR("[%s] Auto-center X read %d failed: %d", dev->name, i, ac_err);
                return ac_err;
            }
            sum_x += adc_sanitize(data->adc_buf[0], cfg->x.adc.resolution);
            if (cfg->has_y) {
                if (cfg->inter_channel_settling_us <= 100) {
                    k_busy_wait((uint32_t)cfg->inter_channel_settling_us);
                } else {
                    k_sleep(K_USEC((uint32_t)cfg->inter_channel_settling_us));
                }
                ac_err = adc_read(cfg->y.adc.dev, &data->seq_y);
                if (ac_err) {
                    LOG_ERR("[%s] Auto-center Y read %d failed: %d", dev->name, i, ac_err);
                    return ac_err;
                }
                sum_y += adc_sanitize(data->adc_buf[1], cfg->y.adc.resolution);
            }
        }
        data->effective_x.center = sum_x / CONFIG_ZMK_ANALOG_STICK_AUTO_CENTER_SAMPLES;
        LOG_INF("[%s] Auto-center X: DT=%d measured=%d", dev->name, cfg->x.center,
                data->effective_x.center);
        if (cfg->has_y) {
            data->effective_y.center = sum_y / CONFIG_ZMK_ANALOG_STICK_AUTO_CENTER_SAMPLES;
            LOG_INF("[%s] Auto-center Y: DT=%d measured=%d", dev->name, cfg->y.center,
                    data->effective_y.center);
        }
    }
#endif

    /* Determine effective deadzone (uses effective center from auto-center or DT) */
    if (cfg->has_deadzone_percent) {
        int32_t half_range = MIN(data->effective_x.center - data->effective_x.min,
                                 data->effective_x.max - data->effective_x.center);
        data->effective_deadzone = (half_range * cfg->deadzone_percent) / 100;
    } else {
        data->effective_deadzone = cfg->deadzone;
    }

    /* Validate effective deadzone */
    if (data->effective_deadzone < 0) {
        LOG_ERR("[%s] effective deadzone must be non-negative", dev->name);
        return -EINVAL;
    }
    if (data->effective_deadzone >= (data->effective_x.center - data->effective_x.min) ||
        data->effective_deadzone >= (data->effective_x.max - data->effective_x.center)) {
        LOG_ERR("[%s] effective deadzone exceeds X axis range", dev->name);
        return -EINVAL;
    }
    if (cfg->has_y &&
        (data->effective_deadzone >= (data->effective_y.center - data->effective_y.min) ||
         data->effective_deadzone >= (data->effective_y.max - data->effective_y.center))) {
        LOG_ERR("[%s] effective deadzone exceeds Y axis range", dev->name);
        return -EINVAL;
    }

    /* Filter setup */
    err = parse_filter_coeffs(dev, data, cfg);
    if (err) {
        return err;
    }

    /* Pre-compute normalization scale for filter */
    if (cfg->has_filter) {
        uint8_t res = cfg->x.adc.resolution;
        if (res < 1 || res > 16) {
            LOG_ERR("[%s] ADC resolution %d out of range [1, 16]", dev->name, res);
            return -EINVAL;
        }
        int32_t full_scale = (int32_t)((1U << res) - 1U);
        data->full_scale_q16 = q16_from_int(full_scale);
        data->inv_full_scale = q16_div(Q16_ONE, data->full_scale_q16);
    }

    /* Pre-compute reciprocals (uses effective axis configs) */
    compute_axis_reciprocals(&data->effective_x, data->effective_deadzone,
                             &data->inv_range_neg_x, &data->inv_range_pos_x);
    if (cfg->has_y) {
        compute_axis_reciprocals(&data->effective_y, data->effective_deadzone,
                                 &data->inv_range_neg_y, &data->inv_range_pos_y);
    }

    /* Polling is handled by the scan coordinator — no per-stick work item */

    LOG_INF("[%s] initialized: has_y=%d, filter=%d, pulse=%d",
            dev->name, cfg->has_y, cfg->has_filter, cfg->pulse_read);

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
        int err = setup_adc(dev, data, cfg);
        if (err) {
            LOG_ERR("[%s] ADC re-setup after resume failed: %d", dev->name, err);
            return err;
        }
        if (cfg->has_enable_gpio && !cfg->pulse_read) {
            gpio_pin_set_dt(&cfg->enable_gpio, 1);
            if (cfg->read_turn_on_time <= 100) {
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
