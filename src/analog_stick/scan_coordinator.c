/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Scan coordinator for multi-stick configurations.
 * Replaces per-stick work items with a single batched scan that:
 * - Groups sticks by enable GPIO for shared settling
 * - Reads all ADC channels per group with one settling wait
 * - Processes all sticks after all reads complete
 * - Calls HID flush at end of cycle
 * - Tracks negotiated BLE CI for adaptive poll floor
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>

#include <analog-stick/analog_stick_internal.h>
#include <analog-stick/hid_accumulator.h>

#if defined(CONFIG_ZMK_BLE) && !defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zephyr/bluetooth/conn.h>
#endif

LOG_MODULE_DECLARE(zmk_analog_stick, CONFIG_ZMK_ANALOG_STICK_LOG_LEVEL);

/* -------------------------------------------------------------------------- */
/* Stick instance registry                                                    */
/* -------------------------------------------------------------------------- */

#define MAX_STICKS CONFIG_ZMK_ANALOG_STICK_STICK_COUNT

struct analog_stick_entry {
    const struct device *dev;
    const struct gpio_dt_spec *enable_gpio;
    uint32_t settling_us;
    bool has_gpio;
    bool pulse_read;
};

static struct analog_stick_entry stick_registry[MAX_STICKS];
static size_t stick_count;

/* -------------------------------------------------------------------------- */
/* GPIO grouping                                                              */
/* -------------------------------------------------------------------------- */

#define MAX_GROUPS MAX_STICKS

struct gpio_group {
    const struct gpio_dt_spec *gpio; /* NULL for no-GPIO group */
    bool has_gpio;
    bool any_pulse_read;
    uint32_t settling_us; /* max settling time in this group */
    uint8_t stick_indices[MAX_STICKS];
    uint8_t count;
};

static struct gpio_group groups[MAX_GROUPS];
static size_t group_count;

static void build_gpio_groups(void) {
    group_count = 0;

    for (size_t i = 0; i < stick_count; i++) {
        struct analog_stick_entry *entry = &stick_registry[i];
        int found = -1;

        if (entry->has_gpio) {
            /* Find existing group with same GPIO */
            for (size_t g = 0; g < group_count; g++) {
                if (groups[g].has_gpio &&
                    groups[g].gpio->port == entry->enable_gpio->port &&
                    groups[g].gpio->pin == entry->enable_gpio->pin) {
                    found = (int)g;
                    break;
                }
            }
        } else {
            /* Find the single shared no-GPIO group */
            for (size_t g = 0; g < group_count; g++) {
                if (!groups[g].has_gpio) {
                    found = (int)g;
                    break;
                }
            }
        }

        if (found < 0) {
            /* Create new group */
            if (group_count >= MAX_GROUPS) {
                LOG_ERR("Too many GPIO groups");
                break;
            }
            found = (int)group_count;
            groups[found].gpio = entry->enable_gpio;
            groups[found].has_gpio = entry->has_gpio;
            groups[found].any_pulse_read = entry->pulse_read;
            groups[found].settling_us = entry->settling_us;
            groups[found].count = 0;
            group_count++;
        }

        struct gpio_group *g = &groups[found];
        if (g->count < MAX_STICKS) {
            g->stick_indices[g->count++] = (uint8_t)i;
        }
        if (entry->settling_us > g->settling_us) {
            g->settling_us = entry->settling_us;
        }
        if (entry->pulse_read) {
            g->any_pulse_read = true;
        }
    }

    LOG_INF("Scan coordinator: %zu sticks in %zu GPIO groups",
            stick_count, group_count);
}

/* -------------------------------------------------------------------------- */
/* CI-aware poll floor                                                        */
/* -------------------------------------------------------------------------- */

#if defined(CONFIG_ZMK_BLE) && !defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

static atomic_t negotiated_ci_ms = ATOMIC_INIT(0);

static void on_le_param_updated(struct bt_conn *conn, uint16_t interval,
                                uint16_t latency, uint16_t timeout) {
    /* interval is in 1.25ms units; convert to ms */
    int32_t ci = (int32_t)((interval * 5 + 3) / 4);
    atomic_set(&negotiated_ci_ms, ci);

    int32_t ci_floor;
#if CONFIG_ZMK_ANALOG_STICK_CI_FLOOR_OVERRIDE > 0
    ci_floor = CONFIG_ZMK_ANALOG_STICK_CI_FLOOR_OVERRIDE;
#else
    ci_floor = MAX(CONFIG_ZMK_ANALOG_STICK_BLE_POLL_FLOOR, ci);
#endif
    LOG_DBG("CI updated to %d ms, effective poll floor: %d ms", ci, ci_floor);
}

static struct bt_conn_cb scan_conn_callbacks = {
    .le_param_updated = on_le_param_updated,
};

#endif /* CONFIG_ZMK_BLE && !CENTRAL */

/* -------------------------------------------------------------------------- */
/* Pulse-read suppression state (per GPIO group)                              */
/* -------------------------------------------------------------------------- */

static bool group_pulse_suppressed[MAX_GROUPS];

/* -------------------------------------------------------------------------- */
/* Active cooldown: stay at fast poll rate briefly after stick returns to      */
/* center, so rapid successive flicks are not missed.                         */
/* -------------------------------------------------------------------------- */

static int64_t last_active_time;  /* k_uptime_get() of last deflected poll */

/* -------------------------------------------------------------------------- */
/* Scan work handler                                                          */
/* -------------------------------------------------------------------------- */

static struct k_work_delayable scan_work;

static void scan_coordinator_work_handler(struct k_work *work) {
    /* --- Phase 1: Read ADC for all sticks, grouped by GPIO --- */
    for (size_t g = 0; g < group_count; g++) {
        struct gpio_group *grp = &groups[g];

        /* Assert group GPIO if pulse-read and not suppressed */
        if (grp->has_gpio && grp->any_pulse_read &&
            !group_pulse_suppressed[g]) {
            gpio_pin_set_dt(grp->gpio, 1);
            if (grp->settling_us <= 100) {
                k_busy_wait(grp->settling_us);
            } else {
                k_sleep(K_USEC(grp->settling_us));
            }
        }

        /* Read all sticks in this group */
        for (uint8_t s = 0; s < grp->count; s++) {
            analog_stick_read_adc(stick_registry[grp->stick_indices[s]].dev);
        }

        /* Deassert group GPIO if pulse-read and not suppressed */
        if (grp->has_gpio && grp->any_pulse_read &&
            !group_pulse_suppressed[g]) {
            gpio_pin_set_dt(grp->gpio, 0);
        }
    }

    /* --- Phase 2: Process all sticks (filter, rescale, emit events) --- */
    for (size_t i = 0; i < stick_count; i++) {
        analog_stick_process_and_emit(stick_registry[i].dev);
    }

    /* --- Phase 3: HID flush (one combined report for all sticks) --- */
    zmk_analog_stick_hid_flush();

    /* --- Phase 4: Adaptive rate selection and pulse-read management --- */
    bool any_active = false;
    int32_t min_active_ms = INT32_MAX;
    int32_t max_idle_ms = 0;

    for (size_t i = 0; i < stick_count; i++) {
        const struct device *dev = stick_registry[i].dev;
        if (analog_stick_is_active(dev)) {
            any_active = true;
            int32_t ap = analog_stick_get_wait_period_active(dev);
            if (ap < min_active_ms) {
                min_active_ms = ap;
            }
        }
        int32_t ip = analog_stick_get_wait_period_idle(dev);
        if (ip > max_idle_ms) {
            max_idle_ms = ip;
        }
    }

    /* Active cooldown: keep fast poll rate briefly after stick returns to
     * center so rapid successive flicks (double-tap) are not missed. */
    int64_t now = k_uptime_get();
    bool in_cooldown = false;

    if (any_active) {
        last_active_time = now;
    } else if (CONFIG_ZMK_ANALOG_STICK_ACTIVE_COOLDOWN_MS > 0 &&
               last_active_time > 0 &&
               (now - last_active_time) < CONFIG_ZMK_ANALOG_STICK_ACTIVE_COOLDOWN_MS) {
        in_cooldown = true;
    }

    int32_t selected_rate_ms = (any_active || in_cooldown) ? min_active_ms : max_idle_ms;
    /* If only in cooldown (no stick currently active), min_active_ms is still
     * INT32_MAX — fall back to the fastest configured active rate. */
    if (in_cooldown && !any_active) {
        /* Use the smallest active period across all sticks */
        selected_rate_ms = INT32_MAX;
        for (size_t i = 0; i < stick_count; i++) {
            int32_t ap = analog_stick_get_wait_period_active(stick_registry[i].dev);
            if (ap < selected_rate_ms) {
                selected_rate_ms = ap;
            }
        }
    }

    /* Pulse-read suppression: auto-disable when settling > half poll period */
    for (size_t g = 0; g < group_count; g++) {
        struct gpio_group *grp = &groups[g];
        if (!grp->has_gpio || !grp->any_pulse_read) {
            continue;
        }

        if (any_active &&
            grp->settling_us > (uint32_t)(selected_rate_ms * 1000 / 2)) {
            if (!group_pulse_suppressed[g]) {
                LOG_INF("pulse-read auto-disabled for group %zu: "
                        "settling %uus > half poll period %uus",
                        g, grp->settling_us,
                        (uint32_t)(selected_rate_ms * 1000 / 2));
                gpio_pin_set_dt(grp->gpio, 1);
                group_pulse_suppressed[g] = true;
            }
        } else if (group_pulse_suppressed[g] && !any_active && !in_cooldown) {
            LOG_INF("pulse-read re-enabled for group %zu: "
                    "returning to idle poll rate", g);
            gpio_pin_set_dt(grp->gpio, 0);
            group_pulse_suppressed[g] = false;
        }
    }

    /* --- Reschedule --- */
    int32_t effective_floor = 0;

#if IS_ENABLED(CONFIG_ZMK_BLE)
#if !defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    {
        int32_t ci = (int32_t)atomic_get(&negotiated_ci_ms);
#if CONFIG_ZMK_ANALOG_STICK_CI_FLOOR_OVERRIDE > 0
        effective_floor = CONFIG_ZMK_ANALOG_STICK_CI_FLOOR_OVERRIDE;
#else
        effective_floor = MAX(CONFIG_ZMK_ANALOG_STICK_BLE_POLL_FLOOR,
                              ci > 0 ? ci : 0);
#endif
    }
#else
    effective_floor = CONFIG_ZMK_ANALOG_STICK_BLE_POLL_FLOOR;
#endif
#endif

    int32_t next_delay = MAX(effective_floor, selected_rate_ms);
    k_work_reschedule_for_queue(&analog_stick_wq, &scan_work,
                                K_MSEC(next_delay));
}

/* -------------------------------------------------------------------------- */
/* Coordinator init (SYS_INIT at APPLICATION level)                           */
/* -------------------------------------------------------------------------- */

#define REGISTER_STICK(n)                                                      \
    if (stick_count < MAX_STICKS) {                                            \
        const struct device *dev = DEVICE_DT_GET(n);                          \
        if (!device_is_ready(dev)) {                                           \
            LOG_ERR("[%s] analog stick device not ready, skipping", dev->name);                \
        } else {                                                               \
            stick_registry[stick_count].dev = dev;                             \
            stick_registry[stick_count].enable_gpio =                          \
                analog_stick_get_enable_gpio(dev);                             \
            stick_registry[stick_count].has_gpio =                             \
                analog_stick_has_enable_gpio(dev);                             \
            stick_registry[stick_count].pulse_read =                           \
                analog_stick_has_pulse_read(dev);                              \
            stick_registry[stick_count].settling_us =                          \
                analog_stick_get_settling_us(dev);                             \
            stick_count++;                                                     \
        }                                                                      \
    }

static int scan_coordinator_init(void) {
    stick_count = 0;
    DT_FOREACH_STATUS_OKAY(zmk_analog_stick, REGISTER_STICK)

    if (stick_count == 0) {
        LOG_WRN("No analog stick instances found");
        return 0;
    }

    build_gpio_groups();

    analog_stick_ensure_wq();

#if defined(CONFIG_ZMK_BLE) && !defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    bt_conn_cb_register(&scan_conn_callbacks);
#endif

    k_work_init_delayable(&scan_work, scan_coordinator_work_handler);

    /* Start scanning at the fastest idle rate so no stick is starved at boot */
    int32_t min_idle = INT32_MAX;
    for (size_t i = 0; i < stick_count; i++) {
        int32_t ip = analog_stick_get_wait_period_idle(stick_registry[i].dev);
        if (ip < min_idle) {
            min_idle = ip;
        }
    }
    if (min_idle == INT32_MAX) {
        min_idle = 0;
    }

    k_work_schedule_for_queue(&analog_stick_wq, &scan_work,
                              K_MSEC(min_idle > 0 ? min_idle : 100));

    LOG_INF("Scan coordinator started: %zu sticks, initial poll %d ms",
            stick_count, min_idle > 0 ? min_idle : 100);

    return 0;
}

BUILD_ASSERT(CONFIG_APPLICATION_INIT_PRIORITY >= CONFIG_ZMK_ANALOG_STICK_INIT_PRIORITY,
             "Scan coordinator must init after analog stick devices");

SYS_INIT(scan_coordinator_init, APPLICATION,
         CONFIG_APPLICATION_INIT_PRIORITY);
