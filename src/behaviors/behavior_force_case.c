/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_force_upper  /* redefined per-driver below */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/behavior.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>
#include <zmk/hid_indicators.h>
#include <zmk/endpoints.h>

#include <dt-bindings/zmk/modifiers.h>
#include <dt-bindings/zmk/hid_usage_pages.h>

/* Standard ZMK v0.3.0 capslock indicator bit via the public API */
#define CAPSLOCK_ACTIVE \
    (zmk_hid_indicators_get_current_profile() & ZMK_HID_INDICATORS_CAPSLOCK)

#define ZMK_SHIFT_MODS (MOD_LSFT | MOD_RSFT)

#define LSHIFT_USAGE 0xE1
#define RSHIFT_USAGE 0xE5

#define KEY_PRESS DEVICE_DT_NAME(DT_INST(0, zmk_behavior_key_press))

/* -----------------------------------------------------------------------
 * Per-key press state — snapshotted at press time, reused at release.
 * ----------------------------------------------------------------------- */
struct force_case_state {
    bool shift_held;
};

/* -----------------------------------------------------------------------
 * Detect sticky shift vs physical shift.
 * Sticky: in explicit_mods but NOT in pressed-keys bitmap.
 * Physical: in explicit_mods AND in pressed-keys bitmap.
 * ----------------------------------------------------------------------- */
static bool is_sticky_shift(void) {
    if (!(zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS)) {
        return false;
    }
    bool lshift_key_pressed = zmk_hid_is_pressed(ZMK_HID_USAGE(HID_USAGE_KEY, LSHIFT_USAGE));
    bool rshift_key_pressed = zmk_hid_is_pressed(ZMK_HID_USAGE(HID_USAGE_KEY, RSHIFT_USAGE));
    return !lshift_key_pressed && !rshift_key_pressed;
}

/* -----------------------------------------------------------------------
 * Shared helper — mod-morph pattern via zmk_behavior_invoke_binding.
 *
 * Identical strategy to behavior_mod_morph.c:
 *   1. Set mask/register before invoke so hid_listener's SET_MODIFIERS
 *      sees the correct shift state when it builds the report.
 *   2. invoke_binding(press) → sticky key sees real keycode, releases.
 *   3. invoke_binding(release) immediately → no auto-repeat.
 *   4. Unregister/clear after invoke.
 *
 * report_shift = want_upper XOR caps_active
 *
 * need_mask: shift held, don't want it     → mask ZMK_SHIFT_MODS
 * need_reg:  shift not held, but want it   → register MOD_LSFT temporarily
 * ----------------------------------------------------------------------- */
static int send_key(uint32_t keycode, bool want_upper,
                    bool shift_held, struct zmk_behavior_binding_event event) {
    bool caps_active  = CAPSLOCK_ACTIVE != 0;
    bool report_shift = want_upper ^ caps_active;

    bool need_mask = shift_held && !report_shift;
    bool need_reg  = !shift_held && report_shift;

    if (need_mask) {
        zmk_hid_masked_modifiers_set(ZMK_SHIFT_MODS);
    }
    if (need_reg) {
        zmk_hid_register_mods(MOD_LSFT);
    }

    struct zmk_behavior_binding key_binding = {
        .behavior_dev = KEY_PRESS,
        .param1 = keycode,
    };

    /* Press — sticky key sees real keycode and releases naturally.
     * hid_listener builds report with our corrected modifier state. */
    int ret = zmk_behavior_invoke_binding(&key_binding, event, true);

    /* Immediate release — prevents auto-repeat on hold */
    zmk_behavior_invoke_binding(&key_binding, event, false);

    if (need_reg) {
        zmk_hid_unregister_mods(MOD_LSFT);
        /* Send one more report so host sees shift gone after the key */
        zmk_endpoints_send_report(HID_USAGE_KEY);
    }
    if (need_mask) {
        zmk_hid_masked_modifiers_clear();
    }

    return ret;
}

/* -----------------------------------------------------------------------
 * FORCE-UPPER (fucase)
 * Ignores CapsLock. Shift inverts: no shift → upper, shift → lower.
 * ----------------------------------------------------------------------- */
#define DT_DRV_COMPAT zmk_behavior_force_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static struct force_case_state force_upper_state[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_upper_state[0];
    state->shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, !state->shift_held, state->shift_held, event);
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    return 0;
}

static const struct behavior_driver_api force_upper_driver_api = {
    .binding_pressed  = on_force_upper_binding_pressed,
    .binding_released = on_force_upper_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_upper_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

/* -----------------------------------------------------------------------
 * FORCE-LOWER (flcase)
 * Ignores CapsLock. Shift inverts: no shift → lower, shift → upper.
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_lower

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static struct force_case_state force_lower_state[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];

static int on_force_lower_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_lower_state[0];
    state->shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, state->shift_held, state->shift_held, event);
}

static int on_force_lower_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    return 0;
}

static const struct behavior_driver_api force_lower_driver_api = {
    .binding_pressed  = on_force_lower_binding_pressed,
    .binding_released = on_force_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

/* -----------------------------------------------------------------------
 * FORCE-TRUE-UPPER (ftucase)
 * Always uppercase. Ignores CapsLock and Shift entirely.
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_true_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_true_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                               struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, shift_held, event);
}

static int on_force_true_upper_binding_released(struct zmk_behavior_binding *binding,
                                                struct zmk_behavior_binding_event event) {
    return 0;
}

static const struct behavior_driver_api force_true_upper_driver_api = {
    .binding_pressed  = on_force_true_upper_binding_pressed,
    .binding_released = on_force_true_upper_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_true_upper_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

/* -----------------------------------------------------------------------
 * FORCE-TRUE-LOWER (ftlcase)
 * Always lowercase. Ignores CapsLock and Shift entirely.
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_true_lower

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_true_lower_binding_pressed(struct zmk_behavior_binding *binding,
                                               struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, false, shift_held, event);
}

static int on_force_true_lower_binding_released(struct zmk_behavior_binding *binding,
                                                struct zmk_behavior_binding_event event) {
    return 0;
}

static const struct behavior_driver_api force_true_lower_driver_api = {
    .binding_pressed  = on_force_true_lower_binding_pressed,
    .binding_released = on_force_true_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_true_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
