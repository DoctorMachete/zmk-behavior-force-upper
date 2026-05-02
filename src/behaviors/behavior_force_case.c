/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

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

#define ZMK_LED_CAPSLOCK_BIT BIT(1) // *** from https://github.com/darknao/zmk/blob/2fad527cc5abed5bb59b4d4a4b0ee511d0e514e9/app/src/rgb_underglow.c#L320 ***

#define ZMK_SHIFT_MODS (MOD_LSFT | MOD_RSFT)

#define LSHIFT_USAGE 0xE1
#define RSHIFT_USAGE 0xE5

/* -----------------------------------------------------------------------
 * Per-key press state — snapshotted at press time, consumed at release.
 * ----------------------------------------------------------------------- */
struct force_case_state {
    bool shift_held;
    bool shift_sticky;
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
 * Shared helper.
 *
 * How sticky key works (from source):
 *
 *   Listener fires on zmk_keycode_state_changed events.
 *
 *   On key DOWN: records modified_key_usage_page/keycode, stops timer,
 *     and if quick_release && timer_started → queues release after reraise.
 *
 *   On key UP: if modified_key_usage_page/keycode matches → releases.
 *
 *   timer_started is set in on_sticky_key_binding_released (when the
 *   sticky key physical key is released), so it is always true by the
 *   time we press the next key.
 *
 * Therefore to make sticky key release correctly we MUST raise the real
 * keycode on the bus for both press and release. hid_listener will
 * process these events and modify the HID report — but we immediately
 * follow with our own masked/corrected send_report that overrides it.
 *
 * Sequence:
 *   1. raise_zmk_keycode_state_changed_from_encoded(keycode, pressed)
 *        → sticky key listener fires synchronously:
 *          - press: records keycode, queues release (quick_release)
 *          - release: sees matching keycode, triggers release_sticky_key_behavior
 *        → hid_listener also fires, modifying the report with wrong shift
 *   2. zmk_hid_masked_modifiers_set + implicit inject → correct shift
 *   3. zmk_hid_press/release → correct key state (already set by step 1,
 *        but we redo it to be safe)
 *   4. zmk_endpoints_send_report → send with corrected modifier state,
 *        overriding whatever hid_listener put in the report in step 1
 *   5. restore implicit + mask
 *
 * The key insight: ZMK's event system processes listeners synchronously
 * in priority order. raise_zmk_keycode_state_changed_from_encoded returns
 * only after all listeners (including sticky key and hid_listener) have
 * run. So by step 2 we know sticky key has already done its work, and
 * hid_listener has set the report — we then fix the report in steps 2-4.
 * ----------------------------------------------------------------------- */
static int send_key(uint32_t keycode, bool pressed, bool want_upper) {
    /*
     * Step 1: raise the real keycode on the bus.
     * This lets sticky key's listener fire and process the event correctly
     * (recording the keycode on press, releasing on key-up).
     * hid_listener will also fire and set the report — but we override it.
     */
    raise_zmk_keycode_state_changed_from_encoded(keycode, pressed, k_uptime_get());

    /*
     * Steps 2-5: override the HID report with the correct shift state.
     * At this point sticky key has already done its cleanup, so
     * explicit_modifiers reflects the post-sticky state.
     */
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active  = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    bool report_shift = want_upper ^ caps_active;

    zmk_hid_masked_modifiers_set(ZMK_SHIFT_MODS);

    if (report_shift) {
        zmk_hid_implicit_modifiers_press(MOD_LSFT);
    }

    /* Re-apply key state directly to be certain it's correct */
    if (pressed) {
        zmk_hid_press(ZMK_HID_USAGE(HID_USAGE_KEY, keycode));
    } else {
        zmk_hid_release(ZMK_HID_USAGE(HID_USAGE_KEY, keycode));
    }

    int ret = zmk_endpoints_send_report(HID_USAGE_KEY);

    if (report_shift) {
        zmk_hid_implicit_modifiers_release();
    }
    zmk_hid_masked_modifiers_clear();

    return ret;
}

/* -----------------------------------------------------------------------
 * FORCE-UPPER (fucase)
 * Ignores CapsLock. Shift inverts: no shift → upper, shift → lower.
 * want_upper is snapshotted at press and reused at release.
 * ----------------------------------------------------------------------- */
#define DT_DRV_COMPAT zmk_behavior_force_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static struct force_case_state force_upper_state[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_upper_state[0];
    state->shift_held   = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    state->shift_sticky = is_sticky_shift();
    return send_key(binding->param1, true, !state->shift_held);
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_upper_state[0];
    return send_key(binding->param1, false, !state->shift_held);
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
    state->shift_held   = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    state->shift_sticky = is_sticky_shift();
    return send_key(binding->param1, true, state->shift_held);
}

static int on_force_lower_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_lower_state[0];
    return send_key(binding->param1, false, state->shift_held);
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
    return send_key(binding->param1, true, true);
}

static int on_force_true_upper_binding_released(struct zmk_behavior_binding *binding,
                                                struct zmk_behavior_binding_event event) {
    return send_key(binding->param1, false, true);
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
    return send_key(binding->param1, true, false);
}

static int on_force_true_lower_binding_released(struct zmk_behavior_binding *binding,
                                                struct zmk_behavior_binding_event event) {
    return send_key(binding->param1, false, false);
}

static const struct behavior_driver_api force_true_lower_driver_api = {
    .binding_pressed  = on_force_true_lower_binding_pressed,
    .binding_released = on_force_true_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_true_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
