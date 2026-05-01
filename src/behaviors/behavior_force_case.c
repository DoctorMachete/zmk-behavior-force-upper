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

#include <dt-bindings/zmk/modifiers.h>

#define ZMK_LED_CAPSLOCK_BIT BIT(1) // *** from https://github.com/darknao/zmk/blob/2fad527cc5abed5bb59b4d4a4b0ee511d0e514e9/app/src/rgb_underglow.c#L320 ***

/* Shift modifier flags covering both left and right shift */
#define ZMK_SHIFT_MODS (MOD_LSFT | MOD_RSFT)

/* -----------------------------------------------------------------------
 * Shared helper.
 *
 * want_upper: true  → we want to produce an uppercase letter
 *             false → we want to produce a lowercase letter
 *
 * Strategy — never raise fake LSHIFT press/release events onto the bus.
 * That would interfere with other behaviors (hold-tap, combos, etc.) that
 * are listening to keycode_state_changed.  Instead we use the same two
 * mechanisms that behavior_mod_morph.c uses:
 *
 *   • zmk_hid_masked_modifiers_set()  — suppress explicit shift from the
 *     HID report for the duration of the keycode event, without generating
 *     any modifier events on the bus.
 *   • implicit_modifiers on the keycode event — inject shift into the HID
 *     report for just this keycode, again without modifier bus events.
 *
 * This guarantees that the externally visible shift state is identical
 * before and after the behavior fires.
 * ----------------------------------------------------------------------- */
static int send_key(uint32_t keycode, bool pressed, bool want_upper,
                    int64_t timestamp) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active  = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    bool shift_held   = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;

    /*
     * Determine what the host would naturally produce given the current
     * caps + shift state, then decide what corrections to apply.
     *
     * Natural output matrix:
     *   caps OFF, shift OFF → lowercase
     *   caps OFF, shift ON  → uppercase
     *   caps ON,  shift OFF → uppercase
     *   caps ON,  shift ON  → lowercase
     *
     * natural_upper = caps_active XOR shift_held
     */
    bool natural_upper = caps_active ^ shift_held;

    bool mask_shift   = false;
    bool inject_shift = false;

    if (want_upper && !natural_upper) {
        /* Need uppercase but host would send lowercase → inject shift */
        inject_shift = true;
    } else if (!want_upper && natural_upper) {
        /* Need lowercase but host would send uppercase → mask shift */
        mask_shift = true;
    }
    /* Otherwise natural output already matches — send bare keycode */

    int ret;

    if (mask_shift) {
        zmk_hid_masked_modifiers_set(ZMK_SHIFT_MODS);
    }

    struct zmk_keycode_state_changed *ev =
        new_zmk_keycode_state_changed_from_encoded(keycode, pressed, timestamp);
    if (ev == NULL) {
        if (mask_shift) {
            zmk_hid_masked_modifiers_clear();
        }
        return -ENOMEM;
    }

    if (inject_shift) {
        ev->implicit_modifiers |= MOD_LSFT;
    }

    ret = raise_zmk_keycode_state_changed(ev);

    if (mask_shift) {
        zmk_hid_masked_modifiers_clear();
    }

    return ret;
}

/* -----------------------------------------------------------------------
 * FORCE-UPPER
 * Goal: always produce uppercase. With shift held, produce lowercase.
 *
 * shift not held → want_upper = true
 * shift held     → want_upper = false  (shift inverts)
 * ----------------------------------------------------------------------- */
#define DT_DRV_COMPAT zmk_behavior_force_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, !shift_held, event.timestamp);
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, false, !shift_held, event.timestamp);
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
 * FORCE-LOWER
 * Goal: always produce lowercase. With shift held, produce uppercase.
 *
 * shift not held → want_upper = false
 * shift held     → want_upper = true   (shift inverts)
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_lower

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_lower_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, shift_held, event.timestamp);
}

static int on_force_lower_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, false, shift_held, event.timestamp);
}

static const struct behavior_driver_api force_lower_driver_api = {
    .binding_pressed  = on_force_lower_binding_pressed,
    .binding_released = on_force_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
