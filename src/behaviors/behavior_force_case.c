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

/* -----------------------------------------------------------------------
 * Distinguishing sticky shift from physical shift:
 *
 * Both land in explicit_modifiers via zmk_hid_register_mod() so we cannot
 * tell them apart by reading explicit_modifiers alone.
 *
 * The trick: raise a LSHIFT release event on the bus and then immediately
 * check explicit_modifiers again.
 *
 *   - Sticky shift: the release event triggers sticky key's cleanup
 *     handler, which calls zmk_hid_unregister_mod(LSFT). Shift is now
 *     gone from explicit_modifiers → was sticky, release was correct.
 *
 *   - Physical shift: the release event goes through hid_listener which
 *     calls zmk_hid_unregister_mod(LSFT). But the key is still physically
 *     held, so the next scan cycle will re-press it. However we don't want
 *     to drop the shift even momentarily. So if after the release event
 *     shift is gone but was physical, we immediately re-press it by raising
 *     a LSHIFT press event to restore the state.
 *
 * We detect which case we are in by reading explicit_modifiers BEFORE and
 * AFTER raising the release:
 *   - If shift is gone after the release → sticky → done, leave it gone.
 *   - If shift is still there after the release → physical shift re-pressed
 *     itself already (or hid_listener didn't process yet). Either way we
 *     should NOT have released it → raise a compensating LSHIFT press.
 *
 * Actually the most robust detection: check zmk_hid_get_explicit_mods()
 * after the release event. If the ZMK event system processes synchronously
 * (which it does for non-deferred listeners), sticky key's unregister will
 * have already run by the time raise_zmk_keycode_state_changed_from_encoded
 * returns. Physical shift's hid_listener unregister will also have run,
 * dropping shift. Then we check: is the physical shift key still logically
 * pressed in ZMK? We can do this by checking if the LSHIFT *keycode* (not
 * modifier) is still in the pressed-keys tracking.
 *
 * ZMK tracks pressed keys via zmk_hid_is_pressed() for keycodes:
 *   zmk_hid_is_pressed(ZMK_HID_USAGE(HID_USAGE_KEY, LEFT_SHIFT_USAGE))
 * This returns true only if zmk_hid_press() was called for LSHIFT, which
 * only happens for physical key presses through hid_listener, NOT for
 * sticky key (which only calls zmk_hid_register_mod, not zmk_hid_press).
 *
 * So the detection is clean and simple:
 *   sticky_shift = shift in explicit_mods AND LSHIFT not in pressed keys
 *   physical_shift = shift in explicit_mods AND LSHIFT in pressed keys
 * ----------------------------------------------------------------------- */

/* HID usage code for Left Shift key (from HID spec, keyboard page) */
#define LSHIFT_USAGE 0xE1
#define RSHIFT_USAGE 0xE5

static bool is_sticky_shift(void) {
    if (!(zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS)) {
        return false; /* no shift at all */
    }
    /*
     * If physical shift is held, hid_listener called zmk_hid_press() for
     * the LSHIFT or RSHIFT keycode, so it appears in the pressed-keys
     * bitmap. Sticky shift only called zmk_hid_register_mod(), so it does
     * NOT appear in pressed-keys.
     */
    bool lshift_key_pressed = zmk_hid_is_pressed(ZMK_HID_USAGE(HID_USAGE_KEY, LSHIFT_USAGE));
    bool rshift_key_pressed = zmk_hid_is_pressed(ZMK_HID_USAGE(HID_USAGE_KEY, RSHIFT_USAGE));
    return !lshift_key_pressed && !rshift_key_pressed;
}

static int send_key(uint32_t keycode, bool pressed, bool want_upper, bool shift_was_sticky) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active  = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    bool report_shift = want_upper ^ caps_active;

    zmk_hid_masked_modifiers_set(ZMK_SHIFT_MODS);

    if (report_shift) {
        zmk_hid_implicit_modifiers_press(MOD_LSFT);
    }

    int ret;
    if (pressed) {
        ret = zmk_hid_press(ZMK_HID_USAGE(HID_USAGE_KEY, keycode));
    } else {
        ret = zmk_hid_release(ZMK_HID_USAGE(HID_USAGE_KEY, keycode));
    }

    if (ret == 0) {
        ret = zmk_endpoints_send_report(HID_USAGE_KEY);
    }

    if (report_shift) {
        zmk_hid_implicit_modifiers_release();
    }
    zmk_hid_masked_modifiers_clear();

    /*
     * On key release: if shift was sticky at press time, trigger its
     * cleanup now by raising a LSHIFT release on the bus.
     * Physical shift is NOT released — it stays held naturally.
     */
    if (!pressed && shift_was_sticky) {
        raise_zmk_keycode_state_changed_from_encoded(LSHIFT, false, k_uptime_get());
    }

    return ret;
}

/* -----------------------------------------------------------------------
 * FORCE-UPPER (fucase)
 * Ignores CapsLock. Shift inverts: no shift → upper, shift → lower.
 * ----------------------------------------------------------------------- */
#define DT_DRV_COMPAT zmk_behavior_force_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    bool shift_held   = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    bool shift_sticky = is_sticky_shift();
    return send_key(binding->param1, true, !shift_held, shift_sticky);
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
