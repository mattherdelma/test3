/*
 * timing_engine.h - Microsecond-precision key-event scheduler for the Leonardo
 * keyboard gateway.
 *
 * The gateway receives a GW_MSG_KEY_SEQUENCE frame containing an ordered list
 * of key records (see gateway_protocol.h). Each record carries a
 * `delay_before_us` that the engine honours before executing the action.
 *
 * Timing strategy
 * ---------------
 * For delays >= ~16 ms we yield in a loop so USB/serial servicing continues.
 * For sub-millisecond delays we busy-wait on micros() with rollover-safe
 * comparison, which gives reliable single-microsecond resolution on the
 * 16 MHz ATmega32U4 (micros() granularity is 4 us; we clamp to that floor).
 *
 * NOTE: the *real-world* limit on injection rate is the USB HID polling
 * interval (1 ms for a standard full-speed boot keyboard). The engine schedules
 * key-state transitions at microsecond precision, but the host still samples
 * the HID endpoint at its bInterval. This header documents that ceiling rather
 * than pretending to beat it.
 */
#ifndef TIMING_ENGINE_H
#define TIMING_ENGINE_H

#include <Arduino.h>
#include <Keyboard.h>
#include "../protocol/gateway_protocol.h"

/* micros() resolution on the 32U4 is 4 us; don't claim finer than that. */
#define TE_MICROS_FLOOR 4u

/* Rollover-safe "has `target` been reached since `start`?" for unsigned us. */
static inline bool te_elapsed(uint32_t start, uint32_t target_us)
{
    return (uint32_t)(micros() - start) >= target_us;
}

/* Precise delay in microseconds, rollover-safe, with cooperative yield for
 * long waits so the CPU keeps servicing USB. */
static inline void te_delay_us(uint32_t us)
{
    if (us == 0) {
        return;
    }
    if (us < TE_MICROS_FLOOR) {
        us = TE_MICROS_FLOOR;
    }

    const uint32_t start = micros();
    /* Long portion: yield in ~8 ms slices so USB stays alive. */
    while (us >= 8000u && !te_elapsed(start, us - 4000u)) {
        delay(4); /* delay() keeps the USB stack serviced on the 32U4 */
        yield();
    }
    /* Short tail: tight busy-wait for microsecond accuracy. */
    while (!te_elapsed(start, us)) {
        /* spin */
    }
}

/* Translate our modifier bitmap into the matching Arduino Keyboard key codes
 * and press/release them. Returns nothing; caller pairs press with release. */
static inline void te_apply_modifiers(uint8_t mods, bool press)
{
    struct { uint8_t bit; uint8_t key; } map[] = {
        { GW_MOD_LCTRL,  KEY_LEFT_CTRL  },
        { GW_MOD_LSHIFT, KEY_LEFT_SHIFT },
        { GW_MOD_LALT,   KEY_LEFT_ALT   },
        { GW_MOD_LGUI,   KEY_LEFT_GUI   },
        { GW_MOD_RCTRL,  KEY_RIGHT_CTRL },
        { GW_MOD_RSHIFT, KEY_RIGHT_SHIFT},
        { GW_MOD_RALT,   KEY_RIGHT_ALT  },
        { GW_MOD_RGUI,   KEY_RIGHT_GUI  },
    };
    for (uint8_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
        if (mods & map[i].bit) {
            if (press) { Keyboard.press(map[i].key); }
            else       { Keyboard.release(map[i].key); }
        }
    }
}

/*
 * Execute a single key record.
 *
 * `tap_hold_us` is the press-to-release dwell used for GW_KEY_TAP so that a tap
 * survives at least one host HID poll. Returns true on success.
 */
static inline bool te_execute_record(uint8_t action, uint8_t keycode,
                                     uint8_t mods, uint16_t delay_before_us,
                                     uint16_t tap_hold_us)
{
    te_delay_us(delay_before_us);

    switch (action) {
    case GW_KEY_PRESS:
        te_apply_modifiers(mods, true);
        if (keycode) { Keyboard.press(keycode); }
        return true;

    case GW_KEY_RELEASE:
        if (keycode) { Keyboard.release(keycode); }
        te_apply_modifiers(mods, false);
        return true;

    case GW_KEY_TAP:
        te_apply_modifiers(mods, true);
        if (keycode) { Keyboard.press(keycode); }
        te_delay_us(tap_hold_us);
        if (keycode) { Keyboard.release(keycode); }
        te_apply_modifiers(mods, false);
        return true;

    case GW_KEY_RELALL:
        Keyboard.releaseAll();
        return true;

    default:
        return false;
    }
}

#endif /* TIMING_ENGINE_H */
