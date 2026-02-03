#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * RtcClock
 * --------
 *
 * Provides a monotonic millisecond timebase and an optional
 * absolute Unix epoch mapping.
 *
 * The monotonic clock NEVER jumps.
 * Absolute time is derived via an offset set exactly once
 * (typically from the first trusted beacon).
 */

// Monotonic RTC timebase in milliseconds since boot
uint32_t rtc_now_ms(void);

// Absolute Unix time in milliseconds, if epoch is known.
// Returns 0 if epoch has not been set.
uint64_t rtc_now_unix_ms(void);

// Set absolute Unix epoch corresponding to a given rtc_now_ms() sample.
// Intended to be called once on first trusted beacon.
void rtc_set_unix_ms(uint64_t unix_ms, uint32_t rtc_now_ms);

// True if rtc_set_unix_ms() has been called
bool rtc_epoch_is_set(void);