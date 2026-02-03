#pragma once
#include <stdint.h>
#include <stdbool.h>

// Initialize RTC / discipline layer
void rtc_init(void);

// Update RTC from a received RDTS beacon
// unix_ms : timestamp extracted from beacon
// rx_ms   : local millis() when packet was received
void rtc_update_from_beacon(uint64_t unix_ms, uint32_t rx_ms);

// Get disciplined current time
uint64_t rtc_now_unix_ms(void);

// Diagnostics
bool    rtc_is_locked(void);
int32_t rtc_get_last_offset_ms(void);