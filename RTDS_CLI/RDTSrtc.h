#pragma once
#include <stdint.h>

/*
    RDTS RTC Abstraction
    Provides a strictly monotonic millisecond timebase with drift injection support.
*/

void rdtsrtc_init(void);

/* Return strictly monotonic milliseconds since boot */
uint64_t rdtsrtc_now_ms(void);

/* Optional calibration / drift injection (ppm) */
void rdtsrtc_set_slew_ppm(int32_t ppm);
void rdtsrtc_clear_slew_ppm(void);

/* Low level tick for diagnostics */
uint64_t rdtsrtc_raw_ticks(void);
uint32_t rdtsrtc_tick_hz(void);