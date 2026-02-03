#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * Disciplined time model (external contract unchanged):
 *
 *   - rtc_ms is a monotonic hardware timebase (currently millis()).
 *   - time_now_unix_ms() returns a disciplined unix time estimate.
 *   - time_predict_unix_ms() predicts without mutating state.
 *   - time_on_beacon() is the ONLY place where discipline occurs.
 *
 * Internally, the implementation is now:
 *   - PLL (phase) + FLL (frequency)
 *   - anchored at last accepted beacon
 *   - NO dependence on total elapsed rtc time
 */

struct TimeBeaconReport {
  uint32_t rtc_rx_ms;
  uint64_t beacon_unix_ms;

  int64_t  raw_unix_ms;
  int64_t  local_unix_ms_pre;
  int64_t  local_unix_ms_post;

  int64_t  delta_rtc_vs_beacon_ms;
  int64_t  delta_real_vs_beacon_ms;

  double   A_ppm;     // now reports frequency estimate directly (ppm)
  int64_t  B_ms;      // diagnostic offset term (epoch_unix_ms - epoch_rtc_ms)
  bool     initialized;
};

void time_init();
bool time_is_initialized();

uint64_t time_predict_unix_ms(uint32_t rtc_now_ms);
uint64_t time_now_unix_ms(uint32_t rtc_now_ms);

TimeBeaconReport time_on_beacon(uint64_t beacon_unix_ms, uint32_t rtc_rx_ms);