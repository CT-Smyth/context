#include "RtcClock.h"
#include <esp_timer.h>

// ---------------- Internal state ----------------

// Offset such that:
//   unix_ms = rtc_now_ms() + g_epoch_offset_ms
static int64_t g_epoch_offset_ms = 0;
static bool    g_epoch_set = false;

// ---------------- Implementation ----------------

// esp_timer_get_time() returns microseconds since boot (monotonic)
uint32_t rtc_now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

uint64_t rtc_now_unix_ms(void) {
  if (!g_epoch_set) {
    return 0;
  }

  return (uint64_t)((int64_t)rtc_now_ms() + g_epoch_offset_ms);
}

void rtc_set_unix_ms(uint64_t unix_ms, uint32_t rtc_ms_at_set) {
  // Compute offset so that:
  //   unix_ms == rtc_ms_at_set + offset
  g_epoch_offset_ms = (int64_t)unix_ms - (int64_t)rtc_ms_at_set;
  g_epoch_set = true;
}

bool rtc_epoch_is_set(void) {
  return g_epoch_set;
}