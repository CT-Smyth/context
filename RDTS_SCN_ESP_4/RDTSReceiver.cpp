#include "RDTSReceiver.h"
#include "TimeDisciplined.h"
#include "RtcClock.h"
#include <stdlib.h>

// ---------------- Policy parameters ----------------

// Max allowed disagreement between estimated local time and beacon
#define MAX_EST_VS_BEACON_ERR_MS 10000  // 10 seconds

// Number of accepted beacons required before declaring LOCKED
#define LOCK_BEACON_COUNT 3

// ---------------- Internal state ----------------

static bool     g_have_beacon = false;
static uint64_t g_last_beacon_unix_ms = 0;
static uint32_t g_accepted_beacon_count = 0;

// ---------------- Optional RTC epoch hook ----------------
//
// This is intentionally weakly-coupled.
// If you later add rtc_set_unix_ms(), enable this.
//

//#define RDTS_HAVE_RTC_SET_EPOCH 1

#ifdef RDTS_HAVE_RTC_SET_EPOCH
void rtc_set_unix_ms(uint64_t unix_ms, uint32_t rtc_now_ms);
#endif

#ifdef RDTS_HAVE_RTC_SET_EPOCH
  if (!rtc_epoch_is_set()) {
    rtc_set_unix_ms(pkt.master_unix_ms, rtc_rx_ms);
  }
#endif

// ---------------- API ----------------

void rdts_receiver_init(void) {
  g_have_beacon = false;
  g_last_beacon_unix_ms = 0;
  g_accepted_beacon_count = 0;
}

RDTSRxResult rdts_receiver_on_packet(
  const rdts_packet_t &pkt,
  uint32_t rtc_rx_ms
) {
  RDTSRxResult r = {};
  r.result = RDTS_RX_REJECTED_POLICY;
  r.est_error_ms = 0;

  // ---- Beacon monotonicity ----
  if (g_have_beacon) {
    if (pkt.master_unix_ms < g_last_beacon_unix_ms) {
      r.result = RDTS_RX_REJECTED_TIME_BACKWARDS;
      return r;
    }
  }

  // ---- Predict local time WITHOUT updating discipline ----
  //
  // IMPORTANT:
  // If disciplined time is not initialized yet, we MUST accept
  // the first beacon unconditionally.
  //
  if (time_is_initialized()) {

    uint64_t est_local_unix_ms = time_predict_unix_ms(rtc_rx_ms);
    int64_t err_est_vs_beacon_ms =
      (int64_t)pkt.master_unix_ms - (int64_t)est_local_unix_ms;

    r.est_error_ms = err_est_vs_beacon_ms;

    if (llabs(err_est_vs_beacon_ms) > MAX_EST_VS_BEACON_ERR_MS) {
      r.result = RDTS_RX_REJECTED_ESTIMATE_ERROR;
      return r;
    }

  } else {
    // No estimate exists yet
    r.est_error_ms = 0;
  }

  // ---- ACCEPT ----

// ---- ACCEPT ----

// Establish absolute RTC epoch on first accepted beacon only
if (!rtc_epoch_is_set()) {
  rtc_set_unix_ms(pkt.master_unix_ms, rtc_rx_ms);
}

g_last_beacon_unix_ms = pkt.master_unix_ms;
g_have_beacon = true;
g_accepted_beacon_count++;

  r.time_report = time_on_beacon(pkt.master_unix_ms, rtc_rx_ms);
  r.result = RDTS_RX_ACCEPTED;
  return r;
}

rdts_time_quality_t rdts_receiver_time_quality(void) {
  if (!g_have_beacon) {
    return RDTS_TIME_QUALITY_INVALID;
  }

  if (g_accepted_beacon_count < LOCK_BEACON_COUNT) {
    return RDTS_TIME_QUALITY_LOCKING;
  }

  return RDTS_TIME_QUALITY_LOCKED;
}