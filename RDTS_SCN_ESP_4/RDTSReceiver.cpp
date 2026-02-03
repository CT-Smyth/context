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

static bool g_have_beacon = false;
static uint64_t g_last_beacon_unix_ms = 0;
static uint32_t g_accepted_beacon_count = 0;

// Reacquire mode: accept next beacon and re-anchor, bypassing time-backwards + estimate gate once.
static bool g_reacquire_armed = false;
static bool g_reacquire_preserve_freq = true;

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
  g_reacquire_armed = false;
  g_reacquire_preserve_freq = true;
}

void rdts_receiver_begin_reacquire(bool preserve_freq) {
  // Reset receiver-side latches so we don't deadlock on "time went backwards".
  g_have_beacon = false;
  g_last_beacon_unix_ms = 0;
  g_accepted_beacon_count = 0;

  // Arm one-shot bypass + re-anchor on next decoded beacon.
  g_reacquire_preserve_freq = preserve_freq;
  g_reacquire_armed = true;
}

RDTSRxResult rdts_receiver_on_packet(const rdts_packet_t &pkt, uint32_t rtc_rx_ms)
{
  RDTSRxResult r = {};
  r.result = RDTS_RX_REJECTED_POLICY;
  r.est_error_ms = 0;

  // -----------------------------------------------------------------------
  // One-shot reacquire: accept next beacon and re-anchor disciplined time.
  // This bypasses:
  //  - receiver monotonic latch (time went backwards)
  //  - estimate gate (10s) which can deadlock if master rebooted
  //  - time_on_beacon() hard outlier reject (±100ms) by using time_reanchor()
  // -----------------------------------------------------------------------
  if (g_reacquire_armed) {
    // Establish absolute RTC epoch on first accepted beacon only
    if (!rtc_epoch_is_set()) {
      rtc_set_unix_ms(pkt.master_unix_ms, rtc_rx_ms);
    }

    g_last_beacon_unix_ms = pkt.master_unix_ms;
    g_have_beacon = true;
    g_accepted_beacon_count = 1;

    r.time_report = time_reanchor(pkt.master_unix_ms, rtc_rx_ms, g_reacquire_preserve_freq);
    r.result = RDTS_RX_ACCEPTED;

    g_reacquire_armed = false;
    return r;
  }

  // ---- Beacon monotonicity (receiver-side) ----
  if (g_have_beacon) {
    if (pkt.master_unix_ms < g_last_beacon_unix_ms) {
      r.result = RDTS_RX_REJECTED_TIME_BACKWARDS;
      return r;
    }
  }

  // ---- Predict local time WITHOUT updating discipline ----
  // If disciplined time is not initialized yet, we accept first beacon unconditionally.
  if (time_is_initialized()) {
    uint64_t est_local_unix_ms = time_predict_unix_ms(rtc_rx_ms);
    int64_t err_est_vs_beacon_ms = (int64_t)pkt.master_unix_ms - (int64_t)est_local_unix_ms;
    r.est_error_ms = err_est_vs_beacon_ms;

    if (llabs(err_est_vs_beacon_ms) > MAX_EST_VS_BEACON_ERR_MS) {
      r.result = RDTS_RX_REJECTED_ESTIMATE_ERROR;
      return r;
    }
  } else {
    r.est_error_ms = 0;
  }

  // ---- ACCEPT ----
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