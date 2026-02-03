#include "TimeDisciplined.h"
#include <math.h>

// ============================================================================
// Tuning knobs  (identical intent to Python model)
// ============================================================================

// Phase (PLL)
static const double K_PHASE = 0.15;          // Kp
static const int64_t PHASE_REJECT_MS = 100;  // hard outlier reject (±100 ms)

// Frequency (FLL)
static const double K_FREQ = 0.01;          // Kf
static const double MAX_RATE_PPM = 2000.0;  // clamp frequency estimate

// Δt bounds (ms) for frequency estimation
static const uint32_t DT_MIN_MS = 30000;    // 30 s minimum
static const uint32_t DT_MAX_MS = 3000000;  // 3000 s maximum
static const int64_t FREQ_DEADBAND_MS = 5;  // ±5 ms deadband for frequency updates

// Monotonic clamp
static const int64_t MONO_SLOP_MS = 0;

// ============================================================================
// Internal state
// ============================================================================

static bool g_init = false;

// ---------------- Raw (undisciplined) mapping ----------------
static int64_t g_raw_offset_ms = 0;

// ---------------- Disciplined clock state ----------------
//
// Anchored model:
//   T_pred = epoch_unix_ms
//          + (rtc_now_ms - epoch_rtc_ms) * (1 + freq_ppm*1e-6)
//          + phase_ms
//
static uint32_t g_epoch_rtc_ms = 0;
static int64_t g_epoch_unix_ms = 0;

static double g_freq_ppm = 0.0;  // frequency estimate (FLL state)
static double g_phase_ms = 0.0;  // phase correction (PLL state)

// For Δt computation
static bool g_have_prev = false;
static uint32_t g_prev_rtc_ms = 0;

// Monotonic latch
static bool g_have_local = false;
static int64_t g_last_local_ms = 0;

// ============================================================================
// Helpers
// ============================================================================

static inline int64_t clamp_i64(int64_t x, int64_t lo, int64_t hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline double clamp_d(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Predict disciplined unix time at a given rtc value (no mutation)
static inline int64_t predict_local_ms(uint32_t rtc_ms) {
  int64_t drtc = (int64_t)(uint32_t)(rtc_ms - g_epoch_rtc_ms);
  double rate = 1.0 + g_freq_ppm * 1e-6;
  double t = (double)g_epoch_unix_ms
             + (double)drtc * rate
             + g_phase_ms;
  return (int64_t)llround(t);
}

// ============================================================================
// Public API
// ============================================================================

bool time_is_initialized() {
  return g_init;
}

void time_init() {
  g_init = false;

  g_raw_offset_ms = 0;

  g_epoch_rtc_ms = 0;
  g_epoch_unix_ms = 0;
  g_freq_ppm = 0.0;
  g_phase_ms = 0.0;

  g_have_prev = false;
  g_prev_rtc_ms = 0;

  g_have_local = false;
  g_last_local_ms = 0;
}

uint64_t time_predict_unix_ms(uint32_t rtc_now_ms) {
  if (!g_init) return 0;
  int64_t t = predict_local_ms(rtc_now_ms);
  return (t < 0) ? 0ULL : (uint64_t)t;
}

uint64_t time_now_unix_ms(uint32_t rtc_now_ms) {
  if (!g_init) return 0;

  int64_t t = predict_local_ms(rtc_now_ms);

  if (g_have_local) {
    if (t < (g_last_local_ms + MONO_SLOP_MS)) {
      t = g_last_local_ms + MONO_SLOP_MS;
    }
  }

  g_have_local = true;
  g_last_local_ms = t;

  return (t < 0) ? 0ULL : (uint64_t)t;
}

// ============================================================================
// Beacon discipline
// ============================================================================

TimeBeaconReport time_on_beacon(uint64_t beacon_unix_ms, uint32_t rtc_rx_ms) {
  TimeBeaconReport r = {};
  r.rtc_rx_ms = rtc_rx_ms;
  r.beacon_unix_ms = beacon_unix_ms;
  r.initialized = g_init;

  // ---------------- First beacon: hard initialization ----------------
  if (!g_init) {
    g_raw_offset_ms = (int64_t)beacon_unix_ms - (int64_t)rtc_rx_ms;

    g_epoch_rtc_ms = rtc_rx_ms;
    g_epoch_unix_ms = (int64_t)beacon_unix_ms;

    g_freq_ppm = 0.0;
    g_phase_ms = 0.0;

    g_have_prev = true;
    g_prev_rtc_ms = rtc_rx_ms;

    g_have_local = true;
    g_last_local_ms = (int64_t)beacon_unix_ms;

    g_init = true;

    r.raw_unix_ms = (int64_t)rtc_rx_ms + g_raw_offset_ms;
    r.local_unix_ms_pre = g_last_local_ms;
    r.local_unix_ms_post = g_last_local_ms;
    r.delta_rtc_vs_beacon_ms = 0;
    r.delta_real_vs_beacon_ms = 0;
    r.A_ppm = g_freq_ppm;
    r.B_ms = g_epoch_unix_ms - (int64_t)g_epoch_rtc_ms;
    r.initialized = true;
    return r;
  }

  // ---------------- Raw mapping ----------------
  int64_t raw_unix = (int64_t)rtc_rx_ms + g_raw_offset_ms;

  // ---------------- Pre-update prediction ----------------
  int64_t local_pre = predict_local_ms(rtc_rx_ms);

  int64_t delta_rtc = (int64_t)beacon_unix_ms - raw_unix;
  int64_t delta_real = (int64_t)beacon_unix_ms - local_pre;

  // ---------------- Discipline update ----------------
  bool accepted = false;

  if (g_have_prev) {
    uint32_t dt_ms = (uint32_t)(rtc_rx_ms - g_prev_rtc_ms);
    if (dt_ms >= DT_MIN_MS && dt_ms <= DT_MAX_MS) {

      // Hard outlier reject (popcorn noise protection)
      if (llabs(delta_real) <= PHASE_REJECT_MS) {

        // PLL: phase update
        g_phase_ms += K_PHASE * (double)delta_real;

        // FLL: frequency update (ppm)
        // Only integrate frequency when phase error exceeds jitter deadband
        if (llabs(delta_real) > FREQ_DEADBAND_MS) {
          double df_ppm = K_FREQ * ((double)delta_real / (double)dt_ms) * 1e6;
          g_freq_ppm = clamp_d(g_freq_ppm + df_ppm,
                               -MAX_RATE_PPM,
                               +MAX_RATE_PPM);
        }

        accepted = true;
      }
    }
  }

  // ---------------- Re-anchor on accepted beacon ----------------
  int64_t local_post;

  if (accepted) {
    // Compute post-update time at this rtc
    local_post = predict_local_ms(rtc_rx_ms);

    // Enforce monotonic
    if (g_have_local) {
      if (local_post < (g_last_local_ms + MONO_SLOP_MS)) {
        local_post = g_last_local_ms + MONO_SLOP_MS;
      }
    }

    // Re-anchor: critical to avoid elapsed-time instability
    g_epoch_rtc_ms = rtc_rx_ms;
    g_epoch_unix_ms = local_post;
    g_phase_ms = 0.0;
  } else {
    local_post = local_pre;
  }

  g_have_local = true;
  g_last_local_ms = local_post;

  if (accepted) {
    g_have_prev = true;
    g_prev_rtc_ms = rtc_rx_ms;
  }

  // ---------------- Report ----------------
  r.raw_unix_ms = raw_unix;
  r.local_unix_ms_pre = local_pre;
  r.local_unix_ms_post = local_post;
  r.delta_rtc_vs_beacon_ms = delta_rtc;
  r.delta_real_vs_beacon_ms = delta_real;
  r.A_ppm = g_freq_ppm;
  r.B_ms = g_epoch_unix_ms - (int64_t)g_epoch_rtc_ms;
  r.initialized = g_init;

  return r;
}