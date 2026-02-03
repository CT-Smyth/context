#include "ScanScheduler.h"

// ---------------- Internal state ----------------

static ScanSchedConfig cfg;

static bool     locked = false;
static uint64_t next_scan_unix_ms = 0;
static int32_t  phase_offset_ms = 0;

// ------------------------------------------------

void scan_sched_init(const ScanSchedConfig *c) {
  cfg = *c;
  locked = false;
  next_scan_unix_ms = 0;
  phase_offset_ms = (int32_t)c->initial_phase_offset_ms;
}

ScanAction scan_sched_poll(
  uint32_t rtc_now_ms,
  uint64_t unix_now_ms,
  bool scan_active
) {
  ScanAction act = { SCAN_ACTION_NONE, 0 };

  if (scan_active) {
    return act;
  }

  // ---------------- Pre-lock ----------------
  if (!locked || unix_now_ms == 0) {
    if (cfg.prelock_back_to_back) {
      act.kind = SCAN_ACTION_START;
      act.duration_ms = cfg.scan_duration_ms;
    }
    return act;
  }

  // ---------------- Locked ----------------
  if (unix_now_ms >= next_scan_unix_ms) {
    act.kind = SCAN_ACTION_START;
    act.duration_ms = cfg.scan_duration_ms;

    // advance exactly one period
    next_scan_unix_ms += cfg.period_ms;
  }

  return act;
}

void scan_sched_on_scan_started(uint32_t rtc_now_ms) {
  (void)rtc_now_ms;
  // no-op for now
}

void scan_sched_on_scan_finished(bool had_sync) {
  // Scheduler is policy-free: application decides if/when recovery is needed.
  (void)had_sync;
}

void scan_sched_on_beacon_accepted(uint64_t beacon_unix_ms) {
  if (!locked) {
    uint64_t next_boundary =
      ((beacon_unix_ms / cfg.period_ms) + 1ULL) * cfg.period_ms;

    // Center scan window on boundary, then apply latency offset
    next_scan_unix_ms =
      next_boundary + (int64_t)phase_offset_ms;

    locked = true;
  }
}

void scan_sched_force_prelock(void)
{
  locked = false;
  next_scan_unix_ms = 0;
  // keep phase_offset_ms and cfg as configured
}
