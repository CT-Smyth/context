#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * ScanScheduler
 *
 * Responsibility:
 *  - Decide WHEN to start a scan
 *  - Decide scan duration
 *
 * Inputs:
 *  - rtc time (ms)
 *  - disciplined unix time (ms, or 0 if unlocked)
 *  - scan active state
 *  - scan result events
 *  - accepted beacon events
 *
 * Outputs:
 *  - start scan now? (yes/no)
 *  - scan duration (ms)
 *
 * No BLE calls.
 * No Serial.
 * No policy decisions.
 */

typedef struct {
  uint32_t period_ms;
  uint32_t scan_duration_ms;
  uint32_t initial_phase_offset_ms; // e.g. 100
  bool     prelock_back_to_back;
} ScanSchedConfig;

typedef enum {
  SCAN_ACTION_NONE = 0,
  SCAN_ACTION_START,
} scan_action_kind_t;

typedef struct {
  scan_action_kind_t kind;
  uint32_t duration_ms;
} ScanAction;

#ifndef SCAN_MISSED_SYNC_THRESHOLD
#define SCAN_MISSED_SYNC_THRESHOLD 5  //TODO 15 PRODUCTION
#endif

uint32_t scan_sched_consecutive_misses(void);


void scan_sched_init(const ScanSchedConfig *cfg);

// Main polling decision point
// unix_now_ms == 0 means "not yet disciplined"
ScanAction scan_sched_poll(
  uint32_t rtc_now_ms,
  uint64_t unix_now_ms,
  bool scan_active
);

// Event hooks
void scan_sched_on_scan_started(uint32_t rtc_now_ms);
void scan_sched_on_scan_finished(bool had_packet);
// Accepted beacon (discipline succeeded)
void scan_sched_on_beacon_accepted(uint64_t beacon_unix_ms);

// Force scheduler back to pre-lock/aggressive mode (as-if reset), preserving config.
void scan_sched_force_prelock(void);