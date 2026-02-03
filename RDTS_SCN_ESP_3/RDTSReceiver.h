#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "RDTS_packet.h"
#include "TimeDisciplined.h"


// Result of processing a beacon
typedef enum {
  RDTS_RX_ACCEPTED = 0,
  RDTS_RX_REJECTED_POLICY,
  RDTS_RX_REJECTED_TIME_BACKWARDS,
  RDTS_RX_REJECTED_ESTIMATE_ERROR,
} rdts_rx_result_t;

typedef enum {
  RDTS_TIME_QUALITY_INVALID = 0,  // no accepted beacon yet
  RDTS_TIME_QUALITY_LOCKING,      // have beacon(s), still converging
  RDTS_TIME_QUALITY_LOCKED,       // stable A/B, small errors
  RDTS_TIME_QUALITY_HOLDOVER      // no recent beacon, extrapolating
} rdts_time_quality_t;

struct RDTSRxResult {
  rdts_rx_result_t result;
  TimeBeaconReport time_report;  // valid only if ACCEPTED
  int64_t est_error_ms;          // for diagnostics on rejection
};

// Initialize internal policy state
void rdts_receiver_init(void);

// Process one decoded RDTS packet
RDTSRxResult rdts_receiver_on_packet(const rdts_packet_t &pkt, uint32_t rtc_rx_ms);

// Time quality / confidence reporting
rdts_time_quality_t rdts_receiver_time_quality();

/*
 * RDTS Time Quality Semantics
 * ---------------------------
 *
 * rdts_time_quality_t describes the *trust level* of the local disciplined
 * time estimate derived from RDTS beacons.
 *
 * This is a diagnostic / policy signal only. It does NOT directly affect
 * clock discipline behavior.
 *
 * Meanings:
 *
 *   RDTS_TIME_QUALITY_INVALID
 *     - No accepted beacon has ever been processed
 *     - Local time is undefined / untrusted
 *
 *   RDTS_TIME_QUALITY_LOCKING
 *     - Beacons are being received and accepted
 *     - Discipline parameters (A/B) are still converging
 *     - Expect measurable error and drift correction activity
 *
 *   RDTS_TIME_QUALITY_LOCKED
 *     - Discipline is stable
 *     - Estimated local time closely matches beacon time
 *     - Suitable for timestamping, logging, and synchronization
 *
 *   RDTS_TIME_QUALITY_HOLDOVER
 *     - No recent accepted beacon
 *     - Local time is extrapolated from last known discipline state
 *     - Error grows with RTC drift and elapsed time
 *
 * Notes:
 *   - HOLDOVER is entered automatically after a beacon silence timeout
 *   - Quality may move backward (e.g. LOCKED → HOLDOVER)
 *   - Quality does NOT imply monotonicity or absolute correctness
 */