#include <Arduino.h>
#include "ble.h"
#include "TimeDisciplined.h"
#include "RDTSReceiver.h"
#include "RtcClock.h"
#include "logger.h"
#include "ScanScheduler.h"



/*
 * ---------------------------------------------------------------------------
 * Application-level RDTS scanner test harness
 *
 * Responsibilities of this file:
 *  - Schedule periodic BLE scan windows
 *  - Consume raw RDTS payloads from ble.cpp
 *  - Decode RDTS packets and apply application-level policy checks
 *  - Emit diagnostics over Serial
 *  - Drive status LEDs for human-observable feedback
 * ---------------------------------------------------------------------------
 */

// ---------------- LED portability ----------------
//
// Many ESP32-C3 boards only define LED_BUILTIN.
// If LED / LED are missing, we map them to LED_BUILTIN.
// Active-low semantics are preserved if your hardware is wired that way.
// If your LEDs are active-high, flip LED_ACTIVE_LOW to 0.

/*
 * Time domains used in this file:
 *
 *  1) RTC / local clock
 *     - Today: millis()
 *     - Later: hardware RTC ticks
 *
 *  2) Beacon time
 *     - pkt.master_unix_ms
 *     - Authoritative network time
 *
 *  3) Disciplined local time
 *     - Estimated Unix time derived from RTC via slew + offset
 *
 * time_on_beacon() updates the mapping:
 *     unix_est = A * rtc_ms + B
 */

#define LED 8

#ifndef LED_ACTIVE_LOW
#define LED_ACTIVE_LOW 1
#endif

static inline void led_on(uint8_t pin) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, LOW);
#else
  digitalWrite(pin, HIGH);
#endif
}
static inline void led_off(uint8_t pin) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, HIGH);
#else
  digitalWrite(pin, LOW);
#endif
}

// Scan scheduling state
static bool scan_active_prev = false;
static bool scan_had_sync = false;  // true iff at least one beacon was ACCEPTED during this scan window

static uint32_t scan_miss_count = 0;


// After first accepted beacon, we schedule scans in *disciplined unix ms*,
// centered on N*SCAN_PERIOD_MS.
static bool sched_locked = false;
static uint64_t next_scan_start_unix = 0;  // disciplined unix ms

// RDTS time sanity tracking (application-level policy)
static bool rdts_time_valid = false;
static uint64_t rdts_last_unix_ms = 0;

// Scan / policy parameters
#define SCAN_PERIOD_MS 60000
#define SCAN_DURATION_MS 100
#define SCAN_WINDOW_OFFSET_MS 100
#define SCAN_MISS_THRESHOLD 10

#define RDTS_MAX_TIME_JUMP_MS (SCAN_PERIOD_MS + 5000ULL)
// This is independent of scan interval or missed beacons.
#define MAX_EST_VS_BEACON_ERR_MS 10000  // 10 seconds

// LED behavior parameters
#define LED_FLASH_DURATION_MS 50

static uint32_t led_until = 0;

static void dump_raw_packet(const rdts_raw_payload_t &raw) {
  Serial.print("[RDTS] RAW len=");
  Serial.print(raw.len);
  Serial.print(" data=");

  for (uint8_t i = 0; i < raw.len; i++) {
    if (raw.data[i] < 0x10) Serial.print('0');
    Serial.print(raw.data[i], HEX);
    Serial.print(' ');
  }
  Serial.print("\n\n");
}

static void dump_decoded_packet(const rdts_packet_t &pkt) {
  Serial.println("[RDTS] DECODED PACKET:");

  Serial.print("  version: ");
  Serial.println(pkt.version);

  Serial.print("  addr_mode: ");
  switch (pkt.addr_mode) {
    case RDTS_ADDR_NONE: Serial.println("NONE"); break;
    case RDTS_ADDR_ALL: Serial.println("ALL"); break;
    case RDTS_ADDR_LIST: Serial.println("LIST"); break;
    default: Serial.println("UNKNOWN"); break;
  }

  Serial.print("  addr_count: ");
  Serial.println(pkt.addr_count);

  Serial.print("  window_len: ");
  Serial.println(pkt.window_len);

  Serial.print("  mode: ");
  Serial.println(pkt.mode);

  Serial.print("  flags: 0x");
  Serial.println(pkt.flags, HEX);

  Serial.print("  master_unix_ms: ");
  Serial.println(pkt.master_unix_ms);

  // ---- Semantic interpretation ----
  Serial.println("  semantics:");

  if (pkt.flags & RDTS_FLAG_NOAUTH) {
    Serial.println("    auth: DISABLED (NOAUTH)");
  } else {
    Serial.println("    auth: ENABLED (MAC present)");
  }

  if (pkt.addr_mode == RDTS_ADDR_LIST && pkt.addr_count > 0) {
    Serial.println("    address list:");
    for (uint8_t i = 0; i < pkt.addr_count; i++) {
      Serial.print("      [");
      Serial.print(i);
      Serial.print("] 0x");
      Serial.println(pkt.addr_list[i], HEX);
    }
  }

  if (!(pkt.flags & RDTS_FLAG_NOAUTH)) {
    Serial.print("    MAC: ");
    for (uint8_t i = 0; i < RDTS_MAC_LEN; i++) {
      if (pkt.mac[i] < 0x10) Serial.print('0');
      Serial.print(pkt.mac[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }

  Serial.println();
}

void setup() {
  // LED setup
  pinMode(LED, OUTPUT);
  led_off(LED);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Beacon Reciever initializing...");

  // Pre-lock: scan windows run back-to-back until first accepted beacon.
  // Post-lock: next_scan_start_unix is computed on first accepted beacon.
  sched_locked = false;
  next_scan_start_unix = 0;

  // Initialize disciplined time model (beacon will lock it)
  time_init();
  rdts_receiver_init();

  // Initialize BLE stack (does not start scanning)
  ble_init();

  ScanSchedConfig sched_cfg = {
    .period_ms = SCAN_PERIOD_MS,
    .scan_duration_ms = SCAN_DURATION_MS,
    .initial_phase_offset_ms = SCAN_WINDOW_OFFSET_MS,
    .prelock_back_to_back = true,
  };

  scan_sched_init(&sched_cfg);

  InitLog();

  Serial.println("\nLOG FROM LAST RUN");
  PlayLog();
  Serial.print("END LOG...Log will be cleared in 5 seconds.");
  delay(1000);
  Serial.print("..1");
  delay(1000);
  Serial.print("..2");
  delay(1000);
  Serial.print("..3");
  delay(1000);
  Serial.print("..4");
  delay(1000);
  Serial.println("..5");
  delay(1000);
  ClearLog();
  Serial.println("Log Cleared\n\n\n");
}

void loop() {
  // Pump BLE scan timing / stop deadline
  ble_process();

  uint32_t now = rtc_now_ms();

  // -------------------------------------------------------------------------
  // Scan scheduling
  // -------------------------------------------------------------------------
  uint64_t unix_now = time_now_unix_ms(now);

  ScanAction act =
    scan_sched_poll(now, unix_now, ble_scan_active());

  if (act.kind == SCAN_ACTION_START) {
    bool ok = ble_scan_start(act.duration_ms);
    if (ok) {
      scan_had_sync = false;
      scan_sched_on_scan_started(now);

      Serial.print("\n[RDTS] Scanning...\n");

      if (unix_now != 0) {
        Serial.print("[TIME] disciplined_unix_ms = ");
        Serial.println((unsigned long long)unix_now);
      } else {
        Serial.println("[TIME] disciplined_unix_ms = UNINITIALIZED");
      }

      led_on(LED);
      led_until = now + LED_FLASH_DURATION_MS;
    }
  }

  // -------------------------------------------------------------------------
  // RDTS packet handling
  // -------------------------------------------------------------------------
  rdts_raw_payload_t raw;
  rdts_packet_t pkt;

  if (rdts_get_latest_raw(&raw)) {

    rdts_decode_result_t res =
      rdts_decode_packet(raw.data, raw.len, &pkt);

    if (res == RDTS_DECODE_OK) {

      // ---- Application-level time policy (monotonic only) ----
      if (rdts_time_valid) {

        // Beacon time must be monotonic
        if (pkt.master_unix_ms < rdts_last_unix_ms) {
          Serial.print("[RDTS] ERROR- beacon time went backwards: ");
          Serial.print((unsigned long long)pkt.master_unix_ms);
          Serial.print(" < ");
          Serial.println((unsigned long long)rdts_last_unix_ms);

          led_on(LED);
          led_until = now + LED_FLASH_DURATION_MS;

          dump_raw_packet(raw);
          return;
        }
      }


      Serial.print("RDTS unix_ms=");
      Serial.println((unsigned long long)pkt.master_unix_ms);
      dump_decoded_packet(pkt);

      // ================  Disciplined time update + report

      RDTSRxResult rx =
        rdts_receiver_on_packet(pkt, raw.rx_ms);

      if (rx.result != RDTS_RX_ACCEPTED) {

        Serial.print("[RDTS] Beacon rejected: ");

        switch (rx.result) {
          case RDTS_RX_REJECTED_TIME_BACKWARDS:
            Serial.println("time went backwards");
            break;

          case RDTS_RX_REJECTED_ESTIMATE_ERROR:
            Serial.print("estimate error = ");
            Serial.print((long long)rx.est_error_ms);
            Serial.println(" ms");
            break;

          default:
            Serial.println("policy");
            break;
        }

        led_on(LED);
        led_until = now + LED_FLASH_DURATION_MS;
        dump_raw_packet(raw);
        return;
      }

      // ---------------- Accepted beacon ----------------

// Stop scan immediately after first accepted beacon (power saving)
scan_had_sync = true;
scan_miss_count = 0;   // reset consecutive-miss counter on success
ble_scan_stop();

      const TimeBeaconReport &tr = rx.time_report;
      scan_sched_on_beacon_accepted(tr.beacon_unix_ms);

      // Lock scan schedule phase on first accepted beacon:
      // next scan starts at (next 60s boundary - HALF_WINDOW).
      if (!sched_locked) {
        const uint32_t HALF_WINDOW_MS = (uint32_t)(SCAN_DURATION_MS / 2);

        // Use the beacon time itself as the phase reference.
        uint64_t t = tr.beacon_unix_ms;

        uint64_t next_boundary =
          ((t / (uint64_t)SCAN_PERIOD_MS) + 1ULL) * (uint64_t)SCAN_PERIOD_MS;

        next_scan_start_unix = (next_boundary - (uint64_t)HALF_WINDOW_MS) + (uint64_t)SCAN_WINDOW_OFFSET_MS;
        sched_locked = true;
      }

      Serial.println("[TIME] Beacon discipline report:");

      Serial.print("  rx_rtc_ms:              ");
      Serial.println((unsigned long)tr.rtc_rx_ms);

      Serial.print("  beacon_unix_ms:         ");
      Serial.println((unsigned long long)tr.beacon_unix_ms);

      Serial.print("  local_unix_raw_ms:      ");
      Serial.println((unsigned long long)tr.raw_unix_ms);

      Serial.print("  local_unix_est_pre_ms:  ");
      Serial.println((unsigned long long)tr.local_unix_ms_pre);

      Serial.print("  local_unix_est_post_ms: ");
      Serial.println((unsigned long long)tr.local_unix_ms_post);

      Serial.print("  err_rtc_minus_beacon_ms:");
      Serial.println((long long)tr.delta_rtc_vs_beacon_ms);

      Serial.print("  err_est_minus_beacon_ms:");
      Serial.println((long long)tr.delta_real_vs_beacon_ms);


      Serial.print("  rtc_slew_ppm:           ");
      Serial.println(tr.A_ppm, 3);

      Serial.print("  rtc_offset_ms:          ");
      Serial.println((long long)tr.B_ms);

      Serial.println();

      Serial.print("  time_quality:           ");
      switch (rdts_receiver_time_quality()) {
        case RDTS_TIME_QUALITY_INVALID: Serial.println("INVALID"); break;
        case RDTS_TIME_QUALITY_LOCKING: Serial.println("LOCKING"); break;
        case RDTS_TIME_QUALITY_LOCKED: Serial.println("LOCKED"); break;
        case RDTS_TIME_QUALITY_HOLDOVER: Serial.println("HOLDOVER"); break;
        default: Serial.println("UNKNOWN"); break;
      }


      DataLog(2, (double)tr.delta_real_vs_beacon_ms, (double)tr.delta_rtc_vs_beacon_ms);


      led_on(LED);
      led_until = now + LED_FLASH_DURATION_MS;

      dump_raw_packet(raw);
    }
  }



  // RDTS payload structurally rejected at BLE layer
  if (rdts_packet_rejected()) {
    Serial.println("[RDTS] ERROR- packet rejected at BLE layer (bad payload)");
  }

  // -------------------------------------------------------------------------
  // Scan end handling
  // -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// Scan end handling
// -------------------------------------------------------------------------
bool scan_active_now = ble_scan_active();
if (scan_active_prev && !scan_active_now) {
  scan_sched_on_scan_finished(scan_had_sync);

  if (!scan_had_sync) {
    Serial.println("[RDTS] no ACCEPTED beacon this scan");
    scan_miss_count++;

    if (scan_miss_count >= SCAN_MISS_THRESHOLD) {
      Serial.println("[RDTS] miss threshold reached -> forcing prelock + reacquire");

      scan_miss_count = 0;

      // Force aggressive prelock scanning (as-if reset)
      scan_sched_force_prelock();

      // Arm one-shot accept + re-anchor on next decoded beacon
      // Preserve learned frequency / holdover quality
      rdts_receiver_begin_reacquire(true);
    }
  }
}
scan_active_prev = scan_active_now;

  // -------------------------------------------------------------------------
  // LED auto-off timing
  // -------------------------------------------------------------------------
  if (led_until && now >= led_until) {
    led_off(LED);
    led_until = 0;
  }

  delay(1);
}