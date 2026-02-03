#include <Arduino.h>
#include "RDTSprint.h"
#include "RDTSserver.h"
#include "RDTSrtc.h"
#include "RDTSble.h"
#include "RDTSpacket.h"
#include "RDTScrypto.h"
#include "RDTSconfig.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Persistent configuration                                                    */
/* -------------------------------------------------------------------------- */

static rdts_config_t g_cfg;

/* -------------------------------------------------------------------------- */
/* Runtime state (authoritative)                                               */
/* -------------------------------------------------------------------------- */

static uint64_t g_anchor_unix = 0;
static uint64_t g_anchor_ticks = 0;

static uint32_t g_beacon_period_ms;
static uint8_t g_burst_len;
static uint16_t g_burst_span_ms;
static int8_t g_tx_power_dbm;

static rdts_mode_t g_mode;

static rdts_addr_mode_t g_addr_mode = RDTS_ADDR_NONE;
static uint8_t g_addr_count = 0;
static uint32_t g_addr_list[RDTS_MAX_ADDR_IDS];
static uint8_t g_window_len = 0;

static bool g_beacon_running = false;
static uint64_t g_next_beacon_ms = 0;
static bool g_drop = false;
static uint8_t g_burst_remaining = 0;
static uint32_t g_burst_interval_ms = 0;
static uint32_t g_next_burst_adv_ms = 0;

static rdtsm_auth_mode_t g_auth_mode = RDTSM_AUTH_DEV;

/* -------------------------------------------------------------------------- */

static void dump_beacon(const rdts_beacon_t* b, size_t len);

/* -------------------------------------------------------------------------- */
/* Initialization / persistence                                                */
/* -------------------------------------------------------------------------- */

void rdtsm_init(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (!rdtscfg_load(&g_cfg)) {
    rdtscfg_set_defaults(&g_cfg);
    rdtscfg_save(&g_cfg);
  }

  g_beacon_period_ms = g_cfg.beacon_period_ms;
  g_burst_len = g_cfg.beacon_burst_len;
  g_burst_span_ms = g_cfg.beacon_burst_span_ms;
  g_tx_power_dbm = g_cfg.tx_power_dbm;
  g_auth_mode = (rdtsm_auth_mode_t)g_cfg.auth_mode;
  g_mode = (rdts_mode_t)g_cfg.default_mode;

  rdtscrypto_set_key(g_cfg.key, g_cfg.key_len);

  g_anchor_unix = 0;
  g_anchor_ticks = rdtsrtc_now_ms();
  g_beacon_running = false;
}

void rdtsm_save_config(void) {
  g_cfg.beacon_period_ms = g_beacon_period_ms;
  g_cfg.beacon_burst_len = g_burst_len;
  g_cfg.beacon_burst_span_ms = g_burst_span_ms;
  g_cfg.tx_power_dbm = g_tx_power_dbm;
  g_cfg.auth_mode = (uint8_t)g_auth_mode;
  g_cfg.default_mode = (uint8_t)g_mode;

  // NOTE:
  // g_cfg.key and g_cfg.key_len are already authoritative.
  // We do NOT read keys back from crypto.

  rdtscfg_save(&g_cfg);
}

void rdtsm_load_defaults(void) {
  rdtscfg_set_defaults(&g_cfg);
  rdtscfg_save(&g_cfg);

  g_beacon_period_ms = g_cfg.beacon_period_ms;
  g_burst_len = g_cfg.beacon_burst_len;
  g_burst_span_ms = g_cfg.beacon_burst_span_ms;
  g_tx_power_dbm = g_cfg.tx_power_dbm;
  g_auth_mode = (rdtsm_auth_mode_t)g_cfg.auth_mode;
  g_mode = (rdts_mode_t)g_cfg.default_mode;

  rdtscrypto_set_key(nullptr, 0);

  g_anchor_unix = 0;
  g_anchor_ticks = rdtsrtc_now_ms();
  g_beacon_running = false;
}


static void recompute_burst_timing(void) {
  if (g_burst_len == 0) g_burst_len = 1;

  g_burst_interval_ms = g_burst_span_ms / g_burst_len;
  if (g_burst_interval_ms < 20)
    g_burst_interval_ms = 20;
}

/* -------------------------------------------------------------------------- */
/* Time authority                                                              */
/* -------------------------------------------------------------------------- */

void rdtsm_set_time_anchor(uint64_t unix_ms) {
  g_anchor_unix = unix_ms;
  g_anchor_ticks = rdtsrtc_now_ms();
}

uint64_t rdtsm_now_unix_ms(void) {
  uint64_t now = rdtsrtc_now_ms();
  return g_anchor_unix + (now - g_anchor_ticks);
}

uint64_t rdtsm_uptime_ms(void) {
  return rdtsrtc_now_ms();
}

/* -------------------------------------------------------------------------- */
/* Security                                                                    */
/* -------------------------------------------------------------------------- */

void rdtsm_set_key(const uint8_t* key, uint8_t len) {
  rdtscrypto_set_key(key, len);
}

void rdtsm_set_auth_mode(rdtsm_auth_mode_t mode) {
  g_auth_mode = mode;
}

rdtsm_auth_mode_t rdtsm_get_auth_mode(void) {
  return g_auth_mode;
}

/* -------------------------------------------------------------------------- */
/* Beacon construction                                                         */
/* -------------------------------------------------------------------------- */

static void build_and_send_beacon(void) {
  rdts_beacon_t b;
  memset(&b, 0, sizeof(b));

  b.version = RDTS_PACKET_VERSION;
  b.addr_mode = g_addr_mode;
  b.addr_count = g_addr_count;
  b.window_len = g_window_len;
  b.mode = (uint8_t)g_mode;
  b.t_master_unix_ms = rdtsm_now_unix_ms();

  if (g_addr_mode == RDTS_ADDR_LIST) {
    memcpy(b.addr_list, g_addr_list, g_addr_count * sizeof(uint32_t));
  }

  uint8_t frame[64];
  size_t len;

  if (g_auth_mode == RDTSM_AUTH_PROD)
    len = rdtspkt_build(frame, sizeof(frame), &b);
  else
    len = rdtspkt_build_noauth(frame, sizeof(frame), &b);

  if (len == 0) return;

  dump_beacon(&b, len);
  rdtsble_advertise_once(frame, len);
  // Toggle activity LED on successful transmit
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

/* -------------------------------------------------------------------------- */
/* Scheduler                                                                   */
/* -------------------------------------------------------------------------- */

void rdtsm_service(void) {
  if (!g_beacon_running) return;

  uint32_t now = rdtsrtc_now_ms();

  // Start a new burst at each beacon period
  if (now >= g_next_beacon_ms) {
    g_next_beacon_ms = now + g_beacon_period_ms;

    if (!g_drop) {
      g_burst_remaining = g_burst_len;
      g_next_burst_adv_ms = now;
    }
  }

  // Emit burst advertisements (fresh payload each time)
  if (g_burst_remaining > 0 && now >= g_next_burst_adv_ms) {
    build_and_send_beacon();  // builds fresh timestamp

    g_burst_remaining--;
    g_next_burst_adv_ms += g_burst_interval_ms;
  }
}



void rdtsm_beacon_start(void) {
  uint32_t now = rdtsrtc_now_ms();

  g_beacon_running = true;
  g_next_beacon_ms = now;

  recompute_burst_timing();

  g_burst_remaining   = 0;   // armed on first scheduler tick
  g_next_burst_adv_ms = now;
}

void rdtsm_beacon_stop(void) {
  g_beacon_running = false;
}

/* -------------------------------------------------------------------------- */
/* Configuration setters / getters                                             */
/* -------------------------------------------------------------------------- */

void rdtsm_set_beacon_period(uint32_t ms) {
  g_beacon_period_ms = ms;
}

void rdtsm_set_beacon_burst_len(uint8_t n) {
  g_burst_len = n;
  recompute_burst_timing();
}

void rdtsm_set_beacon_burst_span(uint16_t ms) {
  g_burst_span_ms = ms;
  recompute_burst_timing();
}

uint32_t rdtsm_get_burst_interval_ms(void)
{
    return g_burst_interval_ms;
}

uint32_t rdtsm_get_beacon_period(void) {
  return g_beacon_period_ms;
}
uint8_t rdtsm_get_beacon_burst_len(void) {
  return g_burst_len;
}
uint16_t rdtsm_get_beacon_burst_span(void) {
  return g_burst_span_ms;
}

void rdtsm_set_beacon_tx_power(int8_t dbm) {
  g_tx_power_dbm = dbm;
}
int8_t rdtsm_get_beacon_tx_power(void) {
  return g_tx_power_dbm;
}

/* -------------------------------------------------------------------------- */
/* Modes and control windows                                                   */
/* -------------------------------------------------------------------------- */

void rdtsm_mode_set(rdts_mode_t mode) {
  g_mode = mode;
}
rdts_mode_t rdtsm_get_mode(void) {
  return g_mode;
}

void rdtsm_control_open_all(uint8_t window_len) {
  g_addr_mode = RDTS_ADDR_ALL;
  g_addr_count = 0;
  g_window_len = window_len;
}

void rdtsm_control_open_list(uint8_t window_len,
                             const uint32_t* ids,
                             uint8_t count) {
  if (count > RDTS_MAX_ADDR_IDS) count = RDTS_MAX_ADDR_IDS;
  g_addr_mode = RDTS_ADDR_LIST;
  g_addr_count = count;
  g_window_len = window_len;
  memcpy(g_addr_list, ids, count * sizeof(uint32_t));
}

void rdtsm_control_stop_asserting(void) {
  g_addr_mode = RDTS_ADDR_NONE;
  g_addr_count = 0;
  g_window_len = 0;
}

rdts_addr_mode_t rdtsm_get_addr_mode(void) {
  return g_addr_mode;
}
uint8_t rdtsm_get_window_len(void) {
  return g_window_len;
}
bool rdtsm_is_beacon_running(void) {
  return g_beacon_running;
}

/* -------------------------------------------------------------------------- */
/* Test hooks                                                                  */
/* -------------------------------------------------------------------------- */

void rdtsm_test_send_oneshot(void) {
  build_and_send_beacon();
}
void rdtsm_test_send_burst_oneshot(void) {
  build_and_send_beacon();
}
void rdtsm_test_set_slew_ppm(int32_t ppm) {
  rdtsrtc_set_slew_ppm(ppm);
}
void rdtsm_test_set_drop_mode(bool en) {
  g_drop = en;
}

/* -------------------------------------------------------------------------- */
/* Diagnostics                                                                 */
/* -------------------------------------------------------------------------- */

static void dump_beacon(const rdts_beacon_t* b, size_t len) {
  Serial.print("[BEACON] t=");
  rdts_print_u64(b->t_master_unix_ms);

  Serial.print(" mode=");
  Serial.print((int)b->mode);

  Serial.print(" addr=");
  if (b->addr_mode == RDTS_ADDR_NONE) Serial.print("NONE");
  else if (b->addr_mode == RDTS_ADDR_ALL) Serial.print("ALL");
  else Serial.print("LIST");

  if (b->addr_mode == RDTS_ADDR_LIST) {
    Serial.print(" ids=");
    for (uint8_t i = 0; i < b->addr_count; i++) {
      Serial.print(b->addr_list[i]);
      if (i + 1 < b->addr_count) Serial.print(",");
    }
  }

  Serial.print(" win=");
  Serial.print(b->window_len);
  Serial.print(" len=");
  Serial.println(len);
}