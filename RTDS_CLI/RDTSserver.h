#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "RDTSpacket.h"

/*
    RDTS Master — Blind Broadcast Time Authority API
*/

/* ------------------------------------------------------------------
 * Authentication mode
 * ------------------------------------------------------------------ */
typedef enum {
    RDTSM_AUTH_DEV  = 0,   // MAC bypass (development only)
    RDTSM_AUTH_PROD = 1    // MAC enforced
} rdtsm_auth_mode_t;

/* ------------------------------------------------------------------
 * Initialization / config management / reset
 * ------------------------------------------------------------------ */
void rdtsm_init(void);
void rdtsm_factory_reset(void);
void rdtsm_save_config(void);
void rdtsm_load_defaults(void);

/* ------------------------------------------------------------------
 * Time authority
 * ------------------------------------------------------------------ */
void     rdtsm_set_time_anchor(uint64_t unix_ms);
uint64_t rdtsm_now_unix_ms(void);
uint64_t rdtsm_uptime_ms(void);

/* ------------------------------------------------------------------
 * Security
 * ------------------------------------------------------------------ */
void rdtsm_set_key(const uint8_t* key, uint8_t len);

void rdtsm_set_auth_mode(rdtsm_auth_mode_t mode);
rdtsm_auth_mode_t rdtsm_get_auth_mode(void);

/* ------------------------------------------------------------------
 * Beacon scheduler / RF parameters
 * ------------------------------------------------------------------ */
void     rdtsm_beacon_start(void);
void     rdtsm_beacon_stop(void);

void     rdtsm_set_beacon_period(uint32_t ms);
uint32_t rdtsm_get_beacon_period(void);

void     rdtsm_set_beacon_burst_len(uint8_t n);
uint8_t  rdtsm_get_beacon_burst_len(void);

void     rdtsm_set_beacon_burst_span(uint16_t ms);
uint16_t rdtsm_get_beacon_burst_span(void);

void     rdtsm_set_beacon_tx_power(int8_t dbm);
int8_t   rdtsm_get_beacon_tx_power(void);

bool     rdtsm_is_beacon_running(void);

uint32_t rdtsm_get_burst_interval_ms(void);

/* ------------------------------------------------------------------
 * Global operating mode
 * ------------------------------------------------------------------ */
void        rdtsm_mode_set(rdts_mode_t mode);
rdts_mode_t rdtsm_get_mode(void);

/* ------------------------------------------------------------------
 * Addressed control windows
 * ------------------------------------------------------------------ */
void rdtsm_control_open_all(uint8_t window_len);
void rdtsm_control_open_list(uint8_t window_len,
                             const uint32_t* ids,
                             uint8_t count);
void rdtsm_control_stop_asserting(void);

rdts_addr_mode_t rdtsm_get_addr_mode(void);
uint8_t          rdtsm_get_window_len(void);

/* ------------------------------------------------------------------
 * Test / validation hooks
 * ------------------------------------------------------------------ */
void rdtsm_test_send_oneshot(void);
void rdtsm_test_send_burst_oneshot(void);
void rdtsm_test_set_slew_ppm(int32_t ppm);
void rdtsm_test_set_drop_mode(bool en);

/* ------------------------------------------------------------------
 * Cooperative service
 * ------------------------------------------------------------------ */
void rdtsm_service(void);