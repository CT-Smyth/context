#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "RDTS_packet.h"
#include "rdts_decode.h"

// Initialize BLE stack (call once)
void ble_init(void);

// Pump BLE events (call from loop)
void ble_process(void);

// Start a single scan for duration_ms
// Returns false if a scan is already running
bool ble_scan_start(uint32_t duration_ms);

// Query scan state
bool ble_scan_active(void);

// Stop an active scan immediately (power-save)
void ble_scan_stop(void);

// RDTS getter API
bool rdts_packet_available(void);
bool rdts_packet_rejected(void);

/*
 * Raw RDTS payload (after manufacturer ID)
 */
#define RDTS_RAW_MAX_LEN  64

typedef struct {
    uint8_t  data[RDTS_RAW_MAX_LEN];
    uint8_t  len;
    uint32_t rx_ms;
} rdts_raw_payload_t;

/*
 * Retrieve latest raw RDTS payload.
 * Consume-on-read.
 */
bool rdts_get_latest_raw(rdts_raw_payload_t *out);