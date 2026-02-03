#pragma once
#include <stdint.h>
#include <stdbool.h>

#define RDTS_CONFIG_MAGIC   0x52545453UL  // "RTTS"
#define RDTS_CONFIG_VERSION 1

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    // --- Time / scheduling ---
    uint32_t beacon_period_ms;
    uint16_t beacon_burst_span_ms;
    uint8_t  beacon_burst_len;
    int8_t   tx_power_dbm;

    // --- Security ---
    uint8_t  auth_mode;        // rdtsm_auth_mode_t
    uint8_t  key_len;          // 0, 16, 32
    uint8_t  key[32];

    // --- Control defaults ---
    uint8_t  default_mode;     // rdts_mode_t

    // Reserved for future expansion
    uint8_t  reserved[32];

} rdts_config_t;

// Load config from non-volatile storage.
// Returns true if valid config was loaded.
bool rdtscfg_load(rdts_config_t* out);

// Save config to non-volatile storage.
void rdtscfg_save(const rdts_config_t* cfg);

// Populate defaults (factory state).
void rdtscfg_set_defaults(rdts_config_t* cfg);