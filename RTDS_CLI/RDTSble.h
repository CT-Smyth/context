#pragma once
#include <stdint.h>

// Initialize BLE adapter
void rdtsble_init(void);

void rdtsm_set_beacon_tx_power(int8_t dbm); 

// Request a timed advertising burst
void rdtsble_advertise_once(const uint8_t* data, uint16_t len);

// Called by BLE stack on RX (for future client use)
void rdtsble_on_rx(const uint8_t* data, uint16_t len);
// Cooperative service routine for BLE adapter
void rdtsble_service(void);