#pragma once
#include <stdint.h>
#include <stdbool.h>

// One-time init (SoftDevice + scanner config)
void ble_init(void);

// Scanner control
void ble_rx_start(void);
void ble_rx_stop(void);

// State query
bool ble_rx_active(void);