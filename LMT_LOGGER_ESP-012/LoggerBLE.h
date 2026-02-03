#pragma once

#include <Arduino.h>

// ============================================================================
// LOGGER BLE UART INTERFACE
// ============================================================================
//
// This module provides a Nordic-style BLE UART (NUS) transport used for:
//   - CLI command input
//   - Control-plane output mirroring (via emitControl / bleWrite)
//
// Responsibilities:
//   - BLE stack initialization and lifecycle control
//   - Advertising and connection management
//   - RX path: feeding received text into the CLI
//   - TX path: best-effort notification-based output
//
// Non-responsibilities:
//   - No device logic (IMU, flash, recording, playback)
//   - No state ownership beyond BLE connection state
//   - No buffering or retry guarantees for TX
//
// Design notes:
//   - BLE is optional and may be enabled/disabled at runtime
//   - BLE RX is only meaningful in MODE_IDLE
//   - The BLE stack is initialized once and never deinitialized
//   - Disabling BLE stops advertising and disconnects clients,
//     but leaves the stack resident
//

// ============================================================================
// BLE LIFECYCLE
// ============================================================================

// Enable BLE UART functionality.
//
// Behavior:
//   - Initializes the BLE stack on first call
//   - Creates NUS service and characteristics once
//   - Starts advertising and allows connections
//
// Safe to call multiple times.
void startBLEUart();

// Disable BLE UART functionality.
//
// Behavior:
//   - Stops advertising
//   - Disconnects any active client
//   - Leaves BLE stack initialized
//
// Safe to call even if BLE was never started.
void stopBLEUart();

// ============================================================================
// BLE STATE QUERIES
// ============================================================================

// Returns true if the BLE stack has been initialized.
bool bleStarted();

// Returns true if a GATT client is currently connected.
bool bleConnected();

// ============================================================================
// RX / TX HELPERS
// ============================================================================
//
// These helpers are safe to call regardless of BLE state.
// If BLE is inactive or no client is connected, they are no-ops.

// Poll BLE RX and feed received text into the CLI.
//
// NOTE:
//   - Intended to be called from the main loop
//   - Effective only when MODE_IDLE
//   - Line-oriented: commands are dispatched on newline
//
// (Currently implemented via BLE write callbacks.)
void serviceBLERx();

// Write raw bytes to the BLE UART TX characteristic.
//
// Behavior:
//   - Best-effort notification
//   - No internal buffering or retry
//   - Silently drops data if not connected
void bleWrite(const uint8_t *buf, size_t len);

// Write a newline-delimited ASCII string to BLE UART.
//
// Behavior:
//   - Splits on '\n' and sends line-by-line
//   - Adds small delays to avoid overrunning notifications
//   - Silently drops output if not connected
void blePrintln(const char *s);