#pragma once

#include <Arduino.h>

// ============================================================================
// LOGGER OTA INTERFACE
// ============================================================================
//
// This module provides ArduinoOTA-based wireless firmware updates.
//
// Responsibilities:
//   - Connecting to WiFi using credentials stored in reserved flash slots
//   - Initializing and servicing ArduinoOTA
//   - Coordinating OTA lifecycle with HTTP services
//
// Non-responsibilities:
//   - No CLI parsing or user interaction
//   - No flash layout ownership (credentials are managed by LoggerCore/CLI)
//   - No background scheduling (service must be called explicitly)
//
// Design notes:
//   - OTA is only intended to run while MODE_IDLE
//   - WiFi is enabled only while OTA is active
//   - OTA authentication uses a stored MD5 password hash
//   - BLE and Serial remain active during OTA
//
// Flash storage conventions (by index):
//   - slot[1]: WiFi SSID (ASCII, null-terminated)
//   - slot[2]: WiFi password (ASCII, null-terminated)
//   - slot[3]: OTA password hash (32-char ASCII MD5 hex)
//
// Failure handling:
//   - Missing credentials or connection failure disables OTA
//   - WiFi is shut down cleanly on failure or stop
//

// ============================================================================
// OTA STATE QUERIES
// ============================================================================

// Returns true if OTA has been successfully initialized and is active.
bool otaStarted();

// Returns true if OTA is active and WiFi is connected.
bool otaHasIP();

// Copy the local WiFi IP address to the provided buffer.
//
// Behavior:
//   - Writes an empty string if OTA is inactive or WiFi is disconnected
//   - Output is a dotted-quad ASCII string ("x.x.x.x")
void otaGetIP(char *out, size_t outLen);

// ============================================================================
// OTA CONTROL
// ============================================================================

// Start OTA service.
//
// Behavior:
//   - Loads WiFi credentials and OTA password hash from flash
//   - Connects to WiFi (blocking, with timeout)
//   - Initializes ArduinoOTA
//   - Starts HTTP services
//
// Safe to call multiple times.
// Intended to be called only when MODE_IDLE.
void startOTA();

// Stop OTA service.
//
// Behavior:
//   - Shuts down ArduinoOTA
//   - Disconnects WiFi and powers down radio
//   - Stops HTTP services
//
// Safe to call even if OTA was never started.
void stopOTA();

// ============================================================================
// OTA SERVICE
// ============================================================================

// Service OTA events.
//
// Behavior:
//   - Must be called periodically from loop()
//   - Does nothing if OTA is inactive
//
// NOTE:
//   - Should only be called while MODE_IDLE
//   - This function may block briefly while handling OTA traffic
void serviceOTA();