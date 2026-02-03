#pragma once

#include <Arduino.h>
#include "LoggerCore.h"

// ============================================================================
// LOGGER CLI INTERFACE
// ============================================================================
//
// This header defines the public command-line interface (CLI) surface.
//
// Responsibilities:
//   - User-facing command parsing and dispatch
//   - Human-readable status and help output
//   - Interactive prompt management
//
// Non-responsibilities:
//   - No device I/O ownership (IMU, flash, BLE, OTA, HTTP)
//   - No state ownership beyond formatting and dispatch
//   - No background tasks or polling
//
// All implementations live in LoggerCLI.cpp.
// This module is intentionally synchronous and stateless.
//
// Design notes:
//   - All output is routed via emitControl() so it mirrors to Serial and BLE
//   - Commands operate directly on LoggerCore global state
//   - This interface is safe to reuse across firmware variants
//

// ============================================================================
// OUTPUT HELPERS
// ============================================================================
//
// These functions format and emit user-visible output.
// They do not perform device I/O or change system state.

// Print help text to the given output stream.
// Intended for both Serial and BLE control-plane output.
void printHelpTo(Stream &out);

// Print full device status via emitControl().
// This mirrors output to Serial and BLE if connected.
void printStatus();

// Stream-specific status formatter.
// Used internally by printStatus() and emitControl().
void printStatusTo(Stream &out);

// Print the interactive CLI prompt to USB Serial.
// NOTE: Prompt output is intentionally not mirrored to BLE.
void printPrompt();

// ============================================================================
// COMMAND DISPATCH
// ============================================================================
//
// Parse and execute a single CLI command line.
//
// Behavior:
//   - Parsing is synchronous and non-blocking
//   - Commands may change global run mode or system state
//   - Unknown commands emit a diagnostic message
//
// This function assumes:
//   - Input is a null-terminated ASCII string
//   - Caller is responsible for input buffering and line editing
//
void handleCommand(const char *cmd);