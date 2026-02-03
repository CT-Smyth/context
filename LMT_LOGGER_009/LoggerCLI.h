#pragma once

#include <Arduino.h>
#include "LoggerCore.h"

// ============================================================================
// CLI INTERFACE
// ============================================================================
//
// This header defines the command-line interface surface:
//  - User-facing output helpers (help, status, prompt)
//  - The command dispatcher
//
// All implementations live in LoggerCLI.cpp.
// No device logic or state ownership exists here.

// -------------------- Output Helpers --------------------

// Print help text to the given output stream
void printHelpTo(Stream &out);

// Print full device status (routes via emitControl)
void printStatus();

// Stream-specific status formatter
void printStatusTo(Stream &out);

// Print the interactive prompt to USB Serial
void printPrompt();

// -------------------- Command Dispatcher --------------------

// Parse and execute a single CLI command line
void handleCommand(const char *cmd);
