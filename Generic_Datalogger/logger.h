#pragma once
#include <Arduino.h>

// Initialize SPIFFS and logging system
bool InitLog();

// Erase the log file
bool ClearLog();

// Log a variable-length record.
// Usage: Log(count, x, y, z, ...)
void DataLog(uint8_t count, ...);

// Print entire log to Serial as:
// millis,x,y,z,...
void PlayLog();