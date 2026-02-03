#pragma once
#include <stdint.h>

// Returns last received beacon time in Unix ms.
// Mock implementation returns 0 for now.
uint64_t getLastBeaconTimeMs();

// Returns true if a beacon has *ever* been received.
// Mock returns false.
bool beaconValid();