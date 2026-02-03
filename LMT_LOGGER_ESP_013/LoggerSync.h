#pragma once
#include <stdint.h>

#define SYNC_FRAMES_PER_PAGE 16   // 256 / 16
#define SYNC_INTERVAL_MS    60000 // 60 seconds

struct SyncFrame {
  uint64_t master_unix_ms;  // from beacon (mocked for now)
  uint32_t local_ms;        // millis() at sampling time
  int16_t  temp_c_x100;     // ESP32 internal temperature
  uint16_t crc16;           // CRC-16-CCITT
};

static_assert(sizeof(SyncFrame) == 16, "SyncFrame must be 16 bytes");