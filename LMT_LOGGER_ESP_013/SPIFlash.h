#pragma once

#include <Arduino.h>
#include <SPI.h>

// ESP32-only internal partition backend
#if defined(ARDUINO_ARCH_ESP32)
#include "esp_partition.h"
#endif

// =============================================================================
// SPI FLASH COMMAND SET (JEDEC / COMMON)
// =============================================================================

#define FLASH_CMD_RDID 0x9F  // Read JEDEC ID
#define FLASH_CMD_READ 0x03  // Read data (3-byte address)
#define FLASH_CMD_PP   0x02  // Page program
#define FLASH_CMD_SE   0x20  // Sector erase (4 KB)
#define FLASH_CMD_CE   0xC7  // Chip erase
#define FLASH_CMD_RDSR 0x05  // Read status register
#define FLASH_CMD_WREN 0x06  // Write enable
#define FLASH_CMD_DP   0xB9  // Deep power-down
#define FLASH_CMD_RDP  0xAB  // Release from power-down

// =============================================================================
// FLASH GEOMETRY
// =============================================================================
//
// This driver exposes a minimal "page + sector" abstraction:
//
// - PAGE = 256 bytes, used as the unit for writePage().
// - SECTOR = 4096 bytes, used as the unit for eraseSector().
//
// NOTE: External flash page programming does not require writes to be a full
// 256 bytes, but this project uses 256-byte pages as the canonical storage unit.
//

#define FLASH_PAGE_SIZE   256
#define FLASH_SECTOR_SIZE 4096

// =============================================================================
// SPI CONFIGURATION
// =============================================================================

#define FLASH_SPI_SPEED 4000000  // 4 MHz (safe default)

// =============================================================================
// INTERNAL FLASH EMULATION (ESP32)
// =============================================================================
//
// If no external JEDEC flash is detected on the CS pin, this driver can emulate
// the same API using an internal flash partition on ESP32.
//
// Requirements (ESP32 only):
// - A writable DATA partition exists with label FLASH_INTERNAL_PART_LABEL.
// - Emulated capacity is exposed as a power-of-two window <= partition size.
//   This preserves upstream code that assumes: capacityBytes = (1UL << cap).
//
// IMPORTANT:
// - In emulated mode, readData/writePage/eraseSector operate on offsets within
//   the emulation window [0, emulatedCapacityBytes()).
// - The partition is not automatically formatted; it is treated as raw bytes.
//
// IMPORTANT:
// The internal backend uses esp_partition_* APIs directly.
// The SPIFFS filesystem must NOT be mounted while this driver is active.


// Default label: "spiffs"
//
// NOTE:
// This driver does NOT use the SPIFFS filesystem.
// It treats the SPIFFS partition as raw flash storage.
#ifndef FLASH_INTERNAL_PART_LABEL
#define FLASH_INTERNAL_PART_LABEL "spiffs"
#endif

class SPIFlash {
public:
  explicit SPIFlash(uint8_t csPin);

  // -------------------------------------------------------------------------
  // Lifecycle
  // -------------------------------------------------------------------------
  //
  // begin():
  //  - Initializes the CS pin.
  //  - Detects external SPI flash via JEDEC RDID.
  //  - If not detected and running on ESP32, tries internal partition backend.
  //
  // NOTE (ESP32): This driver intentionally does NOT call SPI.begin().
  // The sketch must set up SPI pins via SPI.begin(SCK, MISO, MOSI).
  //
  bool begin();

  // Power management (external flash only; no-ops for emulated backend).
  void sleep();
  void wake();

  // -------------------------------------------------------------------------
  // Identification
  // -------------------------------------------------------------------------
  //
  // readID():
  //  - External: returns JEDEC RDID bytes (manufacturer, type, capacity).
  //  - Emulated: returns synthetic 0xEE/0xEE and a "cap" such that:
  //      (1UL << cap) == emulatedCapacityBytes()
  //
  bool readID(uint8_t &man, uint8_t &type, uint8_t &cap);

  // -------------------------------------------------------------------------
  // Erase operations
  // -------------------------------------------------------------------------

  // Erases the 4KB sector containing 'addr' (addr is not required to be aligned).
  bool eraseSector(uint32_t addr);

  // Erases the entire available flash (external) or emulation window (ESP32).
  bool chipErase();

  // -------------------------------------------------------------------------
  // Data access
  // -------------------------------------------------------------------------
  //
  // readData():
  //  - External: raw SPI READ (0x03) with 24-bit address.
  //  - Emulated (ESP32): reads from partition offset.
  //
  // writePage():
  //  - External: PAGE PROGRAM (0x02) with 24-bit address.
  //  - Emulated (ESP32): writes to partition offset (requires 4-byte alignment).
  //
  bool readData(uint32_t addr, uint8_t *buf, uint32_t len);
  bool writePage(uint32_t addr, const uint8_t *buf, uint16_t len);

  // -------------------------------------------------------------------------
  // Introspection
  // -------------------------------------------------------------------------

  bool isEmulated() const { return _emulated; }
  uint32_t emulatedCapacityBytes() const { return _emuCapacityBytes; }

private:
  // -------------------------------------------------------------------------
  // Hardware state
  // -------------------------------------------------------------------------
  uint8_t _cs;

  // -------------------------------------------------------------------------
  // Backend selection
  // -------------------------------------------------------------------------
  bool _emulated = false;

  // In emulated mode, expose a power-of-two capacity <= partition size so
  // upstream code computing (1UL<<cap) cannot overrun.
  uint32_t _emuCapacityBytes = 0;

#if defined(ARDUINO_ARCH_ESP32)
  const esp_partition_t *_part = nullptr;
#endif

  // -------------------------------------------------------------------------
  // Low-level helpers (external SPI)
  // -------------------------------------------------------------------------
  void csLow();
  void csHigh();

  void writeEnable();
  bool waitForReady(uint32_t timeoutMs = 3000);

  bool tryDetectExternalJedec(uint8_t &man, uint8_t &type, uint8_t &cap);

#if defined(ARDUINO_ARCH_ESP32)
  bool tryInitInternalPartition();
#endif

  // Utility helpers
  static uint8_t capCodeForBytesPow2(uint32_t bytesPow2);
  static uint32_t floorPow2(uint32_t x);
};