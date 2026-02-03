#pragma once

#include <Arduino.h>
#include <SPI.h>

// =============================================================================
// SPI FLASH COMMAND SET (JEDEC / COMMON)
// =============================================================================

#define FLASH_CMD_RDID 0x9F  // Read JEDEC ID
#define FLASH_CMD_READ 0x03  // Read data
#define FLASH_CMD_PP 0x02    // Page program
#define FLASH_CMD_SE 0x20    // Sector erase (4 KB)
#define FLASH_CMD_CE 0xC7    // Chip erase
#define FLASH_CMD_RDSR 0x05  // Read status register
#define FLASH_CMD_WREN 0x06  // Write enable
#define FLASH_CMD_DP 0xB9    // Deep power-down
#define FLASH_CMD_RDP 0xAB   // Release from power-down

// =============================================================================
// FLASH GEOMETRY
// =============================================================================

#define FLASH_PAGE_SIZE 256     // Smallest programmable unit
#define FLASH_SECTOR_SIZE 4096  // Smallest erasable unit

// =============================================================================
// SPI CONFIGURATION
// =============================================================================

#define FLASH_SPI_SPEED 4000000  // 4 MHz (safe default)

// =============================================================================
// SPIFlash CLASS
// =============================================================================
//
// Minimal, synchronous SPI NOR flash driver.
//
// Design constraints:
//  - No background operations
//  - No DMA assumptions
//  - No dynamic allocation
//  - Page-aligned writes expected by caller
//
// This driver assumes a JEDEC-compliant SPI NOR flash.
//

class SPIFlash {
public:
  explicit SPIFlash(uint8_t csPin);

  // -------------------------------------------------------------------------
  // Lifecycle
  // -------------------------------------------------------------------------

  bool begin();
  void sleep();
  void wake();

  // -------------------------------------------------------------------------
  // Identification
  // -------------------------------------------------------------------------

  bool readID(uint8_t &man, uint8_t &type, uint8_t &cap);

  // -------------------------------------------------------------------------
  // Erase operations
  // -------------------------------------------------------------------------

  bool eraseSector(uint32_t addr);
  bool chipErase();

  // -------------------------------------------------------------------------
  // Data access
  // -------------------------------------------------------------------------

  bool readData(uint32_t addr, uint8_t *buf, uint32_t len);
  bool writePage(uint32_t addr, const uint8_t *buf, uint16_t len);

private:
  // -------------------------------------------------------------------------
  // Hardware state
  // -------------------------------------------------------------------------

  uint8_t _cs;

  // -------------------------------------------------------------------------
  // Low-level helpers
  // -------------------------------------------------------------------------

  void csLow();
  void csHigh();

  void writeEnable();
  bool waitForReady(uint32_t timeout = 3000);
};
