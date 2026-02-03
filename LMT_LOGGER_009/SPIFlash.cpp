#include "SPIFlash.h"

// =============================================================================
// CONSTRUCTION
// =============================================================================

SPIFlash::SPIFlash(uint8_t csPin)
  : _cs(csPin) {}

// =============================================================================
// CHIP SELECT HELPERS
// =============================================================================

inline void SPIFlash::csLow() {
  digitalWrite(_cs, LOW);
}

inline void SPIFlash::csHigh() {
  digitalWrite(_cs, HIGH);
}

// =============================================================================
// INITIALIZATION
// =============================================================================

bool SPIFlash::begin() {
  pinMode(_cs, OUTPUT);
  csHigh();

  SPI.begin();
  return true;
}

// =============================================================================
// WRITE ENABLE / STATUS
// =============================================================================

void SPIFlash::writeEnable() {
  csLow();
  SPI.beginTransaction(SPISettings(
    FLASH_SPI_SPEED,
    MSBFIRST,
    SPI_MODE0));
  SPI.transfer(FLASH_CMD_WREN);
  SPI.endTransaction();
  csHigh();
}

bool SPIFlash::waitForReady(uint32_t timeout) {
  const uint32_t start = millis();

  while (millis() - start < timeout) {

    csLow();
    SPI.beginTransaction(SPISettings(
      FLASH_SPI_SPEED,
      MSBFIRST,
      SPI_MODE0));

    SPI.transfer(FLASH_CMD_RDSR);
    uint8_t status = SPI.transfer(0);

    SPI.endTransaction();
    csHigh();

    // WIP bit cleared → ready
    if ((status & 0x01) == 0) {
      return true;
    }

    delay(1);
  }

  return false;
}

// =============================================================================
// IDENTIFICATION
// =============================================================================

bool SPIFlash::readID(uint8_t &man,
                      uint8_t &type,
                      uint8_t &cap) {

  csLow();
  SPI.beginTransaction(SPISettings(
    FLASH_SPI_SPEED,
    MSBFIRST,
    SPI_MODE0));

  SPI.transfer(FLASH_CMD_RDID);
  man = SPI.transfer(0);
  type = SPI.transfer(0);
  cap = SPI.transfer(0);

  SPI.endTransaction();
  csHigh();

  return true;
}

// =============================================================================
// ERASE OPERATIONS
// =============================================================================

bool SPIFlash::eraseSector(uint32_t addr) {

  writeEnable();

  csLow();
  SPI.beginTransaction(SPISettings(
    FLASH_SPI_SPEED,
    MSBFIRST,
    SPI_MODE0));

  SPI.transfer(FLASH_CMD_SE);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);

  SPI.endTransaction();
  csHigh();

  return waitForReady(2000);
}

bool SPIFlash::chipErase() {

  writeEnable();

  csLow();
  SPI.beginTransaction(SPISettings(
    FLASH_SPI_SPEED,
    MSBFIRST,
    SPI_MODE0));
  SPI.transfer(FLASH_CMD_CE);
  SPI.endTransaction();
  csHigh();

  // Typical: ~100 ms, Worst-case: tens of seconds
  return waitForReady(100000);
}

// =============================================================================
// DATA ACCESS
// =============================================================================

bool SPIFlash::readData(uint32_t addr,
                        uint8_t *buf,
                        uint32_t len) {

  csLow();
  SPI.beginTransaction(SPISettings(
    FLASH_SPI_SPEED,
    MSBFIRST,
    SPI_MODE0));

  SPI.transfer(FLASH_CMD_READ);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);

  while (len--) {
    *buf++ = SPI.transfer(0);
  }

  SPI.endTransaction();
  csHigh();

  return true;
}

bool SPIFlash::writePage(uint32_t addr,
                         const uint8_t *buf,
                         uint16_t len) {

  if (len > FLASH_PAGE_SIZE) {
    return false;
  }

  writeEnable();

  csLow();
  SPI.beginTransaction(SPISettings(
    FLASH_SPI_SPEED,
    MSBFIRST,
    SPI_MODE0));

  SPI.transfer(FLASH_CMD_PP);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);

  for (uint16_t i = 0; i < len; i++) {
    SPI.transfer(buf[i]);
  }

  SPI.endTransaction();
  csHigh();

  return waitForReady(10);
}

// =============================================================================
// POWER MANAGEMENT
// =============================================================================

void SPIFlash::sleep() {

  csLow();
  SPI.beginTransaction(SPISettings(
    FLASH_SPI_SPEED,
    MSBFIRST,
    SPI_MODE0));
  SPI.transfer(FLASH_CMD_DP);
  SPI.endTransaction();
  csHigh();
}

void SPIFlash::wake() {

  csLow();
  SPI.beginTransaction(SPISettings(
    FLASH_SPI_SPEED,
    MSBFIRST,
    SPI_MODE0));
  SPI.transfer(FLASH_CMD_RDP);
  SPI.endTransaction();
  csHigh();

  // Datasheet-mandated wake delay
  delay(1);
}
