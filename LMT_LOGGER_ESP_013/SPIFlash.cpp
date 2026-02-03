#include "SPIFlash.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "esp_partition.h"
#include "esp_err.h"
#endif

// =============================================================================
// CONSTRUCTION
// =============================================================================

SPIFlash::SPIFlash(uint8_t csPin)
  : _cs(csPin) {}

// =============================================================================
// CHIP SELECT HELPERS
// =============================================================================

void SPIFlash::csLow() {
  digitalWrite(_cs, LOW);
}

void SPIFlash::csHigh() {
  digitalWrite(_cs, HIGH);
}

// =============================================================================
// SMALL UTILS
// =============================================================================

uint32_t SPIFlash::floorPow2(uint32_t x) {
  if (x == 0) return 0;

  // Highest power-of-two <= x
  uint32_t p = 1;
  while ((p << 1) && ((p << 1) <= x)) {
    p <<= 1;
  }
  return p;
}

// JEDEC "capacity" byte is commonly log2(bytes) for many flashes (e.g. 0x18 -> 16MB).
// Upstream code assumes: flashCapacityBytes = (1UL << cap);
// so cap must be log2(bytes) for our emulated backend to behave.
uint8_t SPIFlash::capCodeForBytesPow2(uint32_t bytesPow2) {
  uint8_t c = 0;
  while ((1UL << c) && ((1UL << c) < bytesPow2)) {
    c++;
  }
  return c;
}

// =============================================================================
// EXTERNAL SPI: WRITE ENABLE / STATUS
// =============================================================================

void SPIFlash::writeEnable() {
  csLow();
  SPI.beginTransaction(SPISettings(FLASH_SPI_SPEED, MSBFIRST, SPI_MODE0));
  SPI.transfer(FLASH_CMD_WREN);
  SPI.endTransaction();
  csHigh();
}

bool SPIFlash::waitForReady(uint32_t timeoutMs) {
  const uint32_t start = millis();

  while ((millis() - start) < timeoutMs) {

    csLow();
    SPI.beginTransaction(SPISettings(FLASH_SPI_SPEED, MSBFIRST, SPI_MODE0));

    SPI.transfer(FLASH_CMD_RDSR);
    const uint8_t status = SPI.transfer(0);

    SPI.endTransaction();
    csHigh();

    // WIP bit cleared -> ready
    if ((status & 0x01) == 0) {
      return true;
    }

    delay(1);
  }

  return false;
}

// =============================================================================
// DETECTION
// =============================================================================

bool SPIFlash::tryDetectExternalJedec(uint8_t &man, uint8_t &type, uint8_t &cap) {
  // Minimal JEDEC read; treat 0x00/0xFF as "not present / not wired"
  csLow();
  SPI.beginTransaction(SPISettings(FLASH_SPI_SPEED, MSBFIRST, SPI_MODE0));

  SPI.transfer(FLASH_CMD_RDID);
  man = SPI.transfer(0);
  type = SPI.transfer(0);
  cap = SPI.transfer(0);

  SPI.endTransaction();
  csHigh();

  if (man == 0x00 || man == 0xFF) return false;
  if (cap == 0x00 || cap == 0xFF) return false;

  return true;
}

#if defined(ARDUINO_ARCH_ESP32)
bool SPIFlash::tryInitInternalPartition() {
  // Find a writable partition by label
  _part = esp_partition_find_first(
    ESP_PARTITION_TYPE_DATA,
    ESP_PARTITION_SUBTYPE_ANY,
    FLASH_INTERNAL_PART_LABEL);

  if (!_part) {
    return false;
  }

  // Expose a safe power-of-two capacity <= partition size.
  // Guard against tiny partitions: require at least 64KB.
  const uint32_t sz = (uint32_t)_part->size;
  const uint32_t p2 = floorPow2(sz);

  if (p2 < (64UL * 1024UL)) {
    _part = nullptr;
    return false;
  }

  _emuCapacityBytes = p2;
  _emulated = true;
  return true;
}
#endif

// =============================================================================
// INITIALIZATION
// =============================================================================

bool SPIFlash::begin() {
  pinMode(_cs, OUTPUT);
  csHigh();

  // IMPORTANT (ESP32): Do NOT call SPI.begin() here.
  // The sketch configures SPI pins via SPI.begin(SCK, MISO, MOSI).
  // Calling SPI.begin() with no args can reset pin mapping.

  // Try external JEDEC
  uint8_t man = 0, type = 0, cap = 0;
  if (tryDetectExternalJedec(man, type, cap)) {
    _emulated = false;
    _emuCapacityBytes = 0;
#if defined(ARDUINO_ARCH_ESP32)
    _part = nullptr;
#endif
    return true;
  }

  // External not detected -> try internal partition backend (ESP32 only)
#if defined(ARDUINO_ARCH_ESP32)
  if (tryInitInternalPartition()) {
    return true;
  }
#endif

  // No backend available
  return false;
}

// =============================================================================
// IDENTIFICATION
// =============================================================================

bool SPIFlash::readID(uint8_t &man, uint8_t &type, uint8_t &cap) {

  if (_emulated) {
    // Synthetic, recognizable "emulated" ID.
    // cap must match upstream capacity computation: (1UL << cap).
    man = 0xEE;
    type = 0xEE;
    cap = capCodeForBytesPow2(_emuCapacityBytes);
    return true;
  }

  // External SPI JEDEC
  csLow();
  SPI.beginTransaction(SPISettings(FLASH_SPI_SPEED, MSBFIRST, SPI_MODE0));

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

  if (_emulated) {
#if defined(ARDUINO_ARCH_ESP32)
    if (!_part) return false;

    // Sector-aligned erase within emulation window
    const uint32_t a = addr & ~(uint32_t)(FLASH_SECTOR_SIZE - 1);
    if ((a + FLASH_SECTOR_SIZE) > _emuCapacityBytes) return false;

    const esp_err_t err = esp_partition_erase_range(_part, a, FLASH_SECTOR_SIZE);
    return (err == ESP_OK);
#else
    return false;
#endif
  }

  // External
  writeEnable();

  csLow();
  SPI.beginTransaction(SPISettings(FLASH_SPI_SPEED, MSBFIRST, SPI_MODE0));

  SPI.transfer(FLASH_CMD_SE);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);

  SPI.endTransaction();
  csHigh();

  return waitForReady(2000);
}

bool SPIFlash::chipErase() {

  if (_emulated) {
#if defined(ARDUINO_ARCH_ESP32)
    if (!_part) return false;

    // Erase the exposed emulation window (power-of-two subset of partition)
    uint32_t len = _emuCapacityBytes;

    // Must be multiple of sector size
    len &= ~(uint32_t)(FLASH_SECTOR_SIZE - 1);
    if (len < FLASH_SECTOR_SIZE) return false;

    for (uint32_t off = 0; off < len; off += FLASH_SECTOR_SIZE) {
      const esp_err_t err = esp_partition_erase_range(_part, off, FLASH_SECTOR_SIZE);
      if (err != ESP_OK) return false;
    }
    return true;
#else
    return false;
#endif
  }

  // External
  writeEnable();

  csLow();
  SPI.beginTransaction(SPISettings(FLASH_SPI_SPEED, MSBFIRST, SPI_MODE0));
  SPI.transfer(FLASH_CMD_CE);
  SPI.endTransaction();
  csHigh();

  // Typical: ~100 ms, worst-case: tens of seconds
  return waitForReady(100000);
}

// =============================================================================
// DATA ACCESS
// =============================================================================

bool SPIFlash::readData(uint32_t addr, uint8_t *buf, uint32_t len) {
  if (!buf) return false;

  if (_emulated) {
#if defined(ARDUINO_ARCH_ESP32)
    if (!_part) return false;
    if ((addr + len) > _emuCapacityBytes) return false;

    const esp_err_t err = esp_partition_read(_part, addr, buf, len);
    return (err == ESP_OK);
#else
    return false;
#endif
  }

  // External
  csLow();
  SPI.beginTransaction(SPISettings(FLASH_SPI_SPEED, MSBFIRST, SPI_MODE0));

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

bool SPIFlash::writePage(uint32_t addr, const uint8_t *buf, uint16_t len) {
  if (!buf) return false;
  if (len > FLASH_PAGE_SIZE) return false;

  if (_emulated) {
#if defined(ARDUINO_ARCH_ESP32)
    if (!_part) return false;
    if ((addr + len) > _emuCapacityBytes) return false;

    // Conservative: internal partition writes require 4-byte alignment.
    if ((addr & 0x3) != 0) return false;
    if ((len & 0x3) != 0) return false;

    const esp_err_t err = esp_partition_write(_part, addr, buf, len);
    return (err == ESP_OK);
#else
    return false;
#endif
  }

  // External
  writeEnable();

  csLow();
  SPI.beginTransaction(SPISettings(FLASH_SPI_SPEED, MSBFIRST, SPI_MODE0));

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
  if (_emulated) {
    // No-op for internal partition backend
    return;
  }

  csLow();
  SPI.beginTransaction(SPISettings(FLASH_SPI_SPEED, MSBFIRST, SPI_MODE0));
  SPI.transfer(FLASH_CMD_DP);
  SPI.endTransaction();
  csHigh();
}

void SPIFlash::wake() {
  if (_emulated) {
    // No-op for internal partition backend
    return;
  }

  csLow();
  SPI.beginTransaction(SPISettings(FLASH_SPI_SPEED, MSBFIRST, SPI_MODE0));
  SPI.transfer(FLASH_CMD_RDP);
  SPI.endTransaction();
  csHigh();

  // Datasheet-mandated wake delay
  delay(1);
}