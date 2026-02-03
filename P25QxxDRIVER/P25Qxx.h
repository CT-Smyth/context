/* P25Qxx.h - SPI NOR driver for Puya P25Q* (P25Q128L tested)
   Author: ChatGPT (adapted for Arduino)
   License: MIT-style (use/adapt freely)
*/

#ifndef P25QXX_H
#define P25QXX_H

#include <Arduino.h>
#include <SPI.h>

class P25Qxx {
public:
  // Erase unit selection
  enum EraseType {
    ERASE_4K,
    ERASE_64K
  };

  struct JedecID {
    uint8_t manufacturer;
    uint8_t type;
    uint8_t capacity;
  };

  // Constructor: csPin is the CS/SS pin number (use board constant like D2 if defined)
  P25Qxx(uint8_t csPin, SPISettings settings = SPISettings(8000000, MSBFIRST, SPI_MODE0)) :
    _cs(csPin), _spiSettings(settings), _eraseType(ERASE_64K) {
  }

  // Initialize SPI and CS pin. If you need custom MOSI/MISO/SCK, call SPI.setPins(...) before begin()
  void begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    SPI.begin();
  }

  // Optionally allow changing SPI pins before calling begin()
  void begin(uint8_t miso, uint8_t sck, uint8_t mosi) {
    SPI.setPins(miso, sck, mosi);
    begin();
  }

  // Read simple JEDEC ID (0x9F)
  JedecID readJEDEC() {
    JedecID id{0,0,0};
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0x9F);
    id.manufacturer = SPI.transfer(0x00);
    id.type         = SPI.transfer(0x00);
    id.capacity     = SPI.transfer(0x00);
    SPI.endTransaction();
    _deselect();
    return id;
  }

  // Read SFDP (returns up to "len" bytes into buffer; user provides buffer)
  // addr is 24-bit SFDP address (usually 0)
  bool readSFDP(uint32_t addr, uint8_t *buf, size_t len) {
    if (!buf || len == 0) return false;
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0x5A); // READ SFDP
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >> 8) & 0xFF);
    SPI.transfer(addr & 0xFF);
    SPI.transfer(0x00); // 8 dummy cycles for SFDP (common)
    for (size_t i = 0; i < len; ++i) buf[i] = SPI.transfer(0x00);
    SPI.endTransaction();
    _deselect();
    return true;
  }

  // Read data (standard read 0x03) into buffer (up to size)
  bool readData(uint32_t address, uint8_t *buf, size_t len) {
    if (!buf || len == 0) return false;
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0x03);
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8) & 0xFF);
    SPI.transfer(address & 0xFF);
    for (size_t i=0;i<len;i++) buf[i] = SPI.transfer(0x00);
    SPI.endTransaction();
    _deselect();
    return true;
  }

  // Fast read (0x0B) using a dummy byte (faster clocks)
  bool fastRead(uint32_t address, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0x0B);
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8) & 0xFF);
    SPI.transfer(address & 0xFF);
    SPI.transfer(0x00); // dummy
    for (size_t i=0;i<len;i++) buf[i] = SPI.transfer(0x00);
    SPI.endTransaction();
    _deselect();
    return true;
  }

  // Page program (up to 256 bytes). addr must be within page boundaries as typical for page program.
  // This does NOT split across pages automatically.
  bool pageProgram(uint32_t address, const uint8_t* data, size_t len, uint32_t timeoutMs = 3000) {
    if (!data || len == 0 || len > 256) return false;
    writeEnable();
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0x02); // PAGE PROGRAM
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8) & 0xFF);
    SPI.transfer(address & 0xFF);
    for (size_t i=0;i<len;i++) SPI.transfer(data[i]);
    SPI.endTransaction();
    _deselect();
    return waitForReady(timeoutMs);
  }

  // Write a buffer of arbitrary length — will split into pageProgram calls automatically
  bool writeBuffer(uint32_t addr, const uint8_t* buf, size_t len, uint32_t timeoutMsPerPage = 3000) {
    if (!buf || len == 0) return false;
    size_t remaining = len;
    uint32_t a = addr;
    const uint8_t* p = buf;

    while (remaining > 0) {
      uint16_t pageOffset = a & 0xFF;
      uint16_t toWrite = min<size_t>(256 - pageOffset, remaining);
      if (!pageProgram(a, p, toWrite, timeoutMsPerPage)) return false;
      a += toWrite;
      p += toWrite;
      remaining -= toWrite;
    }
    return true;
  }

  // Erase functions
  bool eraseSector4K(uint32_t address, uint32_t timeoutMs = 4000) {
    writeEnable();
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0x20); // 4K SECTOR ERASE
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8) & 0xFF);
    SPI.transfer(address & 0xFF);
    SPI.endTransaction();
    _deselect();
    return waitForReady(timeoutMs);
  }

  bool eraseBlock64K(uint32_t address, uint32_t timeoutMs = 10000) {
    writeEnable();
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0xD8); // 64K BLOCK ERASE (also 0x52 sometimes supported)
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8) & 0xFF);
    SPI.transfer(address & 0xFF);
    SPI.endTransaction();
    _deselect();
    return waitForReady(timeoutMs);
  }

  // Chip erase (C7 or 60)
  bool chipErase(uint32_t timeoutMs = 60000) {
    writeEnable();
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0xC7); // CHIP ERASE (also 0x60 is common)
    SPI.endTransaction();
    _deselect();
    return waitForReady(timeoutMs);
  }

  // Choose which erase type the high-level erase() uses
  void setEraseType(EraseType et) {
    _eraseType = et;
  }

  // Erase at address using preferred erase type (or falls back)
  bool erase(uint32_t address) {
    if (_eraseType == ERASE_64K) {
      return eraseBlock64K(address);
    } else {
      return eraseSector4K(address);
    }
  }

  // Read status register (RDSR 0x05)
  uint8_t readStatus() {
    uint8_t r;
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0x05);
    r = SPI.transfer(0x00);
    SPI.endTransaction();
    _deselect();
    return r;
  }

  // Write enable/disable
  void writeEnable() {
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0x06); // WREN
    SPI.endTransaction();
    _deselect();
  }

  void writeDisable() {
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0x04); // WRDI
    SPI.endTransaction();
    _deselect();
  }

  // Wait until WIP==0 or timeout
  bool waitForReady(uint32_t timeoutMs = 5000) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
      uint8_t st = readStatus();
      if ((st & 0x01) == 0) return true; // WIP cleared
      delay(1);
    }
    return false; // timed out
  }

  // Deep power down and wake
  void deepPowerDown() {
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0xB9); // DEEP POWER DOWN (common)
    SPI.endTransaction();
    _deselect();
    delay(1);
  }

  void releasePowerDown() {
    // Many chips allow 0xAB to release; alternatively sending 0xAB+3addr+dummy then read ID. Use 0xAB here.
    _select();
    SPI.beginTransaction(_spiSettings);
    SPI.transfer(0xAB);
    SPI.endTransaction();
    _deselect();
    delay(1);
  }

  // Convenience: read manufacturer+type+capacity and return readable string
  String identifyString() {
    JedecID id = readJEDEC();
    char buf[64];
    snprintf(buf, sizeof(buf), "MFG 0x%02X TYPE 0x%02X CAP 0x%02X", id.manufacturer, id.type, id.capacity);
    return String(buf);
  }

private:
  uint8_t _cs;
  SPISettings _spiSettings;
  EraseType _eraseType;

  void _select() {
    digitalWrite(_cs, LOW);
  }
  void _deselect() {
    digitalWrite(_cs, HIGH);
  }
};

#endif // P25QXX_H
