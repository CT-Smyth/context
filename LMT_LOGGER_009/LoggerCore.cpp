#include "LoggerCore.h"
#include <math.h>    // sqrtf, lroundf
#include <stdio.h>   // snprintf
#include <string.h>  // memcpy, memset

// =============================================================================
// DEBUG MACROS
// =============================================================================
//
// NOTE: Kept identical in behavior. Formatting adjusted for readability.

#if VERBOSE_LOG
#define DBG_PRINT(x) Serial.print(x)
#define DBG_PRINTLN(x) Serial.println(x)
#else
#define DBG_PRINT(x) \
  do { \
  } while (0)
#define DBG_PRINTLN(x) \
  do { \
  } while (0)
#endif

// =============================================================================
// CONFIG / CONSTANTS
// =============================================================================

// IMU and Flash pins (XIAO nRF52840)
const uint8_t PIN_IMU_CS = D5;
const uint8_t PIN_FLASH_CS = D2;

const uint32_t DEFAULT_PAGES_TO_LOG = 10;

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================

// -----------------------------------------------------------------------------
// Static flash sector buffer
// -----------------------------------------------------------------------------
static uint8_t g_sectorBuf[FLASH_SECTOR_SIZE];

// -----------------------------------------------------------------------------
// Static buffer for one live frame (no flash interaction)
// -----------------------------------------------------------------------------
static uint8_t g_liveFrameBuf[LIVE_FRAME_BYTES];

// -----------------------------------------------------------------------------
// Async live-frame request state (MODE_IDLE only)
// -----------------------------------------------------------------------------
static bool     g_liveReqPending = false;
static bool     g_liveReqAscii = false;
static uint32_t g_liveReqDeadlineMs = 0;

// IMU / Flash
ICM_20948_SPI myICM;
SPIFlash flash(PIN_FLASH_CS);

uint32_t flashCapacityBytes = 0;
uint32_t flashTotalPages = 0;
uint32_t flashRecordPages = 0;

// Playback / mode
PlaybackFormat playbackFormat = PLAYBACK_ASCII;
RunMode mode = MODE_IDLE;

// Recording state
uint32_t pageStartMs = 0;
uint32_t pageFirstID = 0;
uint32_t recordPageLimit = 0;
uint32_t recordStartPage = 0;
uint32_t frameCounter = 0;

Frame20 pageFrames[FRAMES_PER_PAGE];
uint16_t frameIndexInPage = 0;
uint32_t currentPage = 0;

// Playback state
uint32_t playbackPage = 0;
uint16_t playbackFrameIndex = 0;
bool playbackPageLoaded = false;

uint32_t playbackPagesSeen = 0;
uint32_t playbackCrcWarnings = 0;

uint8_t playbackPageBuf[FLASH_PAGE_SIZE];
// Scratch page buffer used by flushPageToFlash() to avoid stack allocation
static uint8_t g_flushRawPage[FLASH_PAGE_SIZE];
Frame20 *playbackFrames = reinterpret_cast<Frame20 *>(playbackPageBuf);
PageFooter playbackFooter;

uint32_t playbackPageLimit = 0;

// Boot scan diagnostics
uint32_t bootPagesFound = 0;
uint32_t bootValidPages = 0;
uint32_t bootCorruptPages = 0;

// Command buffer (owned by core; filled by .ino input loop)
char cmdBuf[CMD_BUF_SIZE];
uint16_t cmdLen = 0;

// =============================================================================
// OUTPUT HELPERS
// =============================================================================
//
// NOTE: These remain here because they are used across subsystems (Core, BLE, CLI).
// No behavioral changes.

void emitControl(void (*fn)(Stream &)) {
  fn(Serial);

  if (bleUartEnabled && bleConnected) {
    fn(bleuart);
  }
}

void emitEvent(const char *msg) {
  Serial.println(msg);

  if (bleUartEnabled && bleConnected && bleuart.notifyEnabled()) {
    bleuart.println(msg);
  }
}

// =============================================================================
// UTILITIES
// =============================================================================

int16_t floatToQ15(float x) {
  if (x > 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;

  long v = (long)lroundf(x * 32767.0f);
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;

  return (int16_t)v;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;

  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;

    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }

  return crc;
}

// Attempt to acquire a valid quaternion and build a live frame.
// Returns true only if a valid quaternion was obtained.
// On false, dst is not modified (caller can retry or timeout).
static bool tryBuildLiveFrameOnce(uint8_t *dst)
{
  Frame20 f;
  memset(&f, 0, sizeof(f));

  icm_20948_DMP_data_t d0;
  icm_20948_DMP_data_t d1;

  bool gotDualQuat = false;

  // Small bounded scan per attempt. One attempt should be fast.
  for (int i = 0; i < 12; i++) {

    myICM.readDMPdataFromFIFO(&d0);
    if (!(myICM.status == ICM_20948_Stat_Ok ||
          myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
      break;
    }
    if (!(d0.header & DMP_header_bitmap_Quat9)) {
      continue;
    }

    myICM.readDMPdataFromFIFO(&d1);
    if (!(myICM.status == ICM_20948_Stat_Ok ||
          myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
      break;
    }
    if (!(d1.header & DMP_header_bitmap_Quat9)) {
      continue;
    }

    gotDualQuat = true;
    break;
  }

  if (!gotDualQuat) {
    return false;
  }

  // Quaternion reconstruction from first packet of the validated pair
  float fq1 = (float)d0.Quat9.Data.Q1 / 1073741824.0f;
  float fq2 = (float)d0.Quat9.Data.Q2 / 1073741824.0f;
  float fq3 = (float)d0.Quat9.Data.Q3 / 1073741824.0f;

  // Semantic validation: vector part must be near-unit (<= 1 + eps)
  // This rejects the "stuck/saturated" DMP states you observed.
  const float mag2 = fq1 * fq1 + fq2 * fq2 + fq3 * fq3;
  const float eps = 0.05f;

  if (!(mag2 >= 0.0f && mag2 <= (1.0f + eps))) {
    return false;
  }

  float t = 1.0f - mag2;
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  float fq0 = sqrtf(t);

  f.q0 = floatToQ15(fq0);
  f.q1 = floatToQ15(fq1);
  f.q2 = floatToQ15(fq2);
  f.q3 = floatToQ15(fq3);

  // Raw accel + mag sampled "now" (same as your existing live frame design)
  myICM.getAGMT();
  f.ax = myICM.agmt.acc.axes.x;
  f.ay = myICM.agmt.acc.axes.y;
  f.az = myICM.agmt.acc.axes.z;
  f.mx = myICM.agmt.mag.axes.x;
  f.my = myICM.agmt.mag.axes.y;
  f.mz = myICM.agmt.mag.axes.z;

  memcpy(dst, &f, sizeof(Frame20));
  const uint16_t crc = crc16_ccitt(dst, sizeof(Frame20));
  memcpy(dst + sizeof(Frame20), &crc, sizeof(crc));

  return true;
}

// Build a failure frame (zero quat) with current accel/mag and valid CRC.
static void buildLiveFrameFailure(uint8_t *dst)
{
  Frame20 f;
  memset(&f, 0, sizeof(f));

  // quat remains 0,0,0,0
  myICM.getAGMT();
  f.ax = myICM.agmt.acc.axes.x;
  f.ay = myICM.agmt.acc.axes.y;
  f.az = myICM.agmt.acc.axes.z;
  f.mx = myICM.agmt.mag.axes.x;
  f.my = myICM.agmt.mag.axes.y;
  f.mz = myICM.agmt.mag.axes.z;

  memcpy(dst, &f, sizeof(Frame20));
  const uint16_t crc = crc16_ccitt(dst, sizeof(Frame20));
  memcpy(dst + sizeof(Frame20), &crc, sizeof(crc));
}

void buildLiveFrame(uint8_t *dst)
{
  // Build one Frame20 using the same logic as MODE_RECORDING in LMT_LOGGER_008_4.ino
  Frame20 f;
  memset(&f, 0, sizeof(f));

icm_20948_DMP_data_t dmpData;
icm_20948_DMP_data_t nextData;

bool gotValidQuat = false;

// Bounded FIFO scan
for (int i = 0; i < 12; i++) {

  // Read first packet
  myICM.readDMPdataFromFIFO(&dmpData);

  if (!(myICM.status == ICM_20948_Stat_Ok ||
        myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    break;
  }

  // First must be Quat9
  if (!(dmpData.header & DMP_header_bitmap_Quat9)) {
    continue;
  }

  // Read second packet immediately
  myICM.readDMPdataFromFIFO(&nextData);

  if (!(myICM.status == ICM_20948_Stat_Ok ||
        myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    break;
  }

  // Second must also be Quat9
  if (!(nextData.header & DMP_header_bitmap_Quat9)) {
    continue;
  }

  // ---- Valid dual-Quat9 window found ----
  {
    float fq1 = (float)dmpData.Quat9.Data.Q1 / 1073741824.0f;
    float fq2 = (float)dmpData.Quat9.Data.Q2 / 1073741824.0f;
    float fq3 = (float)dmpData.Quat9.Data.Q3 / 1073741824.0f;

    float mag2 = fq1 * fq1 + fq2 * fq2 + fq3 * fq3;
    float t = 1.0f - mag2;
    if (t < 0) t = 0;
    float fq0 = sqrtf(t);

    f.q0 = floatToQ15(fq0);
    f.q1 = floatToQ15(fq1);
    f.q2 = floatToQ15(fq2);
    f.q3 = floatToQ15(fq3);

    gotValidQuat = true;
  }

  break;
}

if (!gotValidQuat) {
  f.q0 = 0;
  f.q1 = 0;
  f.q2 = 0;
  f.q3 = 0;
}

  // --- Raw accel + mag (copied from LMT_LOGGER_008_4.ino) ---
  myICM.getAGMT();
  f.ax = myICM.agmt.acc.axes.x;
  f.ay = myICM.agmt.acc.axes.y;
  f.az = myICM.agmt.acc.axes.z;
  f.mx = myICM.agmt.mag.axes.x;
  f.my = myICM.agmt.mag.axes.y;
  f.mz = myICM.agmt.mag.axes.z;

  // Emit Frame20 + CRC16(Frame20) into dst
  memcpy(dst, &f, sizeof(Frame20));

  const uint16_t crc = crc16_ccitt(dst, sizeof(Frame20));
  memcpy(dst + sizeof(Frame20), &crc, sizeof(crc));
}

void requestLiveFrameBinary(uint32_t timeoutMs)
{
  g_liveReqPending = true;
  g_liveReqAscii = false;
  g_liveReqDeadlineMs = millis() + timeoutMs;
}

void requestLiveFrameAscii(uint32_t timeoutMs)
{
  g_liveReqPending = true;
  g_liveReqAscii = true;
  g_liveReqDeadlineMs = millis() + timeoutMs;
}

bool liveFrameRequestPending()
{
  return g_liveReqPending;
}

static void emitLiveFrameBinary(const uint8_t *buf, size_t len)
{
  Serial.write(buf, len);
  if (bleUartEnabled && bleConnected && bleuart.notifyEnabled()) {
    bleuart.write(buf, len);
  }
}

static void emitLiveFrameAscii(const uint8_t *buf)
{
  const Frame20 *f = (const Frame20 *)buf;
  uint16_t crc;
  memcpy(&crc, buf + sizeof(Frame20), sizeof(crc));

  char line[128];
  snprintf(line, sizeof(line),
           "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,0x%04X",
           f->q0, f->q1, f->q2, f->q3,
           f->ax, f->ay, f->az,
           f->mx, f->my, f->mz,
           crc);

  Serial.println(line);
  if (bleUartEnabled && bleConnected && bleuart.notifyEnabled()) {
    bleuart.println(line);
  }
}

void serviceLiveFrameRequests()
{
  if (!g_liveReqPending) {
    return;
  }

  const uint32_t now = millis();

  // Deadline reached: emit explicit failure (zero quat)
  if ((int32_t)(now - g_liveReqDeadlineMs) >= 0) {
    buildLiveFrameFailure(g_liveFrameBuf);

    if (g_liveReqAscii) {
      emitLiveFrameAscii(g_liveFrameBuf);
    } else {
      emitLiveFrameBinary(g_liveFrameBuf, LIVE_FRAME_BYTES);
    }

    g_liveReqPending = false;
    return;
  }

  // One attempt per service call (cooperative)
  if (tryBuildLiveFrameOnce(g_liveFrameBuf)) {

    if (g_liveReqAscii) {
      emitLiveFrameAscii(g_liveFrameBuf);
    } else {
      emitLiveFrameBinary(g_liveFrameBuf, LIVE_FRAME_BYTES);
    }

    g_liveReqPending = false;
    return;
  }

  // Not valid yet; return immediately.
  // Next loop iteration will retry and naturally yields to SoftDevice.
}

void getMCUSerialString(char *out, size_t outLen) {
  uint32_t id0 = NRF_FICR->DEVICEID[0];
  uint32_t id1 = NRF_FICR->DEVICEID[1];

  snprintf(out, outLen, "%08lX%08lX",
           (unsigned long)id1,
           (unsigned long)id0);
}

// =============================================================================
// RESERVED TAIL STORAGE (256 x 256-byte elements)
// =============================================================================
//
// Storage lives in the last FLASH_RESERVED_PAGES pages of the chip.
// These pages are excluded from motion logging and boot-time page scanning.
//
// NOTE:
//  - Writes are sector-granular (4 KB). To preserve adjacent elements, we
//    read-modify-erase-rewrite the containing sector.

static uint32_t storageBasePage() {
  if (flashTotalPages < FLASH_RESERVED_PAGES) {
    return 0;
  }
  return flashTotalPages - FLASH_RESERVED_PAGES;
}

static uint32_t storagePageForIndex(uint16_t index) {
  return storageBasePage() + (uint32_t)index;
}

bool readStorageElement(uint16_t index, uint8_t *out256) {
  if (!out256 || index >= FLASH_RESERVED_PAGES) {
    return false;
  }

  const uint32_t page = storagePageForIndex(index);
  const uint32_t addr = page * FLASH_PAGE_SIZE;
  return flash.readData(addr, out256, FLASH_PAGE_SIZE);
}

bool writeStorageElement(uint16_t index, const uint8_t *in256) {
  if (!in256 || index >= FLASH_RESERVED_PAGES) {
    return false;
  }

  const uint32_t pageAddr = storagePageForIndex(index) * FLASH_PAGE_SIZE;
  const uint32_t sectorAddr = pageAddr & ~(uint32_t)(FLASH_SECTOR_SIZE - 1);
  const uint32_t pageOffset = pageAddr - sectorAddr;

  if (!flash.readData(sectorAddr, g_sectorBuf, FLASH_SECTOR_SIZE)) {
    return false;
  }

  memcpy(g_sectorBuf + pageOffset, in256, FLASH_PAGE_SIZE);

  if (!flash.eraseSector(sectorAddr)) {
    return false;
  }

  for (uint32_t off = 0; off < FLASH_SECTOR_SIZE; off += FLASH_PAGE_SIZE) {
    if (!flash.writePage(sectorAddr + off, g_sectorBuf + off, FLASH_PAGE_SIZE)) {
      return false;
    }
  }

  return true;
}


// =============================================================================
// DEVICE IDENTIFICATION
// =============================================================================

void printFirmwareVersion(Stream &out) {
  out.print("Firmware version: ");
  out.println(FW_VERSION);
}

void printMCUDeviceID(Stream &out) {
  uint32_t id0 = NRF_FICR->DEVICEID[0];
  uint32_t id1 = NRF_FICR->DEVICEID[1];

  out.print("MCU Device ID: 0x");
  out.print(id1, HEX);
  out.println(id0, HEX);
}

void printIMUDeviceID(Stream &out) {
  uint8_t whoami = 0;

  SPI.beginTransaction(SPISettings(7000000, MSBFIRST, SPI_MODE3));

  // Select USER BANK 0
  digitalWrite(PIN_IMU_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x7F & 0x7F);  // REG_BANK_SEL (write)
  SPI.transfer(0x00);         // Bank 0
  digitalWrite(PIN_IMU_CS, HIGH);

  delayMicroseconds(1);

  // Read WHO_AM_I (0x00)
  digitalWrite(PIN_IMU_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x80 | 0x00);  // Read WHO_AM_I
  whoami = SPI.transfer(0x00);
  digitalWrite(PIN_IMU_CS, HIGH);

  SPI.endTransaction();

  out.print("IMU WHO_AM_I: 0x");
  out.print(whoami, HEX);

  if (whoami == 0xEA) {
    out.println(" (ICM-20948)");
  } else {
    out.println(" (unexpected)");
  }
}

// =============================================================================
// FLASH BOOT SCAN
// =============================================================================

void scanFlashOnBoot() {
  bootPagesFound = 0;
  bootValidPages = 0;
  bootCorruptPages = 0;

  uint8_t pageBuf[FLASH_PAGE_SIZE];

  for (uint32_t page = 0; page < flashRecordPages; page++) {

    const uint32_t addr = page * FLASH_PAGE_SIZE;
    if (!flash.readData(addr, pageBuf, FLASH_PAGE_SIZE)) {
      break;
    }

    const uint16_t footerOffset = FLASH_PAGE_SIZE - sizeof(PageFooter);

    PageFooter footer;
    memcpy(&footer, pageBuf + footerOffset, sizeof(PageFooter));

    if (footer.magic != PAGE_MAGIC) {
      break;
    }

    bootPagesFound++;

    const bool footerSane = (footer.validFrames <= FRAMES_PER_PAGE);
    bool crcOk = false;

    if (footerSane) {
      const uint16_t usedBytes = footer.validFrames * sizeof(Frame20);
      const uint16_t crcLen = usedBytes + offsetof(PageFooter, crc16);

      crcOk = (crc16_ccitt(pageBuf, crcLen) == footer.crc16);
    }

    if (footerSane && crcOk) {
      bootValidPages++;
    } else {
      bootCorruptPages++;
    }
  }

  currentPage = bootPagesFound;
}

void reconstructFrameCounterFromFlash() {
  if (currentPage == 0) {
    frameCounter = 0;
    return;
  }

  const uint32_t addr =
    (currentPage - 1) * FLASH_PAGE_SIZE + FLASH_PAGE_SIZE - sizeof(PageFooter);

  PageFooter footer;
  if (!flash.readData(addr, (uint8_t *)&footer, sizeof(PageFooter))) {
    frameCounter = 0;
    return;
  }

  if (footer.magic != PAGE_MAGIC || footer.validFrames > FRAMES_PER_PAGE) {
    frameCounter = 0;
    return;
  }

  frameCounter = footer.firstFrameID + footer.validFrames;
}

// =============================================================================
// FLASH LOGGING
// =============================================================================

void flushPageToFlash() {

  if (recordPageLimit > 0 && (currentPage - recordStartPage) >= recordPageLimit) {
    return;
  }

  if (currentPage >= flashRecordPages) {
    emitEvent("# Flash full — recording stopped");
    mode = MODE_IDLE;
    return;
  }

  const uint32_t addr = currentPage * FLASH_PAGE_SIZE;

  const uint16_t usedBytes = FRAMES_PER_PAGE * sizeof(Frame20);
  memcpy(g_flushRawPage, pageFrames, usedBytes);

  const uint16_t footerOffset = FLASH_PAGE_SIZE - sizeof(PageFooter);
  memset(g_flushRawPage + usedBytes, 0xFF, footerOffset - usedBytes);

  PageFooter footer;
  footer.magic = PAGE_MAGIC;
  footer.validFrames = frameIndexInPage;
  footer.crc16 = 0;
  footer.firstFrameID = pageFirstID;
  footer.pageStartMs = pageStartMs;

  memcpy(g_flushRawPage + footerOffset, &footer, sizeof(PageFooter));

  const uint16_t crcLen = usedBytes + offsetof(PageFooter, crc16);
  ((PageFooter *)(g_flushRawPage + footerOffset))->crc16 = crc16_ccitt(g_flushRawPage, crcLen);

  flash.writePage(addr, g_flushRawPage, FLASH_PAGE_SIZE);

  frameIndexInPage = 0;
  currentPage++;
}

bool logFrame(const Frame20 &f) {
  if (mode != MODE_RECORDING) {
    return false;
  }

  if (recordPageLimit > 0 && frameIndexInPage == 0 && (currentPage - recordStartPage) >= recordPageLimit) {

    emitEvent("# Recording page limit reached");
    mode = MODE_IDLE;
    printPrompt();
    return false;
  }

  pageFrames[frameIndexInPage++] = f;

  if (frameIndexInPage >= FRAMES_PER_PAGE) {
    flushPageToFlash();
  }

  return true;
}

// =============================================================================
// PLAYBACK
// =============================================================================
//
// NOTE: Kept behavior identical. Minor clarity improvements only.
// Structural issue to discuss (not implemented): CRC check currently increments
// warnings but always calls emitAsciiPageFooter(..., true). That "true" is
// misleading, but changing it could alter output semantics; left as-is.

void playbackTask() {

  if ((playbackPageLimit > 0 && playbackPage >= playbackPageLimit) || (playbackPage >= currentPage)) {

    Serial.println();
    Serial.println("Dump summary:");
    Serial.print("  Pages processed: ");
    Serial.println(playbackPagesSeen);
    Serial.print("  CRC warnings:   ");
    Serial.println(playbackCrcWarnings);
    Serial.println();

    emitEvent("# Dump complete");

    mode = MODE_IDLE;
    printPrompt();
    return;
  }

  if (!playbackPageLoaded) {

    const uint32_t addr = playbackPage * FLASH_PAGE_SIZE;
    flash.readData(addr, playbackPageBuf, FLASH_PAGE_SIZE);

    const uint16_t footerOffset = FLASH_PAGE_SIZE - sizeof(PageFooter);
    memcpy(&playbackFooter,
           playbackPageBuf + footerOffset,
           sizeof(PageFooter));

    if (playbackFooter.magic != PAGE_MAGIC || playbackFooter.validFrames > FRAMES_PER_PAGE) {
      playbackPage++;
      playbackPageLoaded = false;
      return;
    }

    const uint16_t usedBytes = playbackFooter.validFrames * sizeof(Frame20);
    const uint16_t crcLen = usedBytes + offsetof(PageFooter, crc16);

    if (crc16_ccitt(playbackPageBuf, crcLen) != playbackFooter.crc16) {
      playbackCrcWarnings++;
    }

    if (playbackFormat == PLAYBACK_ASCII) {
      emitAsciiPageFooter(playbackPage, playbackFooter, true);
    } else {
      emitBinaryPageFooter(playbackFooter);
    }

    playbackFrameIndex = 0;
    playbackPageLoaded = true;
    playbackPagesSeen++;
    return;
  }

  if (playbackFrameIndex < playbackFooter.validFrames) {

    Frame20 &f = playbackFrames[playbackFrameIndex];
    const uint32_t id = playbackFooter.firstFrameID + playbackFrameIndex;

    if (playbackFormat == PLAYBACK_ASCII) {

      char line[96];
      snprintf(line, sizeof(line),
               "%lu %d %d %d %d %d %d %d %d %d %d",
               (unsigned long)id,
               f.q0, f.q1, f.q2, f.q3,
               f.ax, f.ay, f.az,
               f.mx, f.my, f.mz);

      Serial.println(line);
      if (bleUartEnabled && bleConnected && bleuart.notifyEnabled()) {
        bleuart.println(line);
      }

    } else {

      uint8_t pkt[24];
      pkt[0] = 0x55;
      pkt[1] = 0xAA;
      pkt[2] = sizeof(Frame20);
      pkt[3] = 0x00;

      memcpy(&pkt[4], &f, sizeof(Frame20));

      Serial.write(pkt, sizeof(pkt));
      if (bleUartEnabled && bleConnected && bleuart.notifyEnabled()) {
        bleuart.write(pkt, sizeof(pkt));
      }
    }

    playbackFrameIndex++;
    return;
  }

  playbackPage++;
  playbackPageLoaded = false;
}

// =============================================================================
// RECORDING
// =============================================================================

void startNewRecordingSession() {

  emitEvent("# Starting recording session (append)");

  frameIndexInPage = 0;
  recordPageLimit = 0;
  recordStartPage = currentPage;

  playbackPage = 0;
  playbackFrameIndex = 0;
  playbackPageLoaded = false;

  memset(pageFrames, 0, sizeof(pageFrames));

  myICM.resetFIFO();
  myICM.resetDMP();

  mode = MODE_RECORDING;

  emitEvent("# Recording started (append mode)");
}

// =============================================================================
// PAGE FOOTER EMISSION
// =============================================================================

void emitAsciiPageFooter(uint32_t page,
                         const PageFooter &f,
                         bool crcOk) {

  char line[128];
  snprintf(line, sizeof(line),
           "@PAGE %lu %u %lu %lu 0x%04X %s",
           (unsigned long)page,
           f.validFrames,
           (unsigned long)f.firstFrameID,
           (unsigned long)f.pageStartMs,
           f.crc16,
           crcOk ? "OK" : "BAD");

  Serial.println(line);
  if (bleUartEnabled && bleConnected && bleuart.notifyEnabled()) {
    bleuart.println(line);
  }
}

void emitBinaryPageFooter(const PageFooter &f) {

  uint8_t pkt[4 + sizeof(PageFooter)];
  pkt[0] = 0x56;
  pkt[1] = 0xAA;
  pkt[2] = sizeof(PageFooter);
  pkt[3] = 0x00;

  memcpy(&pkt[4], &f, sizeof(PageFooter));

  Serial.write(pkt, sizeof(pkt));
  if (bleUartEnabled && bleConnected && bleuart.notifyEnabled()) {
    bleuart.write(pkt, sizeof(pkt));
  }
}
