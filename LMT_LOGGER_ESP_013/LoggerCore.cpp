#include "LoggerCore.h"
#include "LoggerCLI.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "driver/temperature_sensor.h"

// =============================================================================
// DEBUG MACROS
// =============================================================================

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

const uint32_t DEFAULT_PAGES_TO_LOG = 10;

// =============================================================================
// GLOBAL OBJECTS / STATE
// =============================================================================

// Sector scratch buffer for reserved tail storage read-modify-write
static uint8_t g_sectorBuf[FLASH_SECTOR_SIZE];

// Buffer for one live frame (Frame20 + CRC16)
static uint8_t g_liveFrameBuf[LIVE_FRAME_BYTES];

// Async live-frame request state (MODE_IDLE only)
static bool g_liveReqPending = false;
static bool g_liveReqAscii = false;
static uint32_t g_liveReqDeadlineMs = 0;

// IMU / Flash
ICM_20948_SPI myICM;
SPIFlash flash(PIN_FLASH_CS);

uint32_t flashCapacityBytes = 0;
uint32_t flashTotalPages = 0;
uint32_t flashRecordPages = 0;  // computed in .ino (excludes tail)

// NEW: derived split
uint32_t flashImuPages = 0;
uint32_t flashSyncPages = 0;

// One sync *page* holds 16 sync frames.
// One sync frame is logged every ~50 IMU pages (60s).
// → One sync page per 800 IMU pages.
static constexpr uint32_t IMU_PAGES_PER_SYNC_PAGE = 800;

// Hardware presence flags
bool imuPresent = false;
bool flashPresent = false;

// IMU simulation
bool imuSimulated = false;

// Playback / mode
PlaybackFormat playbackFormat = PLAYBACK_ASCII;
RunMode mode = MODE_IDLE;

// Recording state (IMU)
uint32_t pageStartMs = 0;
uint32_t pageFirstID = 0;
uint32_t recordPageLimit = 0;
uint32_t recordStartPage = 0;
uint32_t frameCounter = 0;

Frame20 pageFrames[FRAMES_PER_PAGE];
uint16_t frameIndexInPage = 0;
uint32_t currentPage = 0;  // IMU pages written

// Recording state (SYNC) — NEW
SyncFrame syncFrames[SYNC_FRAMES_PER_PAGE];
uint16_t syncFrameIndexInPage = 0;

uint32_t syncCurrentPage = 0;   // sync pages written
uint32_t syncFrameCounter = 0;  // sync frames written (monotonic ID)
uint32_t lastSyncMs = 0;        // last sample time

// Playback state (IMU pages only for now)
uint32_t playbackPage = 0;
uint16_t playbackFrameIndex = 0;
bool playbackPageLoaded = false;

uint32_t playbackPagesSeen = 0;
uint32_t playbackCrcWarnings = 0;

uint8_t playbackPageBuf[FLASH_PAGE_SIZE];
static uint8_t g_flushRawPage[FLASH_PAGE_SIZE];

Frame20 *playbackFrames = reinterpret_cast<Frame20 *>(playbackPageBuf);
PageFooter playbackFooter;

uint32_t playbackPageLimit = 0;

// Boot scan diagnostics
uint32_t bootPagesFound = 0;
uint32_t bootValidPages = 0;
uint32_t bootCorruptPages = 0;

// Command buffer (owned by core; filled by .ino and BLE RX)
char cmdBuf[CMD_BUF_SIZE];
uint16_t cmdLen = 0;

//Temperature sensor (ESP32 silicon)
static temperature_sensor_handle_t g_tempSensor = nullptr;
static bool g_tempSensorReady = false;

// =============================================================================
// OUTPUT HELPERS
// =============================================================================

static void initTempSensorOnce() {
  if (g_tempSensorReady) return;

  temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
  if (temperature_sensor_install(&cfg, &g_tempSensor) == ESP_OK) {
    temperature_sensor_enable(g_tempSensor);
    g_tempSensorReady = true;
  }
}

static int16_t readDieTempCx100() {
  if (!g_tempSensorReady) {
    initTempSensorOnce();
  }

  if (!g_tempSensorReady) {
    return INT16_MIN; // sentinel for “unavailable”
  }

  float temp_c = 0.0f;
  if (temperature_sensor_get_celsius(g_tempSensor, &temp_c) != ESP_OK) {
    return INT16_MIN;
  }

  return (int16_t)lroundf(temp_c * 100.0f);
}

static void streamWrite(const uint8_t *buf, size_t len) {
  Serial.write(buf, len);
  if (bleConnected()) {
    bleWrite(buf, len);
  }
}

static void streamPrintln(const char *s) {
  Serial.println(s);
  if (bleConnected()) {
    blePrintln(s);
  }
}

void emitControl(void (*fn)(Stream &)) {
  fn(Serial);

  if (bleConnected()) {
    class StringStream : public Stream {
    public:
      String s;
      int available() override {
        return 0;
      }
      int read() override {
        return -1;
      }
      int peek() override {
        return -1;
      }
      size_t write(uint8_t c) override {
        s += (char)c;
        return 1;
      }
    };

    StringStream tmp;
    fn(tmp);

    if (tmp.s.length() > 0) {
      blePrintln(tmp.s.c_str());
    }
  }
}

void emitEvent(const char *msg) {
  streamPrintln(msg);
}

// =============================================================================
// IMU READER and SIMULATOR
// =============================================================================

// Constant-rate rotation model (about Y axis so accel changes sensibly)
static float sim_angle = 0.0f;
static uint32_t sim_lastMs = 0;

static void simStep(Frame20 &f) {
  const uint32_t now = millis();
  if (sim_lastMs == 0) sim_lastMs = now;

  const float dt = (now - sim_lastMs) * 0.001f;
  sim_lastMs = now;

  // 45 deg/sec rotation about +Y
  const float omega = 45.0f * (float)M_PI / 180.0f;
  sim_angle += omega * dt;

  if (sim_angle > 2.0f * (float)M_PI) sim_angle -= 2.0f * (float)M_PI;

  const float half = sim_angle * 0.5f;

  // Quaternion for rotation about Y: q = [cos(a/2), 0, sin(a/2), 0]
  const float q0 = cosf(half);
  const float q2 = sinf(half);

  f.q0 = floatToQ15(q0);
  f.q1 = 0;
  f.q2 = floatToQ15(q2);
  f.q3 = 0;

  // Simulated accelerometer: gravity in body frame
  const float g = 16384.0f;
  f.ax = (int16_t)lroundf(sinf(sim_angle) * g);
  f.ay = 0;
  f.az = (int16_t)lroundf(cosf(sim_angle) * g);

  // Simulated magnetometer: simple world field along +X (arbitrary)
  const float B = 1000.0f;
  f.mx = (int16_t)lroundf(cosf(sim_angle) * B);
  f.my = 0;
  f.mz = (int16_t)lroundf(-sinf(sim_angle) * B);
}

bool imuReadFrame(Frame20 &out) {
  if (imuSimulated) {
    simStep(out);
    return true;
  }

  if (!imuPresent) {
    return false;
  }

  icm_20948_DMP_data_t dmpData;
  myICM.readDMPdataFromFIFO(&dmpData);

  if (!(myICM.status == ICM_20948_Stat_Ok || myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    return false;
  }

  if (!(dmpData.header & DMP_header_bitmap_Quat9)) {
    return false;
  }

  float fq1 = (float)dmpData.Quat9.Data.Q1 / 1073741824.0f;
  float fq2 = (float)dmpData.Quat9.Data.Q2 / 1073741824.0f;
  float fq3 = (float)dmpData.Quat9.Data.Q3 / 1073741824.0f;

  const float mag2 = fq1 * fq1 + fq2 * fq2 + fq3 * fq3;
  const float eps = 0.05f;

  if (!(mag2 >= 0.0f && mag2 <= (1.0f + eps))) {
    return false;
  }

  float t = 1.0f - mag2;
  if (t < 0) t = 0;
  if (t > 1) t = 1;

  float fq0 = sqrtf(t);

  out.q0 = floatToQ15(fq0);
  out.q1 = floatToQ15(fq1);
  out.q2 = floatToQ15(fq2);
  out.q3 = floatToQ15(fq3);

  myICM.getAGMT();
  out.ax = myICM.agmt.acc.axes.x;
  out.ay = myICM.agmt.acc.axes.y;
  out.az = myICM.agmt.acc.axes.z;
  out.mx = myICM.agmt.mag.axes.x;
  out.my = myICM.agmt.mag.axes.y;
  out.mz = myICM.agmt.mag.axes.z;

  return true;
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

// =============================================================================
// LIVE FRAME
// =============================================================================

void buildLiveFrame(uint8_t *dst) {
  Frame20 f;

  if (!imuReadFrame(f)) {
    memset(&f, 0, sizeof(f));
  }

  memcpy(dst, &f, sizeof(Frame20));

  const uint16_t crc = crc16_ccitt(dst, sizeof(Frame20));
  memcpy(dst + sizeof(Frame20), &crc, sizeof(crc));
}

void requestLiveFrameBinary(uint32_t timeoutMs) {
  g_liveReqPending = true;
  g_liveReqAscii = false;
  g_liveReqDeadlineMs = millis() + timeoutMs;
}

void requestLiveFrameAscii(uint32_t timeoutMs) {
  g_liveReqPending = true;
  g_liveReqAscii = true;
  g_liveReqDeadlineMs = millis() + timeoutMs;
}

bool liveFrameRequestPending() {
  return g_liveReqPending;
}

static void emitLiveFrameBinary(const uint8_t *buf, size_t len) {
  streamWrite(buf, len);
}

static void emitLiveFrameAscii(const uint8_t *buf) {
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

  streamPrintln(line);
}

void serviceLiveFrameRequests() {
  if (!g_liveReqPending) {
    return;
  }

  buildLiveFrame(g_liveFrameBuf);

  if (g_liveReqAscii) {
    emitLiveFrameAscii(g_liveFrameBuf);
  } else {
    emitLiveFrameBinary(g_liveFrameBuf, LIVE_FRAME_BYTES);
  }

  g_liveReqPending = false;
}

// =============================================================================
// DEVICE IDENTIFICATION
// =============================================================================

void getMCUSerialString(char *out, size_t outLen) {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(out, outLen, "%04X%08X",
           (uint16_t)(mac >> 32),
           (uint32_t)mac);
}

void printFirmwareVersion(Stream &out) {
  out.print("Firmware version: ");
  out.println(FW_VERSION);
}

void printMCUDeviceID(Stream &out) {
  uint64_t mac = ESP.getEfuseMac();
  out.print("MCU Device ID: 0x");
  out.println((uint32_t)mac, HEX);
}

void printIMUDeviceID(Stream &out) {
  uint8_t whoami = 0;

  SPI.beginTransaction(SPISettings(7000000, MSBFIRST, SPI_MODE3));

  digitalWrite(PIN_IMU_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x7F & 0x7F);  // REG_BANK_SEL (write)
  SPI.transfer(0x00);         // Bank 0
  digitalWrite(PIN_IMU_CS, HIGH);

  delayMicroseconds(1);

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
// RESERVED TAIL STORAGE (256 x 256-byte elements)
// =============================================================================

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
// INTERNAL: DERIVE IMU vs SYNC REGION SPLIT
// =============================================================================



// One sync *page* holds 16 sync frames.
// One sync frame is logged every ~50 IMU pages (60s).
// → One sync page per 800 IMU pages.

static void deriveRegionSplit() {
  flashSyncPages = 0;
  flashImuPages = flashRecordPages;

  if (flashRecordPages == 0) {
    return;
  }

  // Nominal ratio: 1 sync page per 800 IMU pages
  uint32_t desiredSync =
    flashRecordPages / IMU_PAGES_PER_SYNC_PAGE;

  // Enforce at least one sync page if we have any IMU storage
  if (desiredSync == 0) {
    desiredSync = 1;
  }

  // Never allocate more sync pages than total pages - 1
  if (desiredSync >= flashRecordPages) {
    desiredSync = flashRecordPages - 1;
  }

  flashSyncPages = desiredSync;
  flashImuPages = flashRecordPages - flashSyncPages;
}

// =============================================================================
// FLASH LAYOUT (PUBLIC / COMPATIBILITY)
// =============================================================================

uint32_t flashDataPages = 0;
uint32_t flashSyncBasePage = 0;
uint32_t flashStorageBasePage = 0;

void computeFlashLayout() {
  // flashTotalPages and flashRecordPages must already be set
  deriveRegionSplit();

  // IMU data pages are the front portion
  flashDataPages = flashImuPages;

  // Sync region starts immediately after IMU pages
  flashSyncBasePage = flashImuPages;

  // Tail storage always lives at the end of physical flash
  if (flashTotalPages >= FLASH_RESERVED_PAGES) {
    flashStorageBasePage = flashTotalPages - FLASH_RESERVED_PAGES;
  } else {
    flashStorageBasePage = 0;
  }
}

// =============================================================================
// FLASH BOOT SCAN (IMU region only)
// =============================================================================

void scanFlashOnBoot() {
  // Geometry must already be computed by computeFlashLayout()

  bootPagesFound = 0;
  bootValidPages = 0;
  bootCorruptPages = 0;

  uint8_t pageBuf[FLASH_PAGE_SIZE];

  for (uint32_t page = 0; page < flashImuPages; page++) {
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

void scanSyncPagesOnBoot() {

  syncCurrentPage = 0;
  syncFrameCounter = 0;

  if (flashSyncPages == 0) {
    return;
  }

  uint8_t pageBuf[FLASH_PAGE_SIZE];

  for (uint32_t i = 0; i < flashSyncPages; i++) {

    const uint32_t page = flashSyncBasePage + i;
    const uint32_t addr = page * FLASH_PAGE_SIZE;

    if (!flash.readData(addr, pageBuf, FLASH_PAGE_SIZE)) {
      break;
    }

    const uint16_t footerOffset =
      FLASH_PAGE_SIZE - sizeof(SyncPageFooter);

    SyncPageFooter footer;
    memcpy(&footer, pageBuf + footerOffset, sizeof(footer));

    if (footer.magic != SYNC_MAGIC) {
      break;
    }

    if (footer.validFrames > SYNC_FRAMES_PER_PAGE) {
      break;
    }

    // Valid sync page
    syncCurrentPage++;

    // Recover monotonic counter
    const uint32_t lastID =
      footer.firstSyncID + footer.validFrames - 1;

    if (lastID > syncFrameCounter) {
      syncFrameCounter = lastID;
    }
  }
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
// FLASH LOGGING (IMU)
// =============================================================================

void flushPageToFlash() {
  if (recordPageLimit > 0 && (currentPage - recordStartPage) >= recordPageLimit) {
    return;
  }

  if (currentPage >= flashImuPages) {
    emitEvent("# Flash full — recording stopped");
    mode = MODE_IDLE;

    // Ensure we can still flush any pending sync page when recording stops.
    flushPendingSyncPageToFlash();
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
  if (!flashPresent) {
    mode = MODE_IDLE;
    return false;
  }

  if (mode != MODE_RECORDING) {
    return false;
  }

  // Opportunistic sync scheduler (time-based)
  serviceSyncScheduler();

  if (recordPageLimit > 0 && frameIndexInPage == 0 && (currentPage - recordStartPage) >= recordPageLimit) {

    emitEvent("# Recording page limit reached");
    mode = MODE_IDLE;

    flushPendingSyncPageToFlash();
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
// SYNC LOGGING (NEW)
// =============================================================================

static uint32_t syncBasePage() {
  // Sync pages live immediately after IMU pages
  return flashImuPages;
}

static bool buildSyncFrame(SyncFrame &out) {
  // Beacon is mocked (returns 0) for now.
  out.master_unix_ms = getLastBeaconTimeMs();
  out.local_ms = millis();

  // Temperature 
  out.temp_c_x100 = readDieTempCx100();

  // CRC over first 14 bytes (exclude crc16 field)
  const uint16_t crc = crc16_ccitt((const uint8_t *)&out, offsetof(SyncFrame, crc16));
  out.crc16 = crc;

  return true;
}

void serviceSyncScheduler() {
  if (!flashPresent) return;
  if (mode != MODE_RECORDING) return;
  if (flashSyncPages == 0) return;  // sync disabled by geometry

  const uint32_t now = millis();

  // Initialize on first call during a session
  if (lastSyncMs == 0) {
    lastSyncMs = now;
    return;
  }

  if ((uint32_t)(now - lastSyncMs) < (uint32_t)SYNC_INTERVAL_MS) {
    return;
  }

  // Step forward by whole intervals to avoid drift accumulation if loop stalls.
  // (We still only emit one frame per call.)
  lastSyncMs += SYNC_INTERVAL_MS;

  // If sync region is full, we stop producing sync frames but do not stop IMU logging.
  if (syncCurrentPage >= flashSyncPages) {
    return;
  }

  SyncFrame sf;
  if (!buildSyncFrame(sf)) {
    return;
  }

  // If this is the first frame in the page, pageStartMs in footer should reflect it.
  // We'll compute footer.pageStartMs from this frame's local_ms when flushing.

  syncFrames[syncFrameIndexInPage++] = sf;
  syncFrameCounter++;

  if (syncFrameIndexInPage >= SYNC_FRAMES_PER_PAGE) {
    flushSyncPageToFlash();
  }
}

void flushSyncPageToFlash() {
  if (!flashPresent) return;
  if (flashSyncPages == 0) return;
  if (syncCurrentPage >= flashSyncPages) return;

  // Only flush full pages here
  if (syncFrameIndexInPage < SYNC_FRAMES_PER_PAGE) {
    return;
  }

  const uint32_t page = syncBasePage() + syncCurrentPage;
  const uint32_t addr = page * FLASH_PAGE_SIZE;

  uint8_t raw[FLASH_PAGE_SIZE];

  const uint16_t usedBytes = SYNC_FRAMES_PER_PAGE * sizeof(SyncFrame);
  memcpy(raw, syncFrames, usedBytes);

  const uint16_t footerOffset = FLASH_PAGE_SIZE - sizeof(SyncPageFooter);
  memset(raw + usedBytes, 0xFF, footerOffset - usedBytes);

  SyncPageFooter footer;
  footer.magic = SYNC_MAGIC;
  footer.validFrames = syncFrameIndexInPage;  // should be 16 here
  footer.crc16 = 0;

  // firstSyncID is 1-based, and syncFrameCounter is the total count after increment
  // This page holds the last 'validFrames' frames, so:
  // first = (syncFrameCounter - validFrames) + 1
  footer.firstSyncID = (syncFrameCounter - footer.validFrames) + 1;

  // local_ms of first frame in this page
  footer.pageStartMs = syncFrames[0].local_ms;

  memcpy(raw + footerOffset, &footer, sizeof(footer));

  const uint16_t crcLen = usedBytes + offsetof(SyncPageFooter, crc16);
  ((SyncPageFooter *)(raw + footerOffset))->crc16 = crc16_ccitt(raw, crcLen);

  flash.writePage(addr, raw, FLASH_PAGE_SIZE);

  // Reset for next page
  syncFrameIndexInPage = 0;
  memset(syncFrames, 0, sizeof(syncFrames));
  syncCurrentPage++;
}

void flushPendingSyncPageToFlash() {
  if (!flashPresent) return;
  if (flashSyncPages == 0) return;
  if (syncCurrentPage >= flashSyncPages) return;

  // If nothing pending, nothing to do.
  if (syncFrameIndexInPage == 0) {
    return;
  }

  const uint32_t page = syncBasePage() + syncCurrentPage;
  const uint32_t addr = page * FLASH_PAGE_SIZE;

  uint8_t raw[FLASH_PAGE_SIZE];

  const uint16_t usedBytes = syncFrameIndexInPage * sizeof(SyncFrame);
  memcpy(raw, syncFrames, usedBytes);

  const uint16_t footerOffset = FLASH_PAGE_SIZE - sizeof(SyncPageFooter);
  memset(raw + usedBytes, 0xFF, footerOffset - usedBytes);

  SyncPageFooter footer;
  footer.magic = SYNC_MAGIC;
  footer.validFrames = syncFrameIndexInPage;
  footer.crc16 = 0;

  footer.firstSyncID = (syncFrameCounter - footer.validFrames) + 1;
  footer.pageStartMs = syncFrames[0].local_ms;

  memcpy(raw + footerOffset, &footer, sizeof(footer));

  const uint16_t crcLen = usedBytes + offsetof(SyncPageFooter, crc16);
  ((SyncPageFooter *)(raw + footerOffset))->crc16 = crc16_ccitt(raw, crcLen);

  flash.writePage(addr, raw, FLASH_PAGE_SIZE);

  // Reset pending buffer
  syncFrameIndexInPage = 0;
  memset(syncFrames, 0, sizeof(syncFrames));
  syncCurrentPage++;
}

// =============================================================================
// PLAYBACK
// =============================================================================

void playbackTask() {
  if (!flashPresent) {
    emitEvent("# Flash unavailable – cannot playback");
    mode = MODE_IDLE;
    printPrompt();
    return;
  }

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
    memcpy(&playbackFooter, playbackPageBuf + footerOffset, sizeof(PageFooter));

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

      streamPrintln(line);

    } else {

      uint8_t pkt[24];
      pkt[0] = 0x55;
      pkt[1] = 0xAA;
      pkt[2] = sizeof(Frame20);
      pkt[3] = 0x00;

      memcpy(&pkt[4], &f, sizeof(Frame20));
      streamWrite(pkt, sizeof(pkt));
    }

    playbackFrameIndex++;
    return;
  }

  playbackPage++;
  playbackPageLoaded = false;
}

void dumpSyncPagesAscii() {

  if (!flashPresent || flashSyncPages == 0) {
    emitEvent("# No sync region present");
    return;
  }

  uint8_t pageBuf[FLASH_PAGE_SIZE];

  for (uint32_t i = 0; i < syncCurrentPage; i++) {

    const uint32_t page = flashSyncBasePage + i;
    const uint32_t addr = page * FLASH_PAGE_SIZE;

    flash.readData(addr, pageBuf, FLASH_PAGE_SIZE);

    const uint16_t footerOffset =
      FLASH_PAGE_SIZE - sizeof(SyncPageFooter);

    SyncPageFooter footer;
    memcpy(&footer, pageBuf + footerOffset, sizeof(footer));

    if (footer.magic != SYNC_MAGIC) {
      continue;
    }

    // Page header
    char hdr[96];
    snprintf(hdr, sizeof(hdr),
             "@SYNC_PAGE %lu frames=%u firstID=%lu start_ms=%lu",
             (unsigned long)i,
             footer.validFrames,
             (unsigned long)footer.firstSyncID,
             (unsigned long)footer.pageStartMs);

    emitEvent(hdr);

    // Frames
    SyncFrame *frames = (SyncFrame *)pageBuf;

    for (uint16_t f = 0; f < footer.validFrames; f++) {
      char line[128];
      snprintf(line, sizeof(line),
               "  %lu unix_ms=%llu local_ms=%lu temp_x100=%d crc=0x%04X",
               (unsigned long)(footer.firstSyncID + f),
               (unsigned long long)frames[f].master_unix_ms,
               (unsigned long)frames[f].local_ms,
               frames[f].temp_c_x100,
               frames[f].crc16);

      emitEvent(line);
    }
  }
}

// =============================================================================
// RECORDING CONTROL
// =============================================================================

void startNewRecordingSession() {
  if (!flashPresent) {
    emitEvent("# Flash unavailable – cannot record");
    return;
  }

  if (flashSyncPages > 0 && syncCurrentPage >= flashSyncPages) {
    emitEvent("# No free sync pages — recording disabled");
    return;
  }

  emitEvent("# Starting recording session (append)");

  frameIndexInPage = 0;
  recordPageLimit = 0;
  recordStartPage = currentPage;

  playbackPage = 0;
  playbackFrameIndex = 0;
  playbackPageLoaded = false;

  memset(pageFrames, 0, sizeof(pageFrames));

  // NEW: reset sync session state
  syncFrameIndexInPage = 0;
  syncCurrentPage = 0;
  syncFrameCounter = 0;
  // Force immediate sync on first scheduler pass
  lastSyncMs = millis() - SYNC_INTERVAL_MS;
  memset(syncFrames, 0, sizeof(syncFrames));

  myICM.resetFIFO();
  myICM.resetDMP();

  mode = MODE_RECORDING;

  emitEvent("# Recording started (append mode)");
}

// =============================================================================
// PAGE FOOTER EMISSION
// =============================================================================

void emitAsciiPageFooter(uint32_t page, const PageFooter &f, bool crcOk) {
  char line[128];
  snprintf(line, sizeof(line),
           "@PAGE %lu %u %lu %lu 0x%04X %s",
           (unsigned long)page,
           f.validFrames,
           (unsigned long)f.firstFrameID,
           (unsigned long)f.pageStartMs,
           f.crc16,
           crcOk ? "OK" : "BAD");

  streamPrintln(line);
}

void emitBinaryPageFooter(const PageFooter &f) {
  uint8_t pkt[4 + sizeof(PageFooter)];
  pkt[0] = 0x56;
  pkt[1] = 0xAA;
  pkt[2] = sizeof(PageFooter);
  pkt[3] = 0x00;

  memcpy(&pkt[4], &f, sizeof(PageFooter));
  streamWrite(pkt, sizeof(pkt));
}