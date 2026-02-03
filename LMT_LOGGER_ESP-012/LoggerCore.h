#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "ICM_20948.h"
#include "SPIFlash.h"

#include "LoggerSync.h"
#include "LoggerBeacon.h"

// =============================================================================
// LoggerCore — Domain / State Owner
// =============================================================================
//
// Owns:
//  - Frame/page formats (on-flash ABI)
//  - Recording/playback state machines
//  - Boot-time flash scan + frameCounter reconstruction
//  - Output planes (CONTROL/EVENT)
//  - Reserved tail storage (256 x 256-byte slots at end of flash)
//  - Sync-log reservation + buffering + flush (NEW)
//
// ABI NOTES
// - Structs are written/read as raw bytes.
// - Little-endian target.
// - Do not reorder/resize structs without versioning.
//

// =============================================================================
// CONFIGURATION
// =============================================================================

#define FW_VERSION "0.13"

// Enable BLE UART automatically at boot (consumed by .ino)
#define BLEonstartup 1

// Logging / playback configuration
#define VERBOSE_LOG 1
// #define ERASE_FLASH_AT_START 1

// =============================================================================
// FRAME FORMAT (20 bytes)
// =============================================================================
//
// Stored frame format (20 bytes), little-endian.
//
struct Frame20 {
  int16_t q0, q1, q2, q3;  // Quaternion (Q15)
  int16_t ax, ay, az;      // Accelerometer (raw)
  int16_t mx, my, mz;      // Magnetometer (raw)
};
static_assert(sizeof(Frame20) == 20, "Frame20 must be exactly 20 bytes");

// Live "frame" command response = Frame20 + CRC16(Frame20)
static constexpr size_t LIVE_FRAME_BYTES = sizeof(Frame20) + sizeof(uint16_t);

// =============================================================================
// PAGE FOOTER FORMAT (IMU pages) — 16 bytes
// =============================================================================

#define PAGE_MAGIC 0x50414745UL  // ASCII "PAGE"

struct PageFooter {
  uint32_t magic;         // PAGE_MAGIC
  uint16_t validFrames;   // Number of valid Frame20 entries
  uint16_t crc16;         // CRC over frames + footer (excluding this field)
  uint32_t firstFrameID;  // Global frame ID of first frame in page
  uint32_t pageStartMs;   // millis() timestamp of first frame
};
static_assert(sizeof(PageFooter) == 16, "PageFooter must be exactly 16 bytes");

// =============================================================================
// SYNC PAGE FORMAT (NEW)
// =============================================================================
//
// Sync pages are stored in a dedicated region after the IMU log region.
//
// Layout:
//   - 0..(n*16-1) : SyncFrame entries (packed)
//   - remaining bytes to footerOffset: 0xFF
//   - last 16 bytes: SyncPageFooter
//
// CRC is computed over:
//   [usedBytes] + [footer bytes up to (but excluding) crc16 field]
//
#define SYNC_MAGIC 0x53594E43UL  // ASCII "SYNC"

struct SyncPageFooter {
  uint32_t magic;        // SYNC_MAGIC
  uint16_t validFrames;  // Number of valid SyncFrame entries
  uint16_t crc16;        // CRC over frames + footer (excluding this field)
  uint32_t firstSyncID;  // Monotonic sync frame ID (starts at 1)
  uint32_t pageStartMs;  // millis() timestamp of first sync frame in page
};
static_assert(sizeof(SyncPageFooter) == 16, "SyncPageFooter must be exactly 16 bytes");

// =============================================================================
// RECORDING PARAMETERS
// =============================================================================

extern const uint32_t DEFAULT_PAGES_TO_LOG;

// Frames per 256-byte page: floor(256 / 20) = 12
static constexpr uint16_t FRAMES_PER_PAGE = 12;

// =============================================================================
// RESERVED TAIL STORAGE (256 pages @ end of flash)
// =============================================================================
//
// Last 256 pages reserved for indexed storage elements.
//
#define FLASH_RESERVED_PAGES 256

// // =============================================================================
// // SYNC REGION RESERVATION (NEW)
// // =============================================================================
// //
// // We reserve a small number of pages *inside* flashRecordPages (i.e. before the
// // reserved tail storage) as a dedicated sync-log area.
// //
// // IMU pages:   [0 .. flashImuPages-1]
// // Sync pages:  [flashImuPages .. flashImuPages+SYNC_RESERVED_PAGES-1]
// //
// // NOTE:
// // - flashRecordPages is still computed in the .ino as (total - tailReserved).
// // - LoggerCore further splits that into IMU vs SYNC.
// //
// // Default 96 pages:
// //   96 pages * 16 sync frames/page = 1536 sync frames
// //   1536 frames @ 60s = 25.6 hours
// //
// #ifndef SYNC_RESERVED_PAGES
// #define SYNC_RESERVED_PAGES 96
// #endif

// =============================================================================
// GLOBAL OBJECTS (DEFINED IN .CPP FILES)
// =============================================================================

// -----------------------------------------------------------------------------
// IMU / Flash
// -----------------------------------------------------------------------------
extern ICM_20948_SPI myICM;
extern SPIFlash flash;

extern uint32_t flashCapacityBytes;
extern uint32_t flashTotalPages;
extern uint32_t flashRecordPages;  // pages excluding tail storage

// NEW: derived split
extern uint32_t flashImuPages;     // usable IMU pages (excludes sync region)
extern uint32_t flashSyncPages;    // reserved sync pages (<= SYNC_RESERVED_PAGES)

extern bool imuPresent;
extern bool flashPresent;
extern bool imuSimulated;

// Read one frame from real IMU or simulator.
bool imuReadFrame(Frame20 &out);

// =============================================================================
// FLASH LAYOUT (PUBLIC / COMPATIBILITY)
// =============================================================================
//
// These symbols are consumed by the .ino for diagnostics and policy decisions.
// They are derived values; do not write to them directly.
//

// Total pages available for IMU data (excludes sync + tail storage)
extern uint32_t flashDataPages;

// First page index of the sync region
extern uint32_t flashSyncBasePage;

// First page index of the reserved tail storage (256 pages)
extern uint32_t flashStorageBasePage;

// Compute / recompute flash layout after flash discovery
void computeFlashLayout();

// -----------------------------------------------------------------------------
// Platform pins (defined in .ino)
// -----------------------------------------------------------------------------
extern const uint8_t PIN_IMU_CS;
extern const uint8_t PIN_FLASH_CS;

// -----------------------------------------------------------------------------
// Run state / modes
// -----------------------------------------------------------------------------
enum PlaybackFormat {
  PLAYBACK_ASCII,
  PLAYBACK_BINARY
};
extern PlaybackFormat playbackFormat;

enum RunMode {
  MODE_IDLE,
  MODE_RECORDING,
  MODE_PLAYBACK
};
extern RunMode mode;

// -----------------------------------------------------------------------------
// Recording state (IMU)
// -----------------------------------------------------------------------------
extern uint32_t pageStartMs;
extern uint32_t pageFirstID;

extern uint32_t recordPageLimit;
extern uint32_t recordStartPage;

extern uint32_t frameCounter;

extern Frame20 pageFrames[FRAMES_PER_PAGE];
extern uint16_t frameIndexInPage;
extern uint32_t currentPage;  // IMU pages written (0..flashImuPages)

// -----------------------------------------------------------------------------
// Recording state (SYNC) — NEW
// -----------------------------------------------------------------------------
extern SyncFrame syncFrames[SYNC_FRAMES_PER_PAGE];
extern uint16_t  syncFrameIndexInPage;

extern uint32_t  syncCurrentPage;   // sync pages written (0..flashSyncPages)
extern uint32_t  syncFrameCounter;  // monotonic sync frame ID (starts at 0 then ++)
extern uint32_t  lastSyncMs;        // last time a sync frame was sampled (millis)

// -----------------------------------------------------------------------------
// Playback state
// -----------------------------------------------------------------------------
extern uint32_t playbackPage;
extern uint16_t playbackFrameIndex;
extern bool playbackPageLoaded;

extern uint32_t playbackPagesSeen;
extern uint32_t playbackCrcWarnings;

extern uint8_t playbackPageBuf[FLASH_PAGE_SIZE];
extern Frame20 *playbackFrames;
extern PageFooter playbackFooter;

extern uint32_t playbackPageLimit;

// -----------------------------------------------------------------------------
// Boot-time flash scan diagnostics
// -----------------------------------------------------------------------------
extern uint32_t bootPagesFound;
extern uint32_t bootValidPages;
extern uint32_t bootCorruptPages;

// -----------------------------------------------------------------------------
// Command buffer (owned by core; filled by .ino and BLE RX)
// -----------------------------------------------------------------------------
#define CMD_BUF_SIZE 320
extern char cmdBuf[CMD_BUF_SIZE];
extern uint16_t cmdLen;

// =============================================================================
// OUTPUT PLANES
// =============================================================================
void emitControl(void (*fn)(Stream &));
void emitEvent(const char *msg);

// =============================================================================
// UTILITIES
// =============================================================================
int16_t floatToQ15(float x);
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

void getMCUSerialString(char *out, size_t outLen);

void printFirmwareVersion(Stream &out);
void printMCUDeviceID(Stream &out);
void printIMUDeviceID(Stream &out);

// =============================================================================
// RESERVED TAIL STORAGE API
// =============================================================================
bool readStorageElement(uint16_t index, uint8_t *out256);
bool writeStorageElement(uint16_t index, const uint8_t *in256);

// =============================================================================
// LIVE FRAME API
// =============================================================================
void buildLiveFrame(uint8_t *dst);

void requestLiveFrameBinary(uint32_t timeoutMs = 100);
void requestLiveFrameAscii(uint32_t timeoutMs = 100);
bool liveFrameRequestPending();
void serviceLiveFrameRequests();

// =============================================================================
// FLASH MANAGEMENT (IMU)
// =============================================================================
void scanFlashOnBoot();
void reconstructFrameCounterFromFlash();

void flushPageToFlash();
bool logFrame(const Frame20 &f);

// =============================================================================
// SYNC LOGGING (NEW)
// =============================================================================
//
// serviceSyncScheduler():
//   - called opportunistically during recording
//   - samples a SyncFrame every SYNC_INTERVAL_MS
//   - buffers until a sync page fills, then flushes
//
// flushSyncPageToFlash():
//   - writes the sync page region, if available
//
// flushPendingSyncPageToFlash():
//   - writes partial sync page (used when stopping recording)
//
void serviceSyncScheduler();
void flushSyncPageToFlash();
void flushPendingSyncPageToFlash();

// =============================================================================
// SYNC PAGE MANAGEMENT
// =============================================================================

// Scan sync pages on boot and recover sync counters
void scanSyncPagesOnBoot();

// Dump sync pages in ASCII format (CLI / debugging)
void dumpSyncPagesAscii();

// =============================================================================
// PLAYBACK
// =============================================================================
void playbackTask();

void emitAsciiPageFooter(uint32_t page, const PageFooter &f, bool crcOk);
void emitBinaryPageFooter(const PageFooter &f);

// =============================================================================
// RECORDING CONTROL
// =============================================================================
void startNewRecordingSession();

// =============================================================================
// Cross-module hooks
// =============================================================================

// BLE transport (LoggerBLE.*)
bool bleConnected();
void bleWrite(const uint8_t *buf, size_t len);
void blePrintln(const char *s);
void startBLEUart();
void stopBLEUart();

// CLI dispatcher (LoggerCLI.*)
void handleCommand(const char *cmd);