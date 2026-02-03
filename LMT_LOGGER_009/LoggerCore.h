#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "ICM_20948.h"
#include "SPIFlash.h"
#include <bluefruit.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

#define FW_VERSION "0.09"

// Enable BLE UART automatically at boot
#define BLEonstartup 1

// Logging / playback configuration
#define VERBOSE_LOG 1
// #define ERASE_FLASH_AT_START 1

// =============================================================================
// PAGE FOOTER FORMAT
// =============================================================================
//
// Each flash page ends with a 16-byte footer describing the page contents.
// The footer is written once per page and is CRC-protected.
//

#define PAGE_MAGIC 0x50414745UL  // ASCII "PAGE"

struct PageFooter {
  uint32_t magic;         // PAGE_MAGIC
  uint16_t validFrames;   // Number of valid Frame20 entries
  uint16_t crc16;         // CRC over frames + footer (excluding crc field)
  uint32_t firstFrameID;  // Global frame ID of first frame in page
  uint32_t pageStartMs;   // millis() timestamp of first frame
};

static_assert(sizeof(PageFooter) == 16, "PageFooter must be exactly 16 bytes");

// =============================================================================
// RECORDING PARAMETERS
// =============================================================================

extern const uint32_t DEFAULT_PAGES_TO_LOG;

// Frames per page: 256 / 20 = 12
static constexpr uint16_t FRAMES_PER_PAGE = 12;

// =============================================================================
// FRAME FORMAT
// =============================================================================
//
// On-flash and playback frame format (20 bytes, packed, little-endian).
//

struct Frame20 {
  int16_t q0, q1, q2, q3;  // Quaternion (Q15)
  int16_t ax, ay, az;      // Accelerometer (raw)
  int16_t mx, my, mz;      // Magnetometer (raw)
};

// Live "frame" command response = Frame20 + CRC16(Frame20)
static constexpr size_t LIVE_FRAME_BYTES = sizeof(Frame20) + sizeof(uint16_t);

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

// -----------------------------------------------------------------------------
// Reserved tail storage region (256 pages @ end of flash)
// -----------------------------------------------------------------------------
//
// This region is excluded from motion logging and is addressed by the CLI as
// 256 elements of 256 bytes each.
//
// NOTE: element[0] is reserved for the MCU serial number.

#define FLASH_RESERVED_PAGES 256

// Total pages available for motion logging (excludes reserved tail region)
extern uint32_t flashRecordPages;

// 256-byte indexed storage (0..255) living in reserved tail pages
bool readStorageElement(uint16_t index, uint8_t *out256);

// Build a single live IMU frame (same format as stored frames, incl CRC)
void buildLiveFrame(uint8_t *dst);

// Async live-frame request model (non-blocking, SoftDevice-safe)
void requestLiveFrameBinary(uint32_t timeoutMs = 100);
void requestLiveFrameAscii(uint32_t timeoutMs = 100);
bool liveFrameRequestPending();
void serviceLiveFrameRequests();
bool writeStorageElement(uint16_t index, const uint8_t *in256);

// -----------------------------------------------------------------------------
// BLE / OTA
// -----------------------------------------------------------------------------
extern BLEUart bleuart;

extern bool bleUartEnabled;
extern bool bleConnected;
extern bool bleStackStarted;
extern bool otaEnabled;

extern BLEDfu bledfu;
extern BLEDis bledis;

// -----------------------------------------------------------------------------
// Playback / Run State
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
// Recording State
// -----------------------------------------------------------------------------
extern uint32_t pageStartMs;
extern uint32_t pageFirstID;

extern uint32_t recordPageLimit;
extern uint32_t recordStartPage;

extern uint32_t frameCounter;

// Sized in .cpp as: Frame20 pageFrames[FRAMES_PER_PAGE]
extern Frame20 pageFrames[FRAMES_PER_PAGE];
extern uint16_t frameIndexInPage;
extern uint32_t currentPage;

// -----------------------------------------------------------------------------
// Playback State
// -----------------------------------------------------------------------------
extern uint32_t playbackPage;
extern uint16_t playbackFrameIndex;
extern bool playbackPageLoaded;

extern uint32_t playbackPagesSeen;
extern uint32_t playbackCrcWarnings;

// Sized in .cpp as: uint8_t playbackPageBuf[FLASH_PAGE_SIZE]
extern uint8_t playbackPageBuf[FLASH_PAGE_SIZE];
extern Frame20 *playbackFrames;
extern PageFooter playbackFooter;

extern uint32_t playbackPageLimit;

// -----------------------------------------------------------------------------
// Boot-time Flash Scan Diagnostics
// -----------------------------------------------------------------------------
extern uint32_t bootPagesFound;
extern uint32_t bootValidPages;
extern uint32_t bootCorruptPages;

// -----------------------------------------------------------------------------
// Command Buffer
// -----------------------------------------------------------------------------
#define CMD_BUF_SIZE 320
extern char cmdBuf[CMD_BUF_SIZE];
extern uint16_t cmdLen;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================

// -----------------------------------------------------------------------------
// Output helpers
// -----------------------------------------------------------------------------
void emitControl(void (*fn)(Stream &));
void emitEvent(const char *msg);

void printHelpTo(Stream &out);
void printPrompt();
void printStatus();
void printStatusTo(Stream &out);

// -----------------------------------------------------------------------------
// BLE / OTA
// -----------------------------------------------------------------------------
void refreshAdvertising();
void startBleStackOnce();

void startOTAAdvertising();
void stopOTAAdvertising();

void startBLEUart();
void stopBLEUart();

void bleConnectCallback(uint16_t conn_handle);
void bleDisconnectCallback(uint16_t conn_handle, uint8_t reason);

// -----------------------------------------------------------------------------
// Utilities
// -----------------------------------------------------------------------------
int16_t floatToQ15(float x);
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

void getMCUSerialString(char *out, size_t outLen);

void printFirmwareVersion(Stream &out);
void printMCUDeviceID(Stream &out);
void printIMUDeviceID(Stream &out);

// -----------------------------------------------------------------------------
// Flash management
// -----------------------------------------------------------------------------
void scanFlashOnBoot();
void reconstructFrameCounterFromFlash();

void flushPageToFlash();
bool logFrame(const Frame20 &f);

// -----------------------------------------------------------------------------
// Playback
// -----------------------------------------------------------------------------
void playbackTask();

void emitAsciiPageFooter(uint32_t page,
                         const PageFooter &f,
                         bool crcOk);

void emitBinaryPageFooter(const PageFooter &f);

// -----------------------------------------------------------------------------
// Recording control
// -----------------------------------------------------------------------------
void startNewRecordingSession();

// -----------------------------------------------------------------------------
// CLI
// -----------------------------------------------------------------------------
void handleCommand(const char *cmd);

