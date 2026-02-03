#include "LoggerCore.h"
#include "LoggerCLI.h"
#include "LoggerBLE.h"
#include "LoggerOTA.h"
#include "LoggerHTTP.h"

// ============================================================================
// HARDWARE PIN MAP (ESP32-C3)
// ============================================================================
//
// Explicit numeric GPIOs only (no Dx aliases on ESP32).
// This file is the *only* place where physical pin mappings live.

#define CS_IMU 7  // IMU chip select
#define CS_FL 2   // External flash chip select

#define PIN_SCK 4
#define PIN_MOSI 6
#define PIN_MISO 5

#define PIN_INT 20    // IMU interrupt
#define PIN_FSYNC 21  // IMU FSYNC

#define PIN_BSENSE 3    // Battery sense (ADC)
#define PIN_CHG_EN 8    // Charger enable (shared with LED; external pull-up)
#define PIN_LED 8       // Same physical pin
#define PIN_CHG_STAT 9  // Charger status input
#define PIN_SYS_EN 10   // System enable

// XTAL pins (GPIO0, GPIO1) — declared but intentionally unused
// #define PIN_XTAL_P 0
// #define PIN_XTAL_N 1

// Internal aliases consumed by LoggerCore
const uint8_t PIN_IMU_CS = CS_IMU;
const uint8_t PIN_FLASH_CS = CS_FL;

// ============================================================================
// RECORDING POLICY
// ============================================================================
//
// The .ino owns *policy*, not mechanics.
// Core implements logging; this file decides *when* to log.

static constexpr uint32_t RECORD_INTERVAL_MS = 100;
static uint32_t lastRecordMs = 0;

// ============================================================================
// SETUP
// ============================================================================
//
// Responsibilities:
//   - Hardware bring-up
//   - SPI + GPIO initialization
//   - Device discovery (flash, IMU)
//   - Boot-time diagnostics
//   - Initial mode selection
//
// No runtime logic should live here.

void setup() {
  Serial.begin(115200);

  // LED is shared with charger enable on this board.
  // External pull-up ensures safe default during reset.
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);  // brief override during boot
  delay(1000);
  digitalWrite(PIN_LED, HIGH);  // release to external pull-up

  Serial.println("ICM-20948 SPI DMP Quat9 + Raw Accel + Raw Mag + Flash Logger");
  printFirmwareVersion(Serial);

  // --------------------------------------------------------------------------
  // SPI + CHIP SELECT INITIALIZATION
  // --------------------------------------------------------------------------

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  pinMode(PIN_IMU_CS, OUTPUT);
  pinMode(PIN_FLASH_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
  digitalWrite(PIN_FLASH_CS, HIGH);

  // --------------------------------------------------------------------------
  // FLASH DISCOVERY
  // --------------------------------------------------------------------------
  //
  // Flash presence gates all logging functionality.
  // The driver may transparently fall back to internal partition emulation.

  flashPresent = false;

  uint8_t man = 0, type = 0, cap = 0;

  if (flash.begin()) {
    flash.readID(man, type, cap);

    if (man != 0x00 && man != 0xFF) {
      flashPresent = true;

      flashCapacityBytes = 1UL << cap;
      flashTotalPages = flashCapacityBytes / FLASH_PAGE_SIZE;

      if (flashTotalPages > FLASH_RESERVED_PAGES) {
        flashRecordPages = flashTotalPages - FLASH_RESERVED_PAGES;
      } else {
        flashRecordPages = 0;
      }

      computeFlashLayout();

      scanFlashOnBoot();
      scanSyncPagesOnBoot();
      reconstructFrameCounterFromFlash();
    }
  }

  if (!flashPresent) {
    emitEvent("# Flash not detected – logging disabled");
  }

  // Boot scan diagnostics
  Serial.println("Flash scan complete:");
  Serial.print("  Pages found:   ");
  Serial.println(bootPagesFound);
  Serial.print("  Valid pages:   ");
  Serial.println(bootValidPages);
  Serial.print("  Corrupt pages: ");
  Serial.println(bootCorruptPages);

#if VERBOSE_LOG
  Serial.print("Flash capacity: ");
  Serial.print(flashCapacityBytes / 1024);
  Serial.println(" KB");

  Serial.print("Flash pages (data region): ");
  Serial.println(flashDataPages);

  Serial.print("Flash pages (IMU region):  ");
  Serial.println(flashImuPages);

  Serial.print("Flash pages (SYNC region): ");
  Serial.println(flashSyncPages);

  Serial.print("SYNC base page:            ");
  Serial.println(flashSyncBasePage);

  Serial.print("Storage base page:         ");
  Serial.println(flashStorageBasePage);
#endif

  // --------------------------------------------------------------------------
  // IMU INITIALIZATION
  // --------------------------------------------------------------------------
  //
  // If the IMU is absent, the system falls back to a deterministic simulator.

  imuPresent = false;

  if (myICM.begin(PIN_IMU_CS, SPI) == ICM_20948_Stat_Ok) {

    myICM.initializeDMP();
    myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);
    myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 5);

    myICM.enableFIFO();
    myICM.enableDMP();
    myICM.resetDMP();
    myICM.resetFIFO();
    myICM.startupMagnetometer();

    imuPresent = true;
    Serial.println("IMU ready.");

  } else {
    imuSimulated = true;
    emitEvent("# IMU not detected – using simulator");
  }

#if BLEonstartup
  startBLEUart();
  Serial.println("# BLE UART started");
#endif

  printIMUDeviceID(Serial);
  Serial.println("\n===========SYSTEM STATUS=============");

  printStatus(); 

  Serial.println("Type 'help' for commands.");
  printPrompt();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
//
// This is a *mode scheduler*.
// Each mode is serviced independently and non-overlapping.

void loop() {

  // ===================== MODE_IDLE =====================
  //
  // CLI, OTA, HTTP, BLE RX, live probes

  if (mode == MODE_IDLE) {

    serviceLiveFrameRequests();

    if (otaStarted()) {
      serviceOTA();
      serviceHTTP();
    }

    // USB Serial CLI input
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\r' || c == '\n') {
        cmdBuf[cmdLen] = 0;
        if (cmdLen > 0) {
          handleCommand(cmdBuf);
          cmdLen = 0;
          if (!liveFrameRequestPending()) {
            printPrompt();
          }
        }
      } else if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = c;
      }
    }

    serviceLiveFrameRequests();
    return;
  }

  // ===================== MODE_PLAYBACK =====================
  //
  // Non-blocking page/frame streaming

  if (mode == MODE_PLAYBACK) {
    playbackTask();
    return;
  }

  // ===================== MODE_RECORDING =====================
  //
  // Fixed-rate acquisition controlled by policy here,
  // mechanics implemented in LoggerCore.

  if (mode == MODE_RECORDING) {

    const uint32_t now = millis();
    if ((uint32_t)(now - lastRecordMs) < RECORD_INTERVAL_MS) {
      return;
    }

    Frame20 f;
    if (!imuReadFrame(f)) {
      return;
    }

    lastRecordMs = now;

    if (frameIndexInPage == 0) {
      pageStartMs = now;
      pageFirstID = frameCounter + 1;
    }

    if (!logFrame(f)) {
      return;
    }

    frameCounter++;

    // Live USB debug output (not part of recorded data)
    Serial.print(frameCounter);
    Serial.print(" ");
    Serial.print(f.q0);
    Serial.print(" ");
    Serial.print(f.q1);
    Serial.print(" ");
    Serial.print(f.q2);
    Serial.print(" ");
    Serial.print(f.q3);
    Serial.print(" ");
    Serial.print(f.ax);
    Serial.print(" ");
    Serial.print(f.ay);
    Serial.print(" ");
    Serial.print(f.az);
    Serial.print(" ");
    Serial.print(f.mx);
    Serial.print(" ");
    Serial.print(f.my);
    Serial.print(" ");
    Serial.println(f.mz);

    return;
  }
}