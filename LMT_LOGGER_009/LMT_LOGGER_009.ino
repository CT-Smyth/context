#include "LoggerCore.h"
#include "LoggerCLI.h"

// ===================== PIN DEFINITIONS =====================
// These pins intentionally live in the .ino to give them
// internal linkage. Do not move them unless you also
// update LoggerCore.cpp to match.

static const uint8_t PIN_IMU_CS = D5;    // IMU CS pin
static const uint8_t PIN_FLASH_CS = D2;  // Flash CS pin

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);
  delay(500); //wait for uart

  Serial.println("ICM-20948 SPI DMP Quat9 + Raw Accel + Raw Mag + Flash Logger");
  printFirmwareVersion(Serial);

  // --- SPI / GPIO init ---
  SPI.begin();

  pinMode(PIN_IMU_CS, OUTPUT);
  pinMode(PIN_FLASH_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
  digitalWrite(PIN_FLASH_CS, HIGH);

  // --- Flash init ---
  flash.begin();

  uint8_t man, type, cap;
  flash.readID(man, type, cap);

  // JEDEC encoding: capacity = 2^cap bytes
  flashCapacityBytes = 1UL << cap;
  flashTotalPages = flashCapacityBytes / FLASH_PAGE_SIZE;

  // Exclude reserved tail pages from motion logging
  if (flashTotalPages > FLASH_RESERVED_PAGES) {
    flashRecordPages = flashTotalPages - FLASH_RESERVED_PAGES;
  } else {
    flashRecordPages = 0;
  }

  // Discover previously recorded pages
  scanFlashOnBoot();
  reconstructFrameCounterFromFlash();


  // --- Flash scan report ---
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

  Serial.print("Flash pages (total): ");
  Serial.println(flashTotalPages);
  Serial.print("Flash pages (recordable): ");
  Serial.println(flashRecordPages);
#endif

  {
    char msg[96];
    snprintf(msg, sizeof(msg),
             "Flash scan: %lu pages found, %lu valid, %lu corrupt",
             (unsigned long)bootPagesFound,
             (unsigned long)bootValidPages,
             (unsigned long)bootCorruptPages);
    emitEvent(msg);
  }

#if VERBOSE_LOG
  Serial.print("Flash JEDEC ID: ");
  Serial.print(man, HEX);
  Serial.print(" ");
  Serial.print(type, HEX);
  Serial.print(" ");
  Serial.println(cap, HEX);
#endif

#if ERASE_FLASH_AT_START
  Serial.println("Erasing entire flash chip...");
  flash.chipErase();
  Serial.println("Flash erase complete.");
#endif

  // --- IMU init ---
  ICM_20948_Status_e status = myICM.begin(PIN_IMU_CS, SPI);
  if (status != ICM_20948_Stat_Ok) {
    Serial.println("IMU init failed.");
    while (1) {}
  }

  myICM.initializeDMP();
  myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);
  myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 5);

  myICM.enableFIFO();
  myICM.enableDMP();
  myICM.resetDMP();
  myICM.resetFIFO();
  myICM.startupMagnetometer();

  Serial.println("IMU ready.");

  printMCUDeviceID(Serial);
  printIMUDeviceID(Serial);

#if BLEonstartup
  startBLEUart();
  Serial.println("# BLE UART enabled at startup");
#endif

  Serial.println("Type 'help' for commands.");
  printPrompt();
}

// ===================== MAIN LOOP =====================

void loop() {

  // ===================== MODE_IDLE =====================
  if (mode == MODE_IDLE) {

    // --- USB Serial input ---
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

    // --- BLE UART input ---
    if (bleUartEnabled && bleConnected) {
      while (bleuart.available()) {
        char c = bleuart.read();

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
    }

    // Service any pending live-frame request (non-blocking)
    serviceLiveFrameRequests();

    return;
  }

  // ===================== MODE_PLAYBACK =====================
  if (mode == MODE_PLAYBACK) {
    playbackTask();
    return;
  }

  // ===================== MODE_RECORDING =====================

  icm_20948_DMP_data_t dmpData;
  myICM.readDMPdataFromFIFO(&dmpData);

  if ((myICM.status == ICM_20948_Stat_Ok || myICM.status == ICM_20948_Stat_FIFOMoreDataAvail) && (dmpData.header & DMP_header_bitmap_Quat9)) {

    Frame20 f;

    // --- Quaternion reconstruction ---
    float fq1 = (float)dmpData.Quat9.Data.Q1 / 1073741824.0f;
    float fq2 = (float)dmpData.Quat9.Data.Q2 / 1073741824.0f;
    float fq3 = (float)dmpData.Quat9.Data.Q3 / 1073741824.0f;

    float mag2 = fq1 * fq1 + fq2 * fq2 + fq3 * fq3;
    float t = 1.0f - mag2;
    if (t < 0) t = 0;
    if (t > 1) t = 1;

    float fq0 = sqrtf(t);

    f.q0 = floatToQ15(fq0);
    f.q1 = floatToQ15(fq1);
    f.q2 = floatToQ15(fq2);
    f.q3 = floatToQ15(fq3);

    // --- Raw accel + mag ---
    myICM.getAGMT();
    f.ax = myICM.agmt.acc.axes.x;
    f.ay = myICM.agmt.acc.axes.y;
    f.az = myICM.agmt.acc.axes.z;
    f.mx = myICM.agmt.mag.axes.x;
    f.my = myICM.agmt.mag.axes.y;
    f.mz = myICM.agmt.mag.axes.z;

    // Allocate page metadata only if this frame will be accepted
    if (frameIndexInPage == 0) {
      pageStartMs = millis();
      pageFirstID = frameCounter + 1;
    }

    // logFrame() may terminate recording
    if (!logFrame(f)) {
      return;
    }

    // Frame counter reflects accepted frames only
    frameCounter++;

    // --- Live output ---
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
  }
}
