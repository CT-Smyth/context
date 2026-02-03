#include "LoggerCLI.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "LoggerBLE.h"
#include "LoggerOTA.h"
#include "LoggerHTTP.h"

// =============================================================================
// INTERNAL BUFFERS
// =============================================================================
//
// Fixed buffers are used intentionally to avoid stack churn and heap usage.
// All buffers are single-threaded and CLI-owned.

#define CLI_LINE_BUF_SIZE 340

static char g_cliLine[CLI_LINE_BUF_SIZE];
static uint8_t g_storePageBuf[FLASH_PAGE_SIZE];
static uint8_t g_readPageBuf[FLASH_PAGE_SIZE];

// =============================================================================
// OUTPUT HELPERS (CONTROL PLANE)
// =============================================================================
//
// All human-readable CLI output ultimately routes through emitControl()
// so it mirrors correctly to Serial and BLE.

static void printCliLine(Stream &out) {
  out.println(g_cliLine);
}

// =============================================================================
// CLI OUTPUT
// =============================================================================

void printHelpTo(Stream &out) {
  out.println("Storage 0 = chip ID, 1 is SSID, 2 is WIFI_password");
  out.println("OTA password hash is stored in flash slot 3 (ASCII MD5 hex)");
  out.println("Development OTA password = TRACE1qazXSW@ (REMOVE THIS LINE)");
  out.println("With OTA on: <ip> /id = chipID  /imu = IMU log  /sync = sync log");
  out.println("=================================================================");
  out.println("Commands:");
  out.println("  erase        (erase motion log only)");
  out.println("  erase_all    (erase entire flash)");
  out.println("  record [pages] (record IMU data and sync frames)");
  out.println("  dump [pages] (output IMU frames as ASCII)");
  out.println("  sdump     (output sync frames as ASCII)");
  out.println("  store <0-255> <ascii>");
  out.println("  read <0-255>");
  out.println("  status");
  out.println("  frame        (emit one live IMU frame in binary + CRC16 little endian)");
  out.println("  aframe       (emit one live IMU frame as ASCII + CRC16)");
  out.println("  ota on");
  out.println("  ota off");
  out.println("  ble on");
  out.println("  ble off");
}

void printPrompt() {
  Serial.println();
  Serial.print("> ");
}

void printStatus() {
  emitControl(printStatusTo);
}

void printStatusTo(Stream &out) {
  out.print("Firmware version: ");
  out.println(FW_VERSION);

  out.print("IMU: ");
  if (imuSimulated) {
    out.println("SIMULATED");
  } else {
    out.println(imuPresent ? "PRESENT" : "NOT DETECTED");
  }

  out.print("Flash: ");
  if (!flashPresent) {
    out.println("NOT DETECTED");
  } else {
    if (flash.isEmulated()) {
      out.print("PRESENT (EMULATED: internal partition '");
      out.print(FLASH_INTERNAL_PART_LABEL);
      out.print("', ");
      out.print(flash.emulatedCapacityBytes() / 1024);
      out.println(" KB)");
    } else {
      out.println("PRESENT");
    }
  }

  out.print("IMU Pages used: ");
  out.println(currentPage);

  out.print("Append start page: ");
  out.println(recordStartPage);

  out.print("Default record limit: ");
  out.println(DEFAULT_PAGES_TO_LOG);

  out.print("Sync pages used / total: ");
  out.print(syncCurrentPage);
  out.print(" / ");
  out.println(flashSyncPages);

  out.print("OTA: ");
  if (!otaStarted()) {
    out.println("OFF");
  } else if (otaHasIP()) {
    char ip[20];
    otaGetIP(ip, sizeof(ip));
    out.print("ON (");
    out.print(ip);
    out.println(")");
  } else {
    out.println("ON (no IP)");
  }

  out.print("HTTP: ");
  out.println(httpStarted() ? "ON" : "OFF");

  out.print("BLE: ");
  if (!bleStarted()) {
    out.println("OFF");
  } else if (bleConnected()) {
    out.println("CONNECTED");
  } else {
    out.println("ON (advertising)");
  }
}

// =============================================================================
// COMMAND HANDLER
// =============================================================================
//
// Parsing is synchronous and intentionally simple.
// Commands directly manipulate global state via LoggerCore APIs.

void handleCommand(const char *cmd) {

  // -------------------- Informational --------------------

  if (strcmp(cmd, "help") == 0) {
    emitControl(printHelpTo);
    return;
  }

  if (strcmp(cmd, "status") == 0) {
    printStatus();
    return;
  }

  // -------------------- BLE --------------------

  if (strcmp(cmd, "ble on") == 0) {
    startBLEUart();
    emitEvent("# BLE UART enabled");
    return;
  }

  if (strcmp(cmd, "ble off") == 0) {
    stopBLEUart();
    emitEvent("# BLE UART disabled");
    return;
  }

  // -------------------- OTA --------------------

  if (strcmp(cmd, "ota on") == 0) {
    if (mode != MODE_IDLE) {
      emitEvent("# OTA requires MODE_IDLE");
      return;
    }
    startOTA();
    return;
  }

  if (strcmp(cmd, "ota off") == 0) {
    stopOTA();
    return;
  }

  // -------------------- Live frame probes --------------------

  if (strcmp(cmd, "frame") == 0) {
    requestLiveFrameBinary(100);
    return;
  }

  if (strcmp(cmd, "aframe") == 0) {
    requestLiveFrameAscii(100);
    return;
  }

  // -------------------- Reserved flash storage --------------------

  if (strncmp(cmd, "store ", 6) == 0) {

    const char *p = cmd + 6;
    while (*p == ' ') p++;

    char *endp = nullptr;
    unsigned long idx = strtoul(p, &endp, 10);

    if (endp == p) {
      emitEvent("Usage: store <0-255> <ascii>");
      return;
    }

    while (*endp == ' ') endp++;

    if (idx >= FLASH_RESERVED_PAGES) {
      emitEvent("Index out of range (0-255)");
      return;
    }

    if (idx == 0) {
      emitEvent("store: index 0 is reserved (read-only)");
      return;
    }

    memset(g_storePageBuf, 0x00, FLASH_PAGE_SIZE);

    size_t n = strlen(endp);
    if (n > FLASH_PAGE_SIZE) n = FLASH_PAGE_SIZE;
    memcpy(g_storePageBuf, endp, n);

    if (!writeStorageElement((uint16_t)idx, g_storePageBuf)) {
      emitEvent("store: flash write failed");
      return;
    }

    snprintf(g_cliLine, sizeof(g_cliLine), "# Stored slot %lu", idx);
    emitEvent(g_cliLine);
    return;
  }

  if (strncmp(cmd, "read ", 5) == 0) {

    const char *p = cmd + 5;
    while (*p == ' ') p++;

    char *endp = nullptr;
    unsigned long idx = strtoul(p, &endp, 10);

    if (endp == p) {
      emitEvent("Usage: read <0-255>");
      return;
    }

    if (idx >= FLASH_RESERVED_PAGES) {
      emitEvent("Index out of range (0-255)");
      return;
    }

    if (idx == 0) {
      char serialStr[32];
      getMCUSerialString(serialStr, sizeof(serialStr));
      snprintf(g_cliLine, sizeof(g_cliLine), "slot[0] = %s", serialStr);
      emitControl(printCliLine);
      return;
    }

    if (!readStorageElement((uint16_t)idx, g_readPageBuf)) {
      emitEvent("read: flash read failed");
      return;
    }

    size_t outLen = 0;
    while (outLen < FLASH_PAGE_SIZE && g_readPageBuf[outLen] != 0x00 && g_readPageBuf[outLen] != 0xFF) {
      outLen++;
    }

    int written = snprintf(g_cliLine, sizeof(g_cliLine), "slot[%lu] = ", idx);
    if (written < 0) written = 0;

    size_t room =
      (written < (int)sizeof(g_cliLine))
        ? (sizeof(g_cliLine) - (size_t)written - 1)
        : 0;

    if (room > 0 && outLen > 0) {
      if (outLen > room) outLen = room;
      memcpy(g_cliLine + written, g_readPageBuf, outLen);
      g_cliLine[written + outLen] = 0;
    }

    emitControl(printCliLine);
    return;
  }

  // -------------------- Destructive operations --------------------

  if (strcmp(cmd, "erase_all") == 0) {
    if (!flashPresent) {
      emitEvent("# Flash unavailable");
      return;
    }

    emitEvent("# Flash erase started");
    flash.chipErase();

    currentPage = 0;
    frameIndexInPage = 0;
    frameCounter = 0;
    recordStartPage = 0;

    emitEvent("# Flash erase complete");
    return;
  }

  if (strcmp(cmd, "erase") == 0) {

    if (!flashPresent) {
      emitEvent("# Flash unavailable");
      return;
    }

    emitEvent("# Log erase started");

    // Erase IMU + SYNC regions (but NOT reserved tail storage)
    uint32_t pagesToErase = flashImuPages + flashSyncPages;

    if (pagesToErase > 0) {

      uint32_t lastAddr =
        (pagesToErase * FLASH_PAGE_SIZE) - 1;

      uint32_t lastSectorAddr =
        lastAddr & ~(uint32_t)(FLASH_SECTOR_SIZE - 1);

      for (uint32_t addr = 0;
           addr <= lastSectorAddr;
           addr += FLASH_SECTOR_SIZE) {
        flash.eraseSector(addr);
      }
    }

    // Reset IMU state
    currentPage = 0;
    frameIndexInPage = 0;
    frameCounter = 0;
    recordStartPage = 0;

    // Reset SYNC state
    syncCurrentPage = 0;
    syncFrameIndexInPage = 0;
    syncFrameCounter = 0;
    lastSyncMs = 0;
    memset(syncFrames, 0, sizeof(syncFrames));

    emitEvent("# Log erase complete");
    return;
  }
  // -------------------- Recording --------------------

  uint32_t pages = 0;
  if (sscanf(cmd, "record %lu", &pages) == 1) {
    startNewRecordingSession();
    recordPageLimit = pages;
    return;
  }

  if (strcmp(cmd, "record") == 0) {
    startNewRecordingSession();
    recordPageLimit = DEFAULT_PAGES_TO_LOG;
    return;
  }

  // -------------------- Playback --------------------

  if (strcmp(cmd, "sdump") == 0) {
    dumpSyncPagesAscii();
    return;
  }

  // uint32_t bpages = 0;
  // if (sscanf(cmd, "bdump %lu", &bpages) == 1) {

  //   if (bpages < 1 || bpages > currentPage) {
  //     Serial.println("Invalid page count for bdump");
  //     return;
  //   }

  //   playbackPage = 0;
  //   playbackFrameIndex = 0;
  //   playbackPagesSeen = 0;
  //   playbackCrcWarnings = 0;
  //   playbackPageLoaded = false;
  //   playbackPageLimit = bpages;

  //   playbackFormat = PLAYBACK_BINARY;
  //   mode = MODE_PLAYBACK;
  //   return;
  // }

  // if (strcmp(cmd, "bdump") == 0) {

  //   if (currentPage == 0) {
  //     emitEvent("No pages recorded");
  //     return;
  //   }

  //   playbackPage = 0;
  //   playbackFrameIndex = 0;
  //   playbackPagesSeen = 0;
  //   playbackCrcWarnings = 0;
  //   playbackPageLoaded = false;
  //   playbackPageLimit = 0;

  //   playbackFormat = PLAYBACK_BINARY;
  //   mode = MODE_PLAYBACK;
  //   return;
  // }

  uint32_t dumpPages = 0;
  if (sscanf(cmd, "dump %lu", &dumpPages) == 1) {

    if (dumpPages > currentPage) {
      dumpPages = currentPage;
    }

    playbackPage = 0;
    playbackFrameIndex = 0;
    playbackPagesSeen = 0;
    playbackCrcWarnings = 0;
    playbackPageLoaded = false;
    playbackPageLimit = dumpPages;

    playbackFormat = PLAYBACK_ASCII;
    mode = MODE_PLAYBACK;
    return;
  }

  if (strcmp(cmd, "dump") == 0) {

    playbackPage = 0;
    playbackFrameIndex = 0;
    playbackPagesSeen = 0;
    playbackCrcWarnings = 0;
    playbackPageLoaded = false;
    playbackPageLimit = 0;

    playbackFormat = PLAYBACK_ASCII;
    mode = MODE_PLAYBACK;
    return;
  }

  // -------------------- Fallback --------------------

  char msg[80];
  snprintf(msg, sizeof(msg), "Unknown command: %s", cmd);
  emitEvent(msg);
}