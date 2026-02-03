#include "LoggerCLI.h"
#include <stdio.h>
#include <stdlib.h>  // strtoul
#include <string.h>

#define CLI_LINE_BUF_SIZE 340
static char g_cliLine[CLI_LINE_BUF_SIZE];
// Fixed buffers to avoid stack allocation during CLI operations
static uint8_t g_storePageBuf[FLASH_PAGE_SIZE];
static uint8_t g_readPageBuf[FLASH_PAGE_SIZE];

// Fixed buffer for "frame"/"aframe" command response (Frame20 + CRC16 little endian)
static uint8_t g_liveFrameOut[LIVE_FRAME_BYTES];

static void printCliLine(Stream &out) {
  out.println(g_cliLine);
}

static void emitBinaryBytes(const uint8_t *buf, size_t len) {
  // USB serial: emit binary frame, then newline for terminal sanity
  Serial.write(buf, len);
  Serial.write('\n');

  // BLE UART: emit raw binary only (no newline)
  if (bleUartEnabled && bleConnected && bleuart.notifyEnabled()) {
    bleuart.write(buf, len);
  }
}

static void emitAsciiFrame(const uint8_t *buf)
{
  const Frame20 *f = (const Frame20 *)buf;
  uint16_t crc;
  memcpy(&crc, buf + sizeof(Frame20), sizeof(crc));

  snprintf(
    g_cliLine,
    CLI_LINE_BUF_SIZE,
    "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,0x%04X",
    f->q0, f->q1, f->q2, f->q3,
    f->ax, f->ay, f->az,
    f->mx, f->my, f->mz,
    crc
  );

  emitControl(printCliLine);
}

// ===================== CLI OUTPUT =====================
//
// All user-facing text formatting and command responses
// are centralized here. This file contains no device I/O
// logic beyond invoking higher-level control functions.

void printHelpTo(Stream &out) {
  out.println("Commands:");
  out.println("  erase        (erase motion log only)");
  out.println("  erase_all    (erase entire flash)");
  out.println("  record [pages]");
  out.println("  dump [pages]");
  out.println("  bdump [pages]");
  out.println("  store <0-255> <ascii>");
  out.println("  read <0-255>");
  out.println("  status");
  out.println("  frame        (emit one live IMU frame in binary + CRC16 little endian)");
  out.println("  aframe    (emit one live IMU frame as ASCII + CRC16)");
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

  out.print("Pages present in flash: ");
  out.println(currentPage);

  out.print("Append start page: ");
  out.println(recordStartPage);

  out.print("Default record limit: ");
  out.println(DEFAULT_PAGES_TO_LOG);

  out.print("OTA: ");
  out.println(otaEnabled ? "ENABLED (advertising)" : "DISABLED");

  out.print("BLE UART: ");
  if (!bleUartEnabled) {
    out.println("DISABLED");
  } else if (!bleConnected) {
    out.println("ENABLED (not connected)");
  } else {
    out.println("ENABLED (connected)");
  }
}

// ===================== COMMAND HANDLER =====================
//
// Command parsing is intentionally simple and synchronous.
// All commands execute immediately and may change global
// mode or system state.

void handleCommand(const char *cmd) {

  // --- informational commands ---

  if (strcmp(cmd, "help") == 0) {
    emitControl(printHelpTo);
    return;
  }

// --- live frame probe (binary, async) ---
if (strcmp(cmd, "frame") == 0) {
  requestLiveFrameBinary(100);
  return;
}

// --- live frame probe (ASCII, async) ---
if (strcmp(cmd, "aframe") == 0) {
  requestLiveFrameAscii(100);
  return;
}

  if (strcmp(cmd, "status") == 0) {
    printStatus();
    return;
  }

  // --- reserved flash storage (store/read) ---

  if (strncmp(cmd, "store ", 6) == 0) {

    const char *p = cmd + 6;
    while (*p == ' ') { p++; }

    char *endp = nullptr;
    unsigned long idx = strtoul(p, &endp, 10);

    if (endp == p) {
      emitEvent("Usage: store <0-255> <ascii>");
      return;
    }

    while (*endp == ' ') { endp++; }

    if (idx >= FLASH_RESERVED_PAGES) {
      emitEvent("Index out of range (0-255)");
      return;
    }

    if (idx == 0) {
      emitEvent("store: index 0 is reserved (read-only)");
      return;
    }

    uint8_t *buf = g_storePageBuf;
    memset(buf, 0x00, FLASH_PAGE_SIZE);

    size_t n = strlen(endp);
    if (n > FLASH_PAGE_SIZE) {
      n = FLASH_PAGE_SIZE;
    }
    memcpy(buf, endp, n);

    if (!writeStorageElement((uint16_t)idx, buf)) {
      emitEvent("store: flash write failed");
      return;
    }

    snprintf(g_cliLine, sizeof(g_cliLine), "# Stored slot %lu", idx);
    emitEvent(g_cliLine);
    return;
  }

  if (strncmp(cmd, "read ", 5) == 0) {

    const char *p = cmd + 5;
    while (*p == ' ') { p++; }

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

    // slot[0] is virtual: report MCU serial directly
    if (idx == 0) {
      char serialStr[32];
      getMCUSerialString(serialStr, sizeof(serialStr));

      snprintf(g_cliLine, sizeof(g_cliLine),
               "slot[0] = %s", serialStr);
      emitControl(printCliLine);
      return;
    }

    uint8_t *buf = g_readPageBuf;
    if (!readStorageElement((uint16_t)idx, buf)) {
      emitEvent("read: flash read failed");
      return;
    }

    size_t outLen = 0;
    while (outLen < FLASH_PAGE_SIZE && buf[outLen] != 0x00 && buf[outLen] != 0xFF) {
      outLen++;
    }

    // Build a single-line response on the CONTROL plane.
    int written = snprintf(g_cliLine, sizeof(g_cliLine),
                           "slot[%lu] = ", idx);
    if (written < 0) written = 0;

    size_t room = (written < (int)sizeof(g_cliLine)) ? (sizeof(g_cliLine) - (size_t)written - 1) : 0;

    if (room > 0 && outLen > 0) {
      if (outLen > room) outLen = room;
      memcpy(g_cliLine + written, buf, outLen);
      g_cliLine[written + outLen] = 0;
    }

    emitControl(printCliLine);
    return;
  }

  // --- destructive operations ---

  if (strcmp(cmd, "erase_all") == 0) {
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
    emitEvent("# Log erase started");

    // Erase only sectors that contain recorded log data
    if (currentPage > 0) {

      // Address of last written page
      uint32_t lastAddr =
        (currentPage - 1) * FLASH_PAGE_SIZE;

      // Last sector that actually contains log data
      uint32_t lastSectorAddr =
        lastAddr & ~(uint32_t)(FLASH_SECTOR_SIZE - 1);

      for (uint32_t addr = 0;
           addr <= lastSectorAddr;
           addr += FLASH_SECTOR_SIZE) {

        flash.eraseSector(addr);
      }
    }

    currentPage = 0;
    frameIndexInPage = 0;
    frameCounter = 0;
    recordStartPage = 0;

    emitEvent("# Log erase complete");
    return;
  }

  // --- recording control ---

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

  // --- OTA / BLE control ---

  if (strcmp(cmd, "ota on") == 0) {
    startOTAAdvertising();
    return;
  }

  if (strcmp(cmd, "ota off") == 0) {
    stopOTAAdvertising();
    return;
  }

  if (strcmp(cmd, "ble on") == 0) {
    startBLEUart();
    return;
  }

  if (strcmp(cmd, "ble off") == 0) {
    stopBLEUart();
    return;
  }

  // --- binary dump ---

  uint32_t bpages = 0;
  if (sscanf(cmd, "bdump %lu", &bpages) == 1) {

    if (bpages < 1 || bpages > currentPage) {
      Serial.println("Invalid page count for bdump");
      return;
    }

    playbackPage = 0;
    playbackFrameIndex = 0;
    playbackPagesSeen = 0;
    playbackCrcWarnings = 0;
    playbackPageLoaded = false;
    playbackPageLimit = bpages;

    playbackFormat = PLAYBACK_BINARY;
    mode = MODE_PLAYBACK;
    return;
  }

  if (strcmp(cmd, "bdump") == 0) {

    if (currentPage == 0) {
      emitEvent("No pages recorded");
      return;
    }

    playbackPage = 0;
    playbackFrameIndex = 0;
    playbackPagesSeen = 0;
    playbackCrcWarnings = 0;
    playbackPageLoaded = false;
    playbackPageLimit = 0;

    playbackFormat = PLAYBACK_BINARY;
    mode = MODE_PLAYBACK;
    return;
  }

  // --- ASCII dump ---

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

  // --- fallback ---

  char msg[80];
  snprintf(msg, sizeof(msg), "Unknown command: %s", cmd);
  emitEvent(msg);
}
