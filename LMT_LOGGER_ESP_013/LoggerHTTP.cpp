#include "LoggerHTTP.h"

#include <WiFi.h>
#include <WebServer.h>

#include "LoggerCore.h"

// ============================================================================
// CONFIG
// ============================================================================
//
// HTTP is intentionally minimal and read-only.
// It is only active while OTA mode is enabled.

#define HTTP_PORT 80
#define FLASH_STREAM_MAGIC 0x4C4D5450UL  // ASCII "LMTP"
#define SYNC_STREAM_MAGIC 0x4C4D5453UL  // ASCII "LMTS"

// ============================================================================
// INTERNAL STATE
// ============================================================================
//
// The HTTP server is dynamically created and destroyed alongside OTA.
// No background threads or async tasks are used.

static WebServer *g_http = nullptr;
static bool g_httpStarted = false;

// Reused buffer for streaming flash pages.
// Avoids heap churn and large stack allocations.
static uint8_t g_pageBuf[FLASH_PAGE_SIZE];


// ============================================================================
// PAGE HEADER (sent before each flash page)
// ============================================================================
//
// Each flash page is preceded by a fixed-size header so clients can:
//   - identify page boundaries
//   - validate page integrity
//   - stream without prior knowledge of log size
//
// Header is little-endian and binary-stable.

struct FlashPageHeader {
  uint32_t magic;        // "LMTP"
  uint32_t pageIndex;    // sequential page number
  uint16_t pageSize;     // always FLASH_PAGE_SIZE (256)
  uint16_t validFrames;  // from PageFooter (if present)
  uint16_t crc16;        // PageFooter CRC16 (if present)
  uint16_t flags;        // bit0: footer valid, bit1: CRC valid
};

static_assert(sizeof(FlashPageHeader) == 16,
              "FlashPageHeader must be 16 bytes");



struct SyncPageHeader {
  uint32_t magic;        // "LMTS"
  uint32_t pageIndex;    // sync page index (0-based)
  uint16_t pageSize;     // FLASH_PAGE_SIZE
  uint16_t validFrames;  // from SyncPageFooter
  uint16_t crc16;        // footer CRC
  uint16_t flags;        // bit0: footer valid, bit1: CRC valid
};

static_assert(sizeof(SyncPageHeader) == 16,
              "SyncPageHeader must be 16 bytes");

// ============================================================================
// HTTP HANDLERS
// ============================================================================

// Root endpoint: redirect browsers to Trace Dynamics site.
// This avoids exposing a directory listing or empty page.
static void handleRootRedirect() {
  g_http->sendHeader("Location", "https://tracedynamics.ai", true);
  g_http->send(302, "text/plain", "Redirecting to Trace Dynamics");
}

// Stream the entire recorded flash log as a chunked HTTP response.
//
// Behavior:
//   - Pages are streamed sequentially from page 0 to currentPage (exclusive)
//   - Logging may continue concurrently
//   - No attempt is made to lock or snapshot flash contents
//   - CRC validity is reported in headers but not enforced
//
// This endpoint is intended for trusted networks and test rigs.
static void handleFlashStream() {

  WiFiClient client = g_http->client();

  // Manual HTTP response (chunked transfer)
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/octet-stream");
  client.println("Transfer-Encoding: chunked");
  client.println("Connection: close");
  client.println();

  // Stream pages one-by-one
  for (uint32_t page = 0; page < currentPage; page++) {

    const uint32_t addr = page * FLASH_PAGE_SIZE;
    if (!flash.readData(addr, g_pageBuf, FLASH_PAGE_SIZE)) {
      // Abort stream on read failure
      break;
    }

    FlashPageHeader hdr = {};
    hdr.magic = FLASH_STREAM_MAGIC;
    hdr.pageIndex = page;
    hdr.pageSize = FLASH_PAGE_SIZE;
    hdr.flags = 0;

    // Attempt to parse page footer (if present)
    const uint16_t footerOffset =
      FLASH_PAGE_SIZE - sizeof(PageFooter);

    PageFooter footer;
    memcpy(&footer,
           g_pageBuf + footerOffset,
           sizeof(PageFooter));

    if (footer.magic == PAGE_MAGIC &&
        footer.validFrames <= FRAMES_PER_PAGE) {

      hdr.flags |= 0x0001;  // footer valid
      hdr.validFrames = footer.validFrames;
      hdr.crc16 = footer.crc16;

      const uint16_t usedBytes =
        footer.validFrames * sizeof(Frame20);
      const uint16_t crcLen =
        usedBytes + offsetof(PageFooter, crc16);

      if (crc16_ccitt(g_pageBuf, crcLen) == footer.crc16) {
        hdr.flags |= 0x0002;  // CRC OK
      }
    }

    // ---- chunk: page header ----
    client.printf("%X\r\n", sizeof(hdr));
    client.write((const uint8_t *)&hdr, sizeof(hdr));
    client.print("\r\n");

    // ---- chunk: raw page data ----
    client.printf("%X\r\n", FLASH_PAGE_SIZE);
    client.write(g_pageBuf, FLASH_PAGE_SIZE);
    client.print("\r\n");
  }

  // Final zero-length chunk terminates the stream
  client.print("0\r\n\r\n");
  client.stop();
}

static void handleSyncStream() {

  if (flashSyncPages == 0 || syncCurrentPage == 0) {
    g_http->send(204, "text/plain", "No sync data");
    return;
  }

  WiFiClient client = g_http->client();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/octet-stream");
  client.println("Transfer-Encoding: chunked");
  client.println("Connection: close");
  client.println();

  for (uint32_t i = 0; i < syncCurrentPage; i++) {

    const uint32_t page = flashSyncBasePage + i;
    const uint32_t addr = page * FLASH_PAGE_SIZE;

    if (!flash.readData(addr, g_pageBuf, FLASH_PAGE_SIZE)) {
      break;
    }

    SyncPageHeader hdr = {};
    hdr.magic     = SYNC_STREAM_MAGIC;
    hdr.pageIndex = i;
    hdr.pageSize  = FLASH_PAGE_SIZE;
    hdr.flags     = 0;

    const uint16_t footerOffset =
      FLASH_PAGE_SIZE - sizeof(SyncPageFooter);

    SyncPageFooter footer;
    memcpy(&footer,
           g_pageBuf + footerOffset,
           sizeof(SyncPageFooter));

    if (footer.magic == SYNC_MAGIC &&
        footer.validFrames <= SYNC_FRAMES_PER_PAGE) {

      hdr.flags |= 0x0001;
      hdr.validFrames = footer.validFrames;
      hdr.crc16 = footer.crc16;

      const uint16_t usedBytes =
        footer.validFrames * sizeof(SyncFrame);
      const uint16_t crcLen =
        usedBytes + offsetof(SyncPageFooter, crc16);

      if (crc16_ccitt(g_pageBuf, crcLen) == footer.crc16) {
        hdr.flags |= 0x0002;
      }
    }

    // ---- chunk: header ----
    client.printf("%X\r\n", sizeof(hdr));
    client.write((const uint8_t *)&hdr, sizeof(hdr));
    client.print("\r\n");

    // ---- chunk: raw page ----
    client.printf("%X\r\n", FLASH_PAGE_SIZE);
    client.write(g_pageBuf, FLASH_PAGE_SIZE);
    client.print("\r\n");
  }

  client.print("0\r\n\r\n");
  client.stop();
}

// ============================================================================
// PUBLIC API
// ============================================================================

bool httpStarted() {
  return g_httpStarted;
}

// Start HTTP server.
//
// Safe to call repeatedly.
// Intended to be called alongside OTA startup.
void startHTTP() {
  if (g_httpStarted) return;

  g_http = new WebServer(HTTP_PORT);

  // Binary flash dump endpoints
  g_http->on("/imu", HTTP_GET, handleFlashStream);
  g_http->on("/sync", HTTP_GET, handleSyncStream);

  // Root redirect
  g_http->on("/", HTTP_GET, handleRootRedirect);

  // Device ID endpoint (equivalent to storage slot 0)
  g_http->on("/id", HTTP_GET, []() {
    char id[32];
    getMCUSerialString(id, sizeof(id));

    g_http->send(
      200,
      "text/plain",
      id);
  });

  // Catch-all redirect for unknown paths
  g_http->onNotFound([]() {
    g_http->sendHeader("Location", "https://tracedynamics.ai", true);
    g_http->send(302, "text/plain", "Redirecting to Trace Dynamics");
  });

  g_http->begin();
  g_httpStarted = true;

  Serial.println("# HTTP: flash streaming enabled");
}

// Stop HTTP server and release resources.
void stopHTTP() {
  if (!g_httpStarted) return;

  g_http->stop();
  delete g_http;
  g_http = nullptr;

  g_httpStarted = false;
  Serial.println("# HTTP: stopped");
}

// Service HTTP client requests.
//
// Must be called periodically from loop() while HTTP is active.
void serviceHTTP() {
  if (!g_httpStarted || !g_http) return;
  g_http->handleClient();
}