#pragma once
#include <Arduino.h>

// HTTP flash export server (Wi-Fi required)

bool httpStarted();

void startHTTP();     // start HTTP server (safe to call repeatedly)
void stopHTTP();      // stop HTTP server
void serviceHTTP();   // call from loop() when OTA is enabled

// =============================================================================
// HTTP API — Motion Logger (OTA / External Power Only)
// =============================================================================
//
// This firmware exposes a minimal HTTP interface when OTA mode is enabled.
// The API is designed for fast, low-overhead extraction of raw motion logs
// and device identification. All endpoints are read-only.
//
// The HTTP server is started and stopped automatically alongside OTA.
//
// -----------------------------------------------------------------------------
// Endpoints
// -----------------------------------------------------------------------------
//
// GET /id
//   Returns the device identifier as ASCII text.
//   This is equivalent to storage slot [0] in the CLI and is derived from the
//   MCU unique ID (ESP32 eFuse MAC).
//
//   Response:
//     Content-Type: text/plain
//     Body: <ASCII device ID>
//
//   Example:
//     7F3A0012C8A4
//
// -----------------------------------------------------------------------------
//
// GET /flash
//   Streams the entire recorded motion log as a binary, chunked HTTP response.
//   Data is emitted sequentially, page-by-page, without buffering the full log
//   in RAM.
//
//   Each page is preceded by a 16-byte FlashPageHeader structure, followed by
//   exactly FLASH_PAGE_SIZE (256) bytes of raw flash page data.
//
//   Transfer encoding:
//     Content-Type: application/octet-stream
//     Transfer-Encoding: chunked
//
//   FlashPageHeader (16 bytes, little-endian):
//     uint32_t magic        // ASCII "LMTP" (0x4C4D5450)
//     uint32_t pageIndex    // Sequential page number
//     uint16_t pageSize     // Always FLASH_PAGE_SIZE (256)
//     uint16_t validFrames  // Frame count from PageFooter (if present)
//     uint16_t crc16        // PageFooter CRC (if present)
//     uint16_t flags        // bit0: footer valid, bit1: CRC valid
//
//   Notes:
//     - Pages are streamed from page 0 up to currentPage (exclusive).
//     - CRC validation is reported but not enforced.
//     - No authentication is currently applied.
//     - Intended for trusted networks / test rigs.
//
// -----------------------------------------------------------------------------
// Usage Notes
// -----------------------------------------------------------------------------
//
// - HTTP is only active while OTA is enabled.
// - Logging continues uninterrupted while streaming.
// - Clients should consume the stream incrementally.
// - The API is stable and version-independent at the binary level.
//
// =============================================================================