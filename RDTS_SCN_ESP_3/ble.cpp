#include "ble.h"

#include <Arduino.h>
#include <string.h>

// ESP32 BLE (NimBLE-based in ESP32 core)
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "RtcClock.h"

// ---------------- Configuration ----------------

// RDTS Company / Manufacturer ID (little-endian in ADV payload)
#define MANUFACTURER_ID 0xF00D

// Scan parameters
// NOTE: ESP32 BLE scan interval/window are in units of 0.625ms in the API,
// but the Arduino wrapper expects "0xNN" style values. We'll keep it simple:
// - Passive scan
// - Short window is enforced by explicit stop deadline.
#define SCAN_INTERVAL 0xA0  // ~100ms (160 * 0.625ms)
#define SCAN_WINDOW 0x50    // ~50ms  (80  * 0.625ms)

// ------------------------------------------------

// Internal state
static bool g_bleInited = false;
static bool g_scan_active = false;

static volatile bool g_rdts_valid = false;
static volatile bool g_rdts_rejected = false;

static rdts_raw_payload_t g_rdts_raw;

// Scan stop scheduling (millis-based; enforced in ble_process)
static uint32_t g_scan_stop_deadline_ms = 0;

// BLE objects (created once)
static BLEScan *g_scan = nullptr;

// Critical section for RDTS payload handoff
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ------------------------------------------------
// RDTS getter API
// ------------------------------------------------

// Debug helper (forward declaration)
static void dump_ble_mfg(const uint8_t *b, size_t len);

bool rdts_packet_available(void) {
  return g_rdts_valid;
}

bool rdts_get_latest_raw(rdts_raw_payload_t *out) {
  if (!g_rdts_valid || !out) return false;

  portENTER_CRITICAL(&g_mux);
  *out = g_rdts_raw;
  g_rdts_valid = false;
  portEXIT_CRITICAL(&g_mux);

  return true;
}

bool rdts_packet_rejected(void) {
  if (!g_rdts_rejected) return false;
  g_rdts_rejected = false;  // consume-on-read
  return true;
}

// ------------------------------------------------
// Scan callbacks
// ------------------------------------------------

class RDTSAdvertisedCB : public BLEAdvertisedDeviceCallbacks {
public:
  void onResult(BLEAdvertisedDevice advertisedDevice) override {

    // Manufacturer data present?
    if (!advertisedDevice.haveManufacturerData()) {
      return;
    }

    // Copy manufacturer data immediately
    String md = advertisedDevice.getManufacturerData();
    const size_t md_len = md.length();
    const uint8_t *b = (const uint8_t *)md.c_str();

    if (md_len < 2) {
      g_rdts_rejected = true;
      dump_ble_mfg(b, md_len);
      return;
    }

    // Company ID is little-endian
    const uint16_t cid = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
    if (cid != MANUFACTURER_ID) {
      g_rdts_rejected = true;
      dump_ble_mfg(b, md_len);
      return;
    }

    const uint8_t *rdts_payload = b + 2;
    const size_t rdts_len = md_len - 2;

    if (rdts_len == 0) {
      g_rdts_rejected = true;
      dump_ble_mfg(b, md_len);
      return;
    }

    if (rdts_len > RDTS_RAW_MAX_LEN) {
      g_rdts_rejected = true;
      dump_ble_mfg(b, md_len);
      return;
    }

    portENTER_CRITICAL(&g_mux);
    memcpy(g_rdts_raw.data, rdts_payload, rdts_len);
    g_rdts_raw.len   = (uint8_t)rdts_len;
    g_rdts_raw.rx_ms = rtc_now_ms();
    g_rdts_valid     = true;
    portEXIT_CRITICAL(&g_mux);
  }
};

static RDTSAdvertisedCB g_adv_cb;

// ------------------------------------------------
// Helpers
// ------------------------------------------------

static void dump_ble_mfg(const uint8_t *b, size_t len) {

  Serial.print("\n[RDTS][BLE] bad Packet: len=");
  Serial.print(len);
  Serial.print(" data=");

  for (size_t i = 0; i < len; i++) {
    if (b[i] < 0x10) Serial.print('0');
    Serial.print(b[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}


// ------------------------------------------------
// BLE lifecycle
// ------------------------------------------------

void ble_init(void) {
  if (g_bleInited) return;

  // Empty device name: we are scan-only; no advertising or GATT
  BLEDevice::init("");

  g_scan = BLEDevice::getScan();
  g_scan->setAdvertisedDeviceCallbacks(&g_adv_cb, false /* wantDuplicates */);

  // Passive scan for lowest RX overhead (we only need ADV payload)
  g_scan->setActiveScan(false);

  // Set scan interval/window; real duty cycle is enforced by stop deadline
  g_scan->setInterval(SCAN_INTERVAL);
  g_scan->setWindow(SCAN_WINDOW);

  g_bleInited = true;
  Serial.println("BLE INIT OK (ESP32 scan-only)");
}

bool ble_scan_start(uint32_t duration_ms) {
  if (!g_bleInited || !g_scan) return false;
  if (g_scan_active) return false;

  // Clear prior rejected flag (optional, but keeps semantics clean per scan)
  g_rdts_rejected = false;

  // Start scan in "continuous" mode and stop it ourselves after duration_ms.
  // Arduino BLE API uses seconds; using 0 keeps it running until stop().
  g_scan->start(0 /* seconds */, nullptr, false);

  g_scan_active = true;
  g_scan_stop_deadline_ms = millis() + duration_ms;
  return true;
}

bool ble_scan_active(void) {
  return g_scan_active;
}

void ble_process(void) {
  if (!g_scan_active) return;

  // Enforce millisecond scan windows here (no timers, no heap).
  const uint32_t now = millis();
  if ((int32_t)(now - g_scan_stop_deadline_ms) >= 0) {
    if (g_scan) {
      g_scan->stop();
      g_scan->clearResults();  // free internal list promptly (library heap, not ours)
    }
    g_scan_active = false;
  }
}

void ble_scan_stop(void) {
  if (!g_scan_active || !g_scan) return;

  g_scan->stop();
  g_scan->clearResults();
  g_scan_active = false;
}