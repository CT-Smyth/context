#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "LoggerCLI.h"
#include "LoggerCore.h"

// ============================================================================
// BLE UART (Nordic-style)
// ============================================================================

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// ============================================================================
// INTERNAL STATE
// ============================================================================

static BLECharacteristic *g_txChar   = nullptr;
static BLEServer         *g_server   = nullptr;

static bool g_bleStarted   = false;   // stack initialized
static bool g_bleConnected = false;   // active connection
static bool g_bleEnabled   = false;   // advertising / connectability allowed

// ============================================================================
// SERVER CALLBACKS
// ============================================================================

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    g_bleConnected = true;
  }

  void onDisconnect(BLEServer *) override {
    g_bleConnected = false;

    // IMPORTANT:
    // Only resume advertising if BLE is explicitly enabled
    if (g_bleEnabled) {
      BLEDevice::startAdvertising();
    }
  }
};

// ============================================================================
// RX CALLBACK
// ============================================================================

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    if (!g_bleConnected) return;

    String v = c->getValue();

    for (size_t i = 0; i < v.length(); i++) {
      char ch = v[i];

      if (ch == '\r' || ch == '\n') {
        cmdBuf[cmdLen] = 0;
        if (cmdLen > 0) {
          handleCommand(cmdBuf);
          cmdLen = 0;
          if (!liveFrameRequestPending()) {
            printPrompt();
          }
        }
      } else if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = ch;
      }
    }
  }
};

// ============================================================================
// PUBLIC STATE QUERIES
// ============================================================================

bool bleStarted() {
  return g_bleStarted;
}

bool bleConnected() {
  return g_bleConnected;
}

// ============================================================================
// TX HELPERS
// ============================================================================

void bleWrite(const uint8_t *buf, size_t len) {
  if (!g_bleConnected || !g_txChar) return;
  g_txChar->setValue(buf, len);
  g_txChar->notify();
}

void blePrintln(const char *s) {
  if (!g_bleConnected || !g_txChar) return;

  const char *p = s;
  while (*p) {
    const char *e = strchr(p, '\n');
    size_t len = e ? (size_t)(e - p) : strlen(p);

    if (len > 0) {
      g_txChar->setValue((uint8_t *)p, len);
      g_txChar->notify();
      delay(2);
    }
    if (!e) break;
    p = e + 1;
  }
}

// ============================================================================
// BLE UART CONTROL
// ============================================================================

void startBLEUart() {

  // One-time stack init
  if (!g_bleStarted) {

    BLEDevice::init("LMT-LOGGER");
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    g_server = BLEDevice::createServer();
    g_server->setCallbacks(new ServerCB());

    BLEService *svc = g_server->createService(NUS_SERVICE_UUID);

    g_txChar = svc->createCharacteristic(
      NUS_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
    );
    g_txChar->addDescriptor(new BLE2902());

    BLECharacteristic *rxChar = svc->createCharacteristic(
      NUS_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR
    );
    rxChar->setCallbacks(new RxCB());

    svc->start();

    g_bleStarted = true;
  }

  // Enable connectability
  g_bleEnabled = true;

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->start();
}

void stopBLEUart() {
  if (!g_bleStarted) return;

  // Disable reconnectability FIRST
  g_bleEnabled = false;

  // Stop advertising
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  if (adv) {
    adv->stop();
  }

  // Disconnect active client (onDisconnect will NOT restart advertising)
  if (g_server && g_bleConnected) {
    g_server->disconnect(g_server->getConnId());
  }

  g_bleConnected = false;
}