#include "LoggerCore.h"

// ===================== BLE / OTA GLOBALS =====================
//
// These symbols are declared extern in LoggerCore.h and must be
// defined in exactly ONE translation unit. This file owns all
// BLE-related state and behavior.

BLEUart bleuart;

bool bleUartEnabled = false;
bool bleConnected = false;
bool bleStackStarted = false;  // Enforces single initialization
bool otaEnabled = false;

BLEDfu bledfu;
BLEDis bledis;

// ===================== BLE ADVERTISING =====================
//
// BLE services are started once and remain registered forever.
// This function ONLY controls which services are advertised.
//
// Safe operations:
//   - Start / stop advertising
//   - Add or remove services from advertising payload
//
// Unsafe operations (DO NOT DO):
//   - Calling Bluefruit.begin() here
//   - Calling bleuart.begin() or bledfu.begin() here
//
void refreshAdvertising() {

  // If nothing is enabled, stop advertising entirely
  if (!bleUartEnabled && !otaEnabled) {
    Bluefruit.Advertising.stop();
    return;
  }

  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(
    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Device Information Service is always safe to advertise
  Bluefruit.Advertising.addService(bledis);

  if (bleUartEnabled) {
    Bluefruit.Advertising.addService(bleuart);
  }

  if (otaEnabled) {
    Bluefruit.Advertising.addService(bledfu);
  }

  Bluefruit.Advertising.addName();
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.start(0);
}

// ===================== BLE STACK LIFETIME =====================
//
// Bluefruit.begin() and service .begin() calls MUST occur
// exactly once for the lifetime of the firmware.
//
void startBleStackOnce() {
  if (bleStackStarted) {
    return;
  }

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();

  Bluefruit.setName("TRACE-LMT");
  Bluefruit.setTxPower(4);
  Bluefruit.Periph.setConnectCallback(bleConnectCallback);
  Bluefruit.Periph.setDisconnectCallback(bleDisconnectCallback);

  // --- Device Information Service ---
  bledis.setManufacturer("Trace_Dynamics");
  bledis.setModel("LMT Logger");
  bledis.setFirmwareRev(FW_VERSION);

  char serialStr[17];
  getMCUSerialString(serialStr, sizeof(serialStr));
  bledis.setSerialNum(serialStr);

  // --- Start services (ONCE) ---
  bledis.begin();
  bleuart.begin();
  bledfu.begin();

  // Helps some BLE clients discover the device
  Bluefruit.ScanResponse.addName();

  bleStackStarted = true;
}

// ===================== OTA CONTROL =====================

void startOTAAdvertising() {
  if (otaEnabled) {
    return;
  }

  Serial.println("Enabling OTA (BLE DFU)...");
  startBleStackOnce();

  otaEnabled = true;
  refreshAdvertising();

  emitEvent("# OTA enabled (DFU advertising)");
  emitEvent("Reconnect required to perform update.");
}

void stopOTAAdvertising() {
  if (!otaEnabled) {
    return;
  }

  emitEvent("Disabling OTA (BLE DFU)...");
  otaEnabled = false;

  refreshAdvertising();

  emitEvent("OTA disabled.");
  emitEvent("Reconnect required for changes to take effect.");
}

// ===================== BLE UART CONTROL =====================

void startBLEUart() {
  if (bleUartEnabled) {
    return;
  }

  Serial.println("Enabling BLE UART...");
  startBleStackOnce();

  bleUartEnabled = true;
  refreshAdvertising();

  Serial.println("BLE UART enabled (advertising).");
}

void stopBLEUart() {
  if (!bleUartEnabled) {
    return;
  }

  Serial.println("Disabling BLE UART...");
  bleUartEnabled = false;
  bleConnected = false;

  refreshAdvertising();
  Serial.println("BLE UART disabled.");
}

// ===================== BLE CALLBACKS =====================

void bleConnectCallback(uint16_t conn_handle) {
  bleConnected = true;

  // --- MTU negotiation ---
  uint16_t requested_mtu = 247;
  Bluefruit.Connection(conn_handle)->requestMtuExchange(requested_mtu);

  emitEvent("# BLE UART connected");

  uint16_t mtu = Bluefruit.Connection(conn_handle)->getMtu();
  Serial.print("# BLE MTU negotiated: ");
  Serial.println(mtu);

  bleuart.print("# BLE MTU: ");
  bleuart.println(mtu);

  // Identify device immediately over BLE UART
  printFirmwareVersion(bleuart);
  printMCUDeviceID(bleuart);
  printIMUDeviceID(bleuart);
}

void bleDisconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  bleConnected = false;
  Serial.println("BLE UART disconnected");
}
