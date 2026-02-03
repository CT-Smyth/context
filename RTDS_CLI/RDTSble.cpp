#include "RDTSble.h"
#include <bluefruit.h>
#include "RDTSserver.h"

// static uint16_t g_span_ms = 0;
// static uint8_t  g_repeat  = 0;
// static uint32_t g_end_ms  = 0;
static uint32_t g_once_stop_ms = 0;

static uint8_t adv_payload[64];
static uint16_t adv_len = 0;

void rdtsble_init(void) {
  Bluefruit.begin();
  Bluefruit.setName("TRACE-MMT_BEACON");
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.setTxPower(8);  // default, overridden later
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
}

void rdtsble_advertise_once(const uint8_t* data, uint16_t len) {
  //Serial.println("[BLE] advertise_once: START");
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.setTxPower(rdtsm_get_beacon_tx_power());

  Bluefruit.Advertising.addData(
    BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
    data,
    len);

  // Choose a very short interval — faster than the SoftDevice timer resolution
  Bluefruit.Advertising.setIntervalMS(32, 32);

  // Start indefinitely — we will stop manually
  Bluefruit.Advertising.start(0);

  // Schedule a stop
  g_once_stop_ms = millis() + 20;  // ~40 ms later, allow 1–2 adv packets

  //Serial.print("[BLE] advertise_once: will stop at ms="); ///DEBUG
  //Serial.println(g_once_stop_ms); ///DEBUG
}

void rdtsble_on_rx(const uint8_t* data, uint16_t len) {
  // client side hook — unused on master
}


void rdtsble_service(void) {
  if (g_once_stop_ms && millis() >= g_once_stop_ms) {
    //Serial.print("[BLE] advertise_once: STOP @ ms="); //debug
    //Serial.println(millis()); //debug
    Bluefruit.Advertising.stop();
    g_once_stop_ms = 0;
  }
}