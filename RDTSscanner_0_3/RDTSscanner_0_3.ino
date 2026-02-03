#include <Arduino.h>
#include <bluefruit.h>
#include "RTC.h"
#include "rdts_decode.h"


// Replace with your actual assigned company ID (little-endian in payload)
#define RDTS_COMPANY_ID 0xF00D

// LED pulse duration (ms)
#define LED_PULSE_MS 50

#define BLE_RX_ON_MS    500     // scan window
#define BLE_RX_OFF_MS  5000    // sleep period

typedef enum {
    BLE_STATE_SLEEP = 0,
    BLE_STATE_RX_ON
} ble_state_t;

static ble_state_t ble_state = BLE_STATE_SLEEP;
static uint32_t    ble_state_deadline = 0;

void scan_callback(ble_gap_evt_adv_report_t* report);
static uint32_t led_off_ms = 0;

// void setup() {

//   pinMode(LED_BUILTIN, OUTPUT);
//   digitalWrite(LED_BUILTIN, LOW);

//   Serial.begin(115200);
//   delay(1000);

//   Serial.println("\n=== RDTS BLE Scanner ===");

//   // Central-only
//   Bluefruit.begin(0, 1);
//   Bluefruit.setName("RDTS_SCANNER");

//   // Scanner config
//   Bluefruit.Scanner.setRxCallback(scan_callback);
//   Bluefruit.Scanner.restartOnDisconnect(false);

//   // Interval / window (units = 0.625 ms)
//   Bluefruit.Scanner.setInterval(160, 80);  // ~100 ms interval, 50 ms window
//   Bluefruit.Scanner.useActiveScan(false);  // passive

//   Serial.println("Starting scan...");
//   Bluefruit.Scanner.start(0);  // continuous
// }

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== RDTS BLE Scanner ===");

    ble_init();

    // start asleep
    ble_state = BLE_STATE_SLEEP;
    ble_state_deadline = millis() + BLE_RX_OFF_MS;
}

// void loop() {
//   // Handle LED pulse timeout
//   if (led_off_ms && millis() >= led_off_ms) {
//     digitalWrite(LED_BUILTIN, HIGH);
//     led_off_ms = 0;
//   }
// }

void loop()
{
    uint32_t now = millis();

    // LED pulse timeout (existing logic)
    if (led_off_ms && now >= led_off_ms) {
        digitalWrite(LED_BUILTIN, HIGH);
        led_off_ms = 0;
    }

    // BLE duty-cycle state machine
    switch (ble_state)
    {
        case BLE_STATE_SLEEP:
            if (now >= ble_state_deadline) {
                ble_rx_start();
                ble_state = BLE_STATE_RX_ON;
                ble_state_deadline = now + BLE_RX_ON_MS;
            }
            break;

        case BLE_STATE_RX_ON:
            if (now >= ble_state_deadline) {
                ble_rx_stop();
                ble_state = BLE_STATE_SLEEP;
                ble_state_deadline = now + BLE_RX_OFF_MS;
            }
            break;
    }

    // Optional: light sleep while idle
    __WFE();   // safe on nRF52, wakes on BLE / SysTick
}

static void print_u64(uint64_t v) {
  uint32_t hi = (uint32_t)(v >> 32);
  uint32_t lo = (uint32_t)(v & 0xFFFFFFFFULL);

  if (hi) {
    Serial.print(hi);
    // zero-pad low part to 10 digits
    uint32_t div = 1000000000UL;
    while (div > lo && div > 1) {
      Serial.print('0');
      div /= 10;
    }
    Serial.print(lo);
  } else {
    Serial.print(lo);
  }
}

bool rdts_decode_unix_ms(const uint8_t* payload,
                         uint8_t len,
                         uint64_t* out_unix_ms) {
  if (!payload || len < 8)
    return false;

  uint64_t t = 0;
  memcpy(&t, payload, sizeof(uint64_t));
  *out_unix_ms = t;
  return true;
}

static void scan_callback(ble_gap_evt_adv_report_t* report) {
  //Serial.println("scan_callback()"); //debug
  uint8_t* p = report->data.p_data;
  uint8_t len = report->data.len;

  static uint64_t last_unix_ms = 0;

  for (uint8_t i = 0; i + 4 < len;) {
    uint8_t field_len = p[i];
    uint8_t field_type = p[i + 1];

    if (field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA && field_len >= 3) {
      uint16_t cid = p[i + 2] | (p[i + 3] << 8);

      if (cid == RDTS_COMPANY_ID) {
        const uint8_t* rdts_payload = &p[i + 4];
        uint8_t rdts_len = field_len - 3;

        // Serial.print("RDTS field_len=");
        // Serial.print(field_len);
        // Serial.print(" rdts_len=");
        // Serial.println(rdts_len);

        // Serial.print("RDTS raw payload: ");
        // for (uint8_t k = 0; k < rdts_len; k++) {
        //   if (rdts_payload[k] < 16) Serial.print('0');
        //   Serial.print(rdts_payload[k], HEX);
        //   Serial.print(' ');
        // }
        // Serial.println();

        rdts_packet_t pkt;
        if (!rdts_decode_packet(rdts_payload,
                                rdts_len,
                                &pkt)) {
          Serial.println("RDTS decode FAILED");
          Bluefruit.Scanner.resume();
          return;
        }
        //////////////////
        uint32_t now = millis();

        // monotonic guard
        if (last_unix_ms && pkt.master_unix_ms < last_unix_ms) {
          Serial.println("RTC: rejected backward time");
        } else {
          last_unix_ms = pkt.master_unix_ms;
          rtc_update_from_beacon(pkt.master_unix_ms, now);
        }

        // diagnostics
        Serial.print("[");
        Serial.print(now);
        Serial.print(" ms] RDTS RSSI=");
        Serial.print(report->rssi);
        Serial.print(" dBm -----  ");

        Serial.print("  RTC: ");
        print_u64(rtc_now_unix_ms());
        Serial.print(" offset=");
        Serial.print(rtc_get_last_offset_ms());
        Serial.println(" ms");

        digitalWrite(LED_BUILTIN, LOW);
        led_off_ms = now + LED_PULSE_MS;

        Bluefruit.Scanner.resume();
        return;
      }
    }

    i += field_len + 1;
  }

  Bluefruit.Scanner.resume();
}