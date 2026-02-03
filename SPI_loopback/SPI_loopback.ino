#include <SPI.h>

const int PIN_CS = D2; // Chip Select pin (optional)
const uint8_t TEST_PATTERN[] = {0x00, 0x55, 0xAA, 0xFF, 0x3C, 0xC3};
const int NUM_PATTERNS = sizeof(TEST_PATTERN);

uint32_t testSpeeds[] = {
  100000, 250000, 500000, 1000000,
  2000000, 4000000, 8000000,
  12000000, 16000000, 20000000,
  24000000, 32000000, 40000000, 50000000
};

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("\n=== SPI Speed Stress Test ==="));
  Serial.println(F("Connect MOSI → MISO for loopback verification."));
  Serial.println(F("Results: OK means data verified at that speed."));
  Serial.println(F("---------------------------------------------"));
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin();

  testSPISpeeds();
}

void loop() {
  // Nothing to do — test runs once
}

bool spiVerify(uint32_t speed) {
  SPI.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(2);

  bool ok = true;
  for (int i = 0; i < NUM_PATTERNS; i++) {
    uint8_t tx = TEST_PATTERN[i];
    uint8_t rx = SPI.transfer(tx);
    if (rx != tx) {
      ok = false;
      break;
    }
  }

  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return ok;
}

void testSPISpeeds() {
  for (unsigned int i = 0; i < sizeof(testSpeeds)/sizeof(testSpeeds[0]); i++) {
    uint32_t spd = testSpeeds[i];
    Serial.print("Testing ");
    Serial.print(spd / 1000);
    Serial.print(" kHz... ");
    bool ok = spiVerify(spd);
    if (ok) Serial.println("✅ OK");
    else Serial.println("❌ FAIL");
    delay(100);
  }

  Serial.println(F("---------------------------------------------"));
  Serial.println(F("Test complete."));
  Serial.println(F("✅ indicates stable operation at that SPI clock speed."));
  Serial.println(F("❌ indicates signal integrity or timing failure."));
}

