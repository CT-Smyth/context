#include <Arduino.h>
#include "logger.h"
#include <math.h>

static int sample_count = 0;
static bool played = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  InitLog();
  ClearLog();

  Serial.println("Logging example started");
}

void loop() {
  if (sample_count < 1000) {
    float x = sinf(sample_count * 0.1f);     // sinusoid
    float y = random(-100, 100) / 100.0f;    // random
    float z = (sample_count % 20) / 20.0f;   // sawtooth 0..1

    DataLog(3, x, y, z);

    sample_count++;
    delay(1);
    return;
  }

  if (!played) {
    Serial.println("\n--- PLAYBACK ---");
    PlayLog();
    Serial.println("--- END PLAYBACK ---\n");

    ClearLog();
    Serial.println("Log cleared\n");

    sample_count = 0;
    played = true;
    delay(2000);
    return;
  }

  // restart demo
  played = false;
}