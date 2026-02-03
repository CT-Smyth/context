#include "logger.h"
#include <SPIFFS.h>
#include <stdarg.h>

static const char *LOG_PATH = "/log.csv";

// ------------------------------------------------------------
// Init
// ------------------------------------------------------------
bool InitLog() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    return false;
  }
  return true;
}

// ------------------------------------------------------------
// Clear log
// ------------------------------------------------------------
bool ClearLog() {
  if (SPIFFS.exists(LOG_PATH)) {
    SPIFFS.remove(LOG_PATH);
  }
  return true;
}

// ------------------------------------------------------------
// Append log entry
// ------------------------------------------------------------
void DataLog(uint8_t count, ...) {
  File f = SPIFFS.open(LOG_PATH, FILE_APPEND);
  if (!f) {
    Serial.println("Failed to open log file");
    return;
  }

  // First field: millis
  f.print(millis());

  // Variable arguments
  va_list args;
  va_start(args, count);

  for (uint8_t i = 0; i < count; i++) {
    double v = va_arg(args, double);  // floats promoted to double
    f.print(",");
    f.print(v, 6);
  }

  va_end(args);

  f.print("\n");
  f.close();
}

// ------------------------------------------------------------
// Playback
// ------------------------------------------------------------
void PlayLog() {
  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) {
    Serial.println("(log empty)");
    return;
  }

  while (f.available()) {
    Serial.write(f.read());
  }

  f.close();
}