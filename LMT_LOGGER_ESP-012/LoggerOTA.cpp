#include "LoggerOTA.h"

#include <WiFi.h>
#include <ArduinoOTA.h>

#include "LoggerCore.h"
#include "LoggerHTTP.h"

// ============================================================================
// INTERNAL STATE
// ============================================================================
//
// OTA is a transient mode layered on top of MODE_IDLE.
// WiFi is enabled only while OTA is active.

static bool g_otaStarted = false;

// ============================================================================
// FLASH-BASED CREDENTIAL LOADING
// ============================================================================
//
// Flash slot usage (documented in LoggerOTA.h):
//   slot[1] = WiFi SSID
//   slot[2] = WiFi password
//   slot[3] = OTA password hash (ASCII MD5 hex)
//
// Slots are expected to contain null-terminated ASCII strings.
// Missing or malformed data disables OTA startup.

// Load WiFi SSID and password from reserved flash slots.
// Returns true if a non-empty SSID was loaded.
static bool loadWifiCreds(char *ssid, size_t ssidLen,
                          char *pass, size_t passLen) {
  uint8_t buf[FLASH_PAGE_SIZE];

  if (!readStorageElement(1, buf)) return false;
  strncpy(ssid, (char *)buf, ssidLen);
  ssid[ssidLen - 1] = 0;

  if (!readStorageElement(2, buf)) return false;
  strncpy(pass, (char *)buf, passLen);
  pass[passLen - 1] = 0;

  return ssid[0] != 0;
}

// Load OTA password hash from flash slot 3.
// ArduinoOTA requires a 32-character ASCII MD5 hex string.
static bool loadOtaHash(char *hash, size_t hashLen) {
  uint8_t buf[FLASH_PAGE_SIZE];

  if (!readStorageElement(3, buf)) return false;

  strncpy(hash, (char *)buf, hashLen);
  hash[hashLen - 1] = 0;

  return strlen(hash) == 32;
}

// ============================================================================
// PUBLIC STATE QUERIES
// ============================================================================

bool otaStarted() {
  return g_otaStarted;
}

// ============================================================================
// OTA STARTUP
// ============================================================================
//
// startOTA() performs a blocking WiFi connection attempt.
// This is intentional and only allowed while MODE_IDLE.
//
// On any failure:
//   - WiFi is shut down
//   - HTTP is stopped
//   - OTA is left disabled

void startOTA() {
  if (g_otaStarted) return;

  char ssid[64] = {};
  char pass[64] = {};

  if (!loadWifiCreds(ssid, sizeof(ssid), pass, sizeof(pass))) {
    Serial.println("# OTA: WiFi credentials missing");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;

  uint32_t start = millis();
  Serial.print("# OTA: connecting WiFi");

  // Blocking connect loop with timeout
  while (WiFi.status() != WL_CONNECTED &&
         (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" FAILED");

    // Clean rollback on failure
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    stopHTTP();

    g_otaStarted = false;
    Serial.println("# OTA disabled");
    return;
  }

  Serial.println(" connected");

  // Register OTA lifecycle callbacks
  ArduinoOTA
    .onStart([]() {
      Serial.println("# OTA: start");
    })
    .onEnd([]() {
      Serial.println("# OTA: end");
    })
    .onError([](ota_error_t err) {
      Serial.printf("# OTA error %u\n", err);
    });

  char otaHash[40] = {};

  if (!loadOtaHash(otaHash, sizeof(otaHash))) {
    Serial.println("# OTA: password hash missing");

    // Abort OTA if authentication is not configured
    stopHTTP();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }

  ArduinoOTA.setPasswordHash(otaHash);
  ArduinoOTA.begin();

  g_otaStarted = true;

  // HTTP is enabled alongside OTA (e.g. for /flash endpoint)
  startHTTP();

  Serial.println("# OTA enabled");
}

// ============================================================================
// OTA SHUTDOWN
// ============================================================================
//
// stopOTA() fully disables OTA and WiFi.
// BLE and Serial remain active.

void stopOTA() {
  if (!g_otaStarted) return;

  ArduinoOTA.end();

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  g_otaStarted = false;
  Serial.println("# OTA disabled");
}

// ============================================================================
// OTA SERVICE
// ============================================================================
//
// Must be called periodically from loop() while OTA is active.
// Does nothing when OTA is disabled.

void serviceOTA() {
  if (!g_otaStarted) return;
  ArduinoOTA.handle();
}

// ============================================================================
// IP ADDRESS HELPERS
// ============================================================================

bool otaHasIP() {
  if (!g_otaStarted) return false;
  return WiFi.status() == WL_CONNECTED;
}

void otaGetIP(char *out, size_t outLen) {
  if (!out || outLen == 0) return;

  if (!otaHasIP()) {
    out[0] = 0;
    return;
  }

  IPAddress ip = WiFi.localIP();
  snprintf(out, outLen, "%u.%u.%u.%u",
           ip[0], ip[1], ip[2], ip[3]);
}