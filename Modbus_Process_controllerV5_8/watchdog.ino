void watchdogTimerSetup(){
  //==================================ESP32 Watchdog timer - Note: esp32 v3.x.x requires different code
#if defined ESP32
  esp_task_wdt_deinit();  // ensure a watchdog is not already configured
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR == 3
    // v3 board manager detected
    // Create and initialize the watchdog timer(WDT) configuration structure
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,  // Convert seconds to milliseconds
    .idle_core_mask = 1 << 0,          // Monitor core 1 only
    .trigger_panic = true              // Enable panic
  };
  // Initialize the WDT with the configuration structure
  esp_task_wdt_init(&wdt_config);  // Pass the pointer to the configuration structure
  esp_task_wdt_add(NULL);          // Add current thread to WDT watch
  esp_task_wdt_reset();            // reset timer
#else
    // pre v3 board manager assumed
  esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                //add current thread to WDT watch
#endif
#endif
}