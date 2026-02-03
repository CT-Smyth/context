void config_and_init() {

// ======================================================================================RESTORE STATE
  //restore state from flash
  loadData();
  loadSettings();
  loadBootMode();
  restoreOutputs();
  syncModbusData();
  syncModbusSettings();
  system_initialized = true;

  if (valid_flash != 12345) {
    resetDefaults();
  }

// ======================================================================================DISPLAY
#ifdef MY_OLED
  obdI2CInit(&oled, MY_OLED, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L);
#endif

#ifdef MY_OLED
  obdFill(&oled, OBD_WHITE, 1);
#ifdef C3_42_OLED
  obdWriteString(&oled, 0, 48, 16, (char *)VERSION_NUMBER, FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&oled, 0, 30, 24, (char *)"HOLD BUTTON", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&oled, 0, 30, 32, (char *)"TO SET WIFI", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&oled, 0, 30, 40, (char *)"OR OTA UPDT", FONT_6x8, OBD_BLACK, 1);
#else
  obdWriteString(&oled, 0, 24, 0, (char *)VERSION_NUMBER, FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&oled, 0, 0, 8, (char *)" HOLD BUTTON", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&oled, 0, 0, 24, (char *)" TO SET WIFI", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&oled, 0, 0, 40, (char *)"OR OTA UPDATE", FONT_6x8, OBD_BLACK, 1);
#endif

#endif
// ======================================================================================BOOT MODES AND CONFIG PORTAL
  Serial.print("BOOT MODE: ");
  Serial.println(boot_mode);

  delay(2000); // wait for button press
  int debounce = 0;

  for (int t = 0; t < 11; t++) {
    if (digitalRead(button_in_pin) == 0) {
      debounce++;
    }
  }

  if ((debounce > 8) || (boot_mode == 1)) { //if button pressed or boot mode is set to '1' from last boot
    char szTemp[32];
    uint8_t IP2 = WiFi.localIP()[2];
    uint8_t IP3 = WiFi.localIP()[3];
    launchPortal();

#ifdef MY_OLED
    obdFill(&oled, OBD_WHITE, 1);
#ifdef C3_42_OLED
    obdWriteString(&oled, 0, 30, 16, (char *)" OTA and ", FONT_6x8, OBD_BLACK, 1);
    obdWriteString(&oled, 0, 30, 24, (char *)"WiFI config", FONT_6x8, OBD_BLACK, 1);
    sprintf(szTemp, "x.x.%d.%d", IP2, IP3);
    obdWriteString(&oled, 0, 30, 32, szTemp, FONT_6x8, OBD_BLACK, 1);
    obdWriteString(&oled, 0, 30, 40, (char *)"192.168.4.1", FONT_6x8, OBD_BLACK, 1);
#else
    obdWriteString(&oled, 0, 0, 0, (char *)" OTA and ", FONT_6x8, OBD_BLACK, 1);
    obdWriteString(&oled, 0, 0, 8, (char *)"WiFI config", FONT_6x8, OBD_BLACK, 1);
    sprintf(szTemp, "x.x.%d.%d", IP2, IP3);
    obdWriteString(&oled, 0, 5, 24, szTemp, FONT_6x8, OBD_BLACK, 1);
    obdWriteString(&oled, 0, 0, 32, (char *)"192.168.4.1", FONT_6x8, OBD_BLACK, 1);
#endif
#endif

    delay(10000); //show this page or status LED for 10 seconds
  } else {
    Serial.print("\nSTARTING UP ..");
    for (int t = 0; t < 5; t++) {
      digitalWrite(ledPin, ledOn);
      delay(50);
      Serial.print("..");
      digitalWrite(ledPin, !ledOn);
      delay(150);
      Serial.print("..");
    }
  }


// set boot-to-portal flag in case of double reset  - it will be cleared after DOUBLE_RESET_TIME
  boot_mode = boot_mode + 1;  
  saveBootMode();
  manPage();
  Serial.println("Initialization complete.");
  if (boot_mode > 2) {  // second or subsequent double reset
    default_display_mode++;
    if (default_display_mode > 4) default_display_mode = 0;
    display_mode = default_display_mode;
    saveSettings();
  }


// ======================================================================================INITIALIZE TIMERS
  ten_ms_heartbeat = millis();
  buttonTime = millis();
  rawPulseTime = millis();
}