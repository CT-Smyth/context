

// KEYSTORE KEYLENGTH MAX 15 CHAR!!!!!

void saveData() {
  Serial.println("\nSAVE DATA");
  last_save = millis();
  blink = millis() + 200;
  keystore.begin("keyData", false);
  keystore.putUShort("relay_status", relay_status);
  keystore.putUShort("horn_status", horn_status);
  keystore.putUShort("l_units_MSW", lifetime_units_MSW);
  keystore.putUShort("l_units_LSW", lifetime_units_LSW);
  keystore.putUShort("l_flow_units", largest_flow_units);
  keystore.putUShort("l_flow_dur", longest_flow_duration);
  keystore.putUShort("output_pulses", output_pulses);
  keystore.putULong("last_save", last_save);

  keystore.end();

  lifetime_units_from_flash_MSW = lifetime_units_MSW;
  lifetime_units_from_flash_LSW = lifetime_units_LSW;
}

void saveSettings() {
  if (system_initialized == true) {
    blink = millis() + 200;
    Serial.println("\nSAVE SETTINGS");
    valid_flash = 12345;
    keystore.begin("keyData", false);
    keystore.putUShort("valid_flash", valid_flash);
    keystore.putUShort("mode", mode);
    keystore.putUShort("pin_mode", pin_mode);
    keystore.putUShort("sen_p_per_unit", sensor_pulses_per_unit);
    keystore.putUShort("save_interval", save_interval);
    keystore.putUShort("horn_units", horn_units);
    keystore.putUShort("relay_units", relay_units);
    keystore.putUShort("horn_seconds", horn_seconds);
    keystore.putUShort("relay_seconds", relay_seconds);
    keystore.putUShort("flow_stop_sec", flow_stop_seconds);
    keystore.putUShort("relay_latchmode", relay_latchmode);
    keystore.putUShort("horn_latchmode", horn_latchmode);
    keystore.putUShort("output_pulse_ms", output_pulse_ms);
    keystore.putUShort("output_p_off_ms", output_pulse_off_ms);
    keystore.putUShort("units_per_event", units_per_event);
    keystore.putUShort("p_per_trigger", output_pulses_per_trigger);
    keystore.putUShort("p_for_prime", output_pulses_for_prime);
    keystore.putUShort("flow_limit", min_flow);
    keystore.putUShort("analog_thresh", analog_threshold);
    keystore.putUShort("analog_min", analog_map_min);
    keystore.putUShort("analog_max", analog_map_max);
    keystore.putUShort("analog_r_min", analog_range_min);
    keystore.putUShort("analog_r_max", analog_range_max);
    keystore.putUShort("flowrate_sec", flowRate_seconds);
    keystore.putUShort("def_disp_mode", default_display_mode);
    keystore.end();
  }
}

void saveRelayState() {
  // only save pinstate for relay if mode 1 + pulses are not active to avoid flash wear
  if (((mode != 0)) && ((relay_latchmode == 1) || (relay_latchmode == 3))) {
    keystore.begin("keyData", false);
    keystore.putUShort("relay_status", relay_status);
    keystore.end();

    //Serial.print("saved relay_status = ");  //DEBUG
    //Serial.println(relay_status);           //DEBUG
  }
}

void saveHornState() {

  if ((horn_latchmode == 1) || (horn_latchmode == 3)) {
    keystore.begin("keyData", false);
    keystore.putUShort("horn_status", horn_status);
    keystore.end();

    //Serial.print("saved horn_status = ");  //DEBUG
    //Serial.println(horn_status);           //DEBUG
  }
}

void saveBootMode() {
  // Serial.print("Saving boot mode ==");  //DEBUG
  // Serial.print(boot_mode);
  keystore.begin("keyData", false);
  keystore.putUShort("boot_mode", boot_mode);
  keystore.end();
}

void saveDisplayMode() {
  // Serial.print("Saving display mode ==");  //DEBUG
  // Serial.print(default_display_mode);
  keystore.begin("keyData", false);
  keystore.putUShort("def_disp_mode", default_display_mode);
  keystore.end();
}


void loadData() {
  Serial.println("\nLOAD DATA");
  blink = millis() + 100;

  keystore.begin("keyData", false);
  valid_flash = keystore.getUShort("valid_flash", 0);
  largest_flow_units = keystore.getUShort("l_flow_units", 0);
  longest_flow_duration = keystore.getUShort("l_flow_dur", 0);
  lifetime_units_MSW = keystore.getUShort("l_units_MSW", 0);
  lifetime_units_LSW = keystore.getUShort("l_units_LSW", 0);  
  output_pulses = keystore.getUShort("output_pulses", 0);  
  last_save = keystore.getULong("last_save");
  keystore.end();

  lifetime_units_from_flash_MSW = lifetime_units_MSW;
  lifetime_units_from_flash_LSW = lifetime_units_LSW;
}

void loadSettings() {
  Serial.println("\nLOAD SETTINGS");
  blink = millis() + 100;

  keystore.begin("keyData", false);
  valid_flash = keystore.getUShort("valid_flash", 0);
  mode = keystore.getUShort("mode", 0);
  pin_mode = keystore.getUShort("pin_mode", 0);
  sensor_pulses_per_unit = keystore.getUShort("sen_p_per_unit", 0);
  save_interval = keystore.getUShort("save_interval", 0);
  horn_units = keystore.getUShort("horn_units", 0);
  relay_units = keystore.getUShort("relay_units", 0);
  horn_seconds = keystore.getUShort("horn_seconds", 0);
  relay_seconds = keystore.getUShort("relay_seconds", 0);
  flow_stop_seconds = keystore.getUShort("flow_stop_sec", 0);
  relay_latchmode = keystore.getUShort("relay_latchmode", 0);
  horn_latchmode = keystore.getUShort("horn_latchmode", 0);
  output_pulse_ms = keystore.getUShort("output_pulse_ms", 0);
  output_pulse_off_ms = keystore.getUShort("output_p_off_ms", 0);
  units_per_event = keystore.getUShort("units_per_event", 0);
  output_pulses_per_trigger = keystore.getUShort("p_per_trigger", 0);
  output_pulses_for_prime = keystore.getUShort("p_for_prime", 0);
  min_flow = keystore.getUShort("flow_limit", 0);
  analog_threshold = keystore.getUShort("analog_thresh", 0);
  analog_map_min = keystore.getUShort("analog_min", 0);
  analog_map_max = keystore.getUShort("analog_max", 0);
  analog_range_min = keystore.getUShort("analog_r_min", 0);
  analog_range_max = keystore.getUShort("analog_r_max", 0);
  flowRate_seconds = keystore.getUShort("flowrate_sec", 0);
  default_display_mode = keystore.getUShort("def_disp_mode", 0);
  display_mode = default_display_mode;
  keystore.end();
  configure_interrupt_type();
}



void loadBootMode() {
  keystore.begin("keyData", false);
  boot_mode = keystore.getUShort("boot_mode", 0);
  keystore.end();
  // Serial.print("Loading boot mode");  //DEBUG
  // Serial.print(boot_mode);
}

void restoreOutputs() {
  //Serial.println("Loading pin status");  //DEBUG

  if ((relay_latchmode == 1) || (relay_latchmode == 3)) {
    keystore.begin("keyData", false);
    relay_status = keystore.getUShort("relay_status", 0);
    keystore.end();
    //Serial.println(relay_status);            //DEBUG
    setRelay(relay_status);
  }

  if ((horn_latchmode == 1) || (horn_latchmode == 3)) {
    keystore.begin("keyData", false);
    horn_status = keystore.getUShort("horn_status", 0);
    keystore.end();
    //Serial.println(horn_status);            //DEBUG
    setHorn(horn_status);
  }
}
