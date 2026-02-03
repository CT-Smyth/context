//Modbus callbacks and functions
void syncModbusData() {
  int8_t wifiSig = WiFi.RSSI();
  uint16_t uptime = millis() / 3600000;  //uptime hours
  uint16_t rssi_db = abs(wifiSig);

  //IREGs
  mBus.Ireg(LIFETIME_UNITS_LSW, lifetime_units_LSW);
  mBus.Ireg(LIFETIME_UNITS_MSW, lifetime_units_MSW);
  mBus.Ireg(THIS_FLOW_UNITS, this_flow_units);
  mBus.Ireg(THIS_FLOW_DURATION, this_flow_duration);
  mBus.Ireg(LARGEST_FLOW_UNITS, largest_flow_units);
  mBus.Ireg(LONGEST_FLOW_DURATION, longest_flow_duration);
  mBus.Ireg(FLOW_RATE, flow_rate);
  mBus.Ireg(OUTPUT_PULSES_TODO, output_pulses_todo);
  mBus.Ireg(SENSOR_PULSES, sensor_pulses);
  mBus.Ireg(ANALOG_SENSE, analog_sense);
  mBus.Ireg(UNITS_QUEUED, units_queued);
  mBus.Ireg(UPTIME_HRS, uptime);
  mBus.Ireg(WIFI_RSSI, rssi_db);
  mBus.Ireg(OUTPUT_PULSES, output_pulses);

  //HREGs
  mBus.Hreg(UNITS_SINCE_BOOT, units_since_boot);
  mBus.Hreg(FLOW_TIME_SINCE_BOOT, flow_time_since_boot);
}

void syncModbusSettings() {
  mBus.Hreg(MODE, mode);
  mBus.Hreg(PIN_MODE, pin_mode);
  mBus.Hreg(SENSOR_PULSES_PER_UNIT, sensor_pulses_per_unit);
  mBus.Hreg(SAVE_INTERVAL, save_interval);
  mBus.Hreg(HORN_UNITS, horn_units);
  mBus.Hreg(RELAY_UNITS, relay_units);
  mBus.Hreg(HORN_SECONDS, horn_seconds);
  mBus.Hreg(RELAY_SECONDS, relay_seconds);
  mBus.Hreg(FLOW_STOP_SECONDS, flow_stop_seconds);
  mBus.Hreg(RELAY_LATCHMODE, relay_latchmode);
  mBus.Hreg(HORN_LATCHMODE, horn_latchmode);
  mBus.Hreg(OUTPUT_PULSE_MS, output_pulse_ms);
  mBus.Hreg(OUTPUT_PULSE_OFF_MS, output_pulse_off_ms);
  mBus.Hreg(UNITS_PER_EVENT, units_per_event);
  mBus.Hreg(OUTPUT_PULSES_PER_TRIGGER, output_pulses_per_trigger);
  mBus.Hreg(OUTPUT_PULSES_FOR_PRIME, output_pulses_for_prime);
  mBus.Hreg(MIN_FLOW, min_flow);
  mBus.Hreg(ANALOG_THRESHOLD, analog_threshold);
  mBus.Hreg(ANALOG_MAP_MIN, analog_map_min);
  mBus.Hreg(ANALOG_MAP_MAX, analog_map_max);
  mBus.Hreg(ANALOG_RANGE_MIN, analog_range_min);
  mBus.Hreg(ANALOG_RANGE_MAX, analog_range_max);
  mBus.Hreg(FLOWRATE_SECONDS, flowRate_seconds);

  mBus.task();
}


void setupModbus() {
  //MODBUS SETUP
  mBus.onConnect(connect_callback);  // Add callback on connection event. must return 1 to allow connection
  mBus.server();

  //COILs
  mBus.addCoil(HORN_COIL);
  mBus.onSetCoil(HORN_COIL, set_HORN_callback);
  mBus.onGetCoil(HORN_COIL, get_HORN_callback);

  mBus.addCoil(RELAY_COIL);
  mBus.onSetCoil(RELAY_COIL, set_RELAY_callback);
  mBus.onGetCoil(RELAY_COIL, get_RELAY_callback);

  mBus.addCoil(STOP_STATUS_COIL);
  mBus.onSetCoil(STOP_STATUS_COIL, set_STOP_callback);
  mBus.onGetCoil(STOP_STATUS_COIL, get_STOP_callback);

  mBus.addCoil(SAVE_COIL);
  mBus.onSetCoil(SAVE_COIL, set_SAVE_callback);
  mBus.onGetCoil(SAVE_COIL, get_SAVE_callback);

  //ISTS
  mBus.addIsts(HORN_COIL);
  mBus.onSetIsts(HORN_COIL, set_HORN_ISTS_callback);
  mBus.onGetIsts(HORN_COIL, get_HORN_ISTS_callback);

  mBus.addIsts(RELAY_COIL);
  mBus.onSetIsts(RELAY_COIL, set_RELAY_ISTS_callback);
  mBus.onGetIsts(RELAY_COIL, get_RELAY_ISTS_callback);

  //IREGs
  mBus.addIreg(LIFETIME_UNITS_MSW, lifetime_units_MSW);
  mBus.onGetIreg(LIFETIME_UNITS_MSW, get_LIFETIME_UNITS_MSW_callback);

  mBus.addIreg(LIFETIME_UNITS_LSW, lifetime_units_LSW);
  mBus.onGetIreg(LIFETIME_UNITS_LSW, get_LIFETIME_UNITS_LSW_callback);

  mBus.addIreg(THIS_FLOW_UNITS, 0);
  mBus.onGetIreg(THIS_FLOW_UNITS, get_THIS_FLOW_UNITS_callback);

  mBus.addIreg(THIS_FLOW_DURATION, 0);
  mBus.onGetIreg(THIS_FLOW_DURATION, get_THIS_FLOW_DURATION_callback);

  mBus.addIreg(LARGEST_FLOW_UNITS, largest_flow_units);
  mBus.onGetIreg(LARGEST_FLOW_UNITS, get_LARGEST_FLOW_UNITS_callback);

  mBus.addIreg(LONGEST_FLOW_DURATION, longest_flow_duration);
  mBus.onGetIreg(LONGEST_FLOW_DURATION, get_LONGEST_FLOW_DURATION_callback);

  mBus.addIreg(FLOW_RATE, 0);
  mBus.onGetIreg(FLOW_RATE, get_FLOW_RATE_callback);

  mBus.addIreg(OUTPUT_PULSES_TODO, 0);
  mBus.onGetIreg(OUTPUT_PULSES_TODO, get_OUTPUT_PULSES_TODO_callback);

  mBus.addIreg(SENSOR_PULSES, 0);
  mBus.onGetIreg(SENSOR_PULSES, get_SENSOR_PULSES_callback);

  mBus.addIreg(ANALOG_SENSE, 0);  //  Use addIreg() for analog Inputs
  mBus.onGetIreg(ANALOG_SENSE, get_ANALOG_SENSE_callback);

  mBus.addIreg(UNITS_QUEUED, 0);
  mBus.onGetIreg(UNITS_QUEUED, get_UNITS_QUEUED_callback);

  mBus.addIreg(UPTIME_HRS, 0);
  mBus.onGetIreg(UPTIME_HRS, get_UPTIME_HRS_callback);

  mBus.addIreg(WIFI_RSSI, 0);
  mBus.onGetIreg(WIFI_RSSI, get_WIFI_RSSI_callback);

  mBus.addIreg(OUTPUT_PULSES, 0);
  mBus.onGetIreg(OUTPUT_PULSES, get_OUTPUT_PULSES_callback);


  //HREGs
  mBus.addHreg(OUTPUT_PULSES_TO_ADD, 0);
  mBus.onSetHreg(OUTPUT_PULSES_TO_ADD, set_OUTPUT_PULSES_TO_ADD_callback);
  mBus.onGetHreg(OUTPUT_PULSES_TO_ADD, get_OUTPUT_PULSES_TO_ADD_callback);

  mBus.addHreg(UNITS_SINCE_BOOT, 0);
  mBus.onSetHreg(UNITS_SINCE_BOOT, set_UNITS_SINCE_BOOT_callback);
  mBus.onGetHreg(UNITS_SINCE_BOOT, get_UNITS_SINCE_BOOT_callback);

  mBus.addHreg(FLOW_TIME_SINCE_BOOT, 0);
  mBus.onSetHreg(FLOW_TIME_SINCE_BOOT, set_FLOW_TIME_SINCE_BOOT_callback);
  mBus.onGetHreg(FLOW_TIME_SINCE_BOOT, get_FLOW_TIME_SINCE_BOOT_callback);

  mBus.addHreg(SHOW_DISPLAY, 0);
  mBus.onSetHreg(SHOW_DISPLAY, set_SHOW_DISPLAY_callback);
  mBus.onGetHreg(SHOW_DISPLAY, get_SHOW_DISPLAY_callback);

  mBus.addHreg(DISPLAY_MODE, 0);
  mBus.onSetHreg(DISPLAY_MODE, set_DISPLAY_MODE_callback);
  mBus.onGetHreg(DISPLAY_MODE, get_DISPLAY_MODE_callback);

  mBus.addHreg(MODE, 0);
  mBus.onSetHreg(MODE, set_MODE_callback);
  mBus.onGetHreg(MODE, get_MODE_callback);

  mBus.addHreg(PIN_MODE, 0);
  mBus.onSetHreg(PIN_MODE, set_PIN_MODE_callback);
  mBus.onGetHreg(PIN_MODE, get_PIN_MODE_callback);

  mBus.addHreg(SENSOR_PULSES_PER_UNIT, 0);
  mBus.onSetHreg(SENSOR_PULSES_PER_UNIT, set_SENSOR_PULSES_PER_UNIT_callback);
  mBus.onGetHreg(SENSOR_PULSES_PER_UNIT, get_SENSOR_PULSES_PER_UNIT_callback);

  mBus.addHreg(SAVE_INTERVAL, 0);
  mBus.onSetHreg(SAVE_INTERVAL, set_SAVE_INTERVAL_callback);
  mBus.onGetHreg(SAVE_INTERVAL, get_SAVE_INTERVAL_callback);

  mBus.addHreg(HORN_UNITS, 0);
  mBus.onSetHreg(HORN_UNITS, set_HORN_UNITS_callback);
  mBus.onGetHreg(HORN_UNITS, get_HORN_UNITS_callback);

  mBus.addHreg(RELAY_UNITS, 0);
  mBus.onSetHreg(RELAY_UNITS, set_RELAY_UNITS_callback);
  mBus.onGetHreg(RELAY_UNITS, get_RELAY_UNITS_callback);

  mBus.addHreg(HORN_SECONDS, 0);
  mBus.onSetHreg(HORN_SECONDS, set_HORN_SECONDS_callback);
  mBus.onGetHreg(HORN_SECONDS, get_HORN_SECONDS_callback);

  mBus.addHreg(RELAY_SECONDS, 0);
  mBus.onSetHreg(RELAY_SECONDS, set_RELAY_SECONDS_callback);
  mBus.onGetHreg(RELAY_SECONDS, get_RELAY_SECONDS_callback);

  mBus.addHreg(FLOW_STOP_SECONDS, 0);
  mBus.onSetHreg(FLOW_STOP_SECONDS, set_FLOW_STOP_SECONDS_callback);
  mBus.onGetHreg(FLOW_STOP_SECONDS, get_FLOW_STOP_SECONDS_callback);

  mBus.addHreg(RELAY_LATCHMODE, 0);
  mBus.onSetHreg(RELAY_LATCHMODE, set_RELAY_LATCHMODE_callback);
  mBus.onGetHreg(RELAY_LATCHMODE, get_RELAY_LATCHMODE_callback);

  mBus.addHreg(HORN_LATCHMODE, 0);
  mBus.onSetHreg(HORN_LATCHMODE, set_HORN_LATCHMODE_callback);
  mBus.onGetHreg(HORN_LATCHMODE, get_HORN_LATCHMODE_callback);

  mBus.addHreg(OUTPUT_PULSE_MS, 0);
  mBus.onSetHreg(OUTPUT_PULSE_MS, set_OUTPUT_PULSE_MS_callback);
  mBus.onGetHreg(OUTPUT_PULSE_MS, get_OUTPUT_PULSE_MS_callback);

  mBus.addHreg(OUTPUT_PULSE_OFF_MS, 0);
  mBus.onSetHreg(OUTPUT_PULSE_OFF_MS, set_OUTPUT_PULSE_OFF_MS_callback);
  mBus.onGetHreg(OUTPUT_PULSE_OFF_MS, get_OUTPUT_PULSE_OFF_MS_callback);

  mBus.addHreg(UNITS_PER_EVENT, 0);
  mBus.onSetHreg(UNITS_PER_EVENT, set_UNITS_PER_EVENT_callback);
  mBus.onGetHreg(UNITS_PER_EVENT, get_UNITS_PER_EVENT_callback);

  mBus.addHreg(OUTPUT_PULSES_PER_TRIGGER, 0);
  mBus.onSetHreg(OUTPUT_PULSES_PER_TRIGGER, set_OUTPUT_PULSES_PER_TRIGGER_callback);
  mBus.onGetHreg(OUTPUT_PULSES_PER_TRIGGER, get_OUTPUT_PULSES_PER_TRIGGER_callback);

  mBus.addHreg(OUTPUT_PULSES_FOR_PRIME, 0);
  mBus.onSetHreg(OUTPUT_PULSES_FOR_PRIME, set_OUTPUT_PULSES_FOR_PRIME_callback);
  mBus.onGetHreg(OUTPUT_PULSES_FOR_PRIME, get_OUTPUT_PULSES_FOR_PRIME_callback);

  mBus.addHreg(MIN_FLOW, 0);
  mBus.onSetHreg(MIN_FLOW, set_MIN_FLOW_callback);
  mBus.onGetHreg(MIN_FLOW, get_MIN_FLOW_callback);

  mBus.addHreg(ANALOG_THRESHOLD, 0);
  mBus.onSetHreg(ANALOG_THRESHOLD, set_ANALOG_THRESHOLD_callback);
  mBus.onGetHreg(ANALOG_THRESHOLD, get_ANALOG_THRESHOLD_callback);

  mBus.addHreg(ANALOG_MAP_MIN, 0);
  mBus.onSetHreg(ANALOG_MAP_MIN, set_ANALOG_MAP_MIN_callback);
  mBus.onGetHreg(ANALOG_MAP_MIN, get_ANALOG_MAP_MIN_callback);

  mBus.addHreg(ANALOG_MAP_MAX, 0);
  mBus.onSetHreg(ANALOG_MAP_MAX, set_ANALOG_MAP_MAX_callback);
  mBus.onGetHreg(ANALOG_MAP_MAX, get_ANALOG_MAP_MAX_callback);

  mBus.addHreg(ANALOG_RANGE_MIN, 0);
  mBus.onSetHreg(ANALOG_RANGE_MIN, set_ANALOG_RANGE_MIN_callback);
  mBus.onGetHreg(ANALOG_RANGE_MIN, get_ANALOG_RANGE_MIN_callback);

  mBus.addHreg(ANALOG_RANGE_MAX, 0);
  mBus.onSetHreg(ANALOG_RANGE_MAX, set_ANALOG_RANGE_MAX_callback);
  mBus.onGetHreg(ANALOG_RANGE_MAX, get_ANALOG_RANGE_MAX_callback);

  mBus.addHreg(FLOWRATE_SECONDS, 0);
  mBus.onSetHreg(FLOWRATE_SECONDS, set_FLOWRATE_SECONDS_callback);
  mBus.onGetHreg(FLOWRATE_SECONDS, get_FLOWRATE_SECONDS_callback);

  mBus.addHreg(ADD_TOTAL_UNITS, 0);
  mBus.onSetHreg(ADD_TOTAL_UNITS, set_ADD_TOTAL_UNITS_callback);
  mBus.onGetHreg(ADD_TOTAL_UNITS, get_ADD_TOTAL_UNITS_callback);

  mBus.addHreg(RESET_HREG);
  mBus.onSetHreg(RESET_HREG, set_RESET_callback);
  mBus.onGetHreg(RESET_HREG, get_RESET_callback);
}



bool connect_callback(IPAddress ip) {
  Serial.print("Modbus connected from: ");
  Serial.println(ip);
  return true;
}

//coil callbacks
uint16_t set_HORN_callback(TRegister* reg, uint16_t val) {
  horn_set_by_modbus = bool(val);
  setHorn(horn_set_by_modbus);
  return val;
}
uint16_t get_HORN_callback(TRegister* reg, uint16_t val) {
  //dataPage();
  return val;
}

uint16_t set_RELAY_callback(TRegister* reg, uint16_t val) {
  relay_set_by_modbus = bool(val);
  setRelay(relay_set_by_modbus);
  return val;
}
uint16_t get_RELAY_callback(TRegister* reg, uint16_t val) {
  //manPage();
  return val;
}

uint16_t set_STOP_callback(TRegister* reg, uint16_t val) {
  Serial.println("STOP OPERATION");
  stop();
  return val;
}
uint16_t get_STOP_callback(TRegister* reg, uint16_t val) {
  Serial.println("\n\nStatus to serial terminal: \n");
  manPage();
  return val;
}

uint16_t set_SAVE_callback(TRegister* reg, uint16_t val) {
  saveSettings();
  //saveData();
  return val;
}
uint16_t get_SAVE_callback(TRegister* reg, uint16_t val) {
  saveData();
  return val;
}



//Ists callbacks
uint16_t get_HORN_ISTS_callback(TRegister* reg, uint16_t val) {
  return val;
}
uint16_t set_HORN_ISTS_callback(TRegister* reg, uint16_t val) {
  return val;
}

uint16_t get_RELAY_ISTS_callback(TRegister* reg, uint16_t val) {
  return val;
}
uint16_t set_RELAY_ISTS_callback(TRegister* reg, uint16_t val) {
  return val;
}



//Ireg callbacks
uint16_t get_LIFETIME_UNITS_MSW_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_LIFETIME_UNITS_LSW_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_THIS_FLOW_UNITS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_THIS_FLOW_DURATION_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_LARGEST_FLOW_UNITS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_LONGEST_FLOW_DURATION_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_FLOW_RATE_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_OUTPUT_PULSES_TODO_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_SENSOR_PULSES_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_ANALOG_SENSE_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_UNITS_QUEUED_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_UPTIME_HRS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_WIFI_RSSI_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t get_OUTPUT_PULSES_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}



//Hreg Callbacks
uint16_t set_OUTPUT_PULSES_TO_ADD_callback(TRegister* reg, uint16_t val) {
  output_pulses_todo = output_pulses_todo + val;
  Serial.print(val);
  Serial.print(" pulses added ... ");
  Serial.print("total pulses to do = ");
  Serial.println(output_pulses_todo);
  return val;
}
uint16_t get_OUTPUT_PULSES_TO_ADD_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_UNITS_SINCE_BOOT_callback(TRegister* reg, uint16_t val) {
  units_since_boot = val;
  return val;
}
uint16_t get_UNITS_SINCE_BOOT_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_FLOW_TIME_SINCE_BOOT_callback(TRegister* reg, uint16_t val) {
  flow_time_since_boot = val;
  return val;
}
uint16_t get_FLOW_TIME_SINCE_BOOT_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t set_SHOW_DISPLAY_callback(TRegister* reg, uint16_t val) {
  if (val > 1000) {
    val = 1000;
  }
  displayTimeout = (val * 60);
  displayTime = millis() + u_long(displayTimeout * 1000);
  return val;
}
uint16_t get_SHOW_DISPLAY_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t set_DISPLAY_MODE_callback(TRegister* reg, uint16_t val) {
  if (val > 5) {
    val = 5;
  }
  default_display_mode = val;
  display_mode = default_display_mode;
  displayTime = millis() + 300000;
  saveSettings();
  return val;
}
uint16_t get_DISPLAY_MODE_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t set_MODE_callback(TRegister* reg, uint16_t val) {
  mode = val;
  saveSettings();
  return val;
}
uint16_t get_MODE_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_PIN_MODE_callback(TRegister* reg, uint16_t val) {
  pin_mode = val;
  configure_interrupt_type();
  saveSettings();
  return val;
}
uint16_t get_PIN_MODE_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_SENSOR_PULSES_PER_UNIT_callback(TRegister* reg, uint16_t val) {
  sensor_pulses_per_unit = val;
  if (sensor_pulses_per_unit < 1) sensor_pulses_per_unit = 1;
  saveSettings();
  return val;
}
uint16_t get_SENSOR_PULSES_PER_UNIT_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_SAVE_INTERVAL_callback(TRegister* reg, uint16_t val) {
  save_interval = val;
  saveSettings();
  return val;
}
uint16_t get_SAVE_INTERVAL_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_HORN_UNITS_callback(TRegister* reg, uint16_t val) {
  horn_units = val;
  saveSettings();
  return val;
}
uint16_t get_HORN_UNITS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_RELAY_UNITS_callback(TRegister* reg, uint16_t val) {
  relay_units = val;
  saveSettings();
  return val;
}
uint16_t get_RELAY_UNITS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_HORN_SECONDS_callback(TRegister* reg, uint16_t val) {
  horn_seconds = val;
  saveSettings();
  return val;
}
uint16_t get_HORN_SECONDS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_RELAY_SECONDS_callback(TRegister* reg, uint16_t val) {
  relay_seconds = val;
  saveSettings();
  return val;
}
uint16_t get_RELAY_SECONDS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t set_FLOW_STOP_SECONDS_callback(TRegister* reg, uint16_t val) {
  flow_stop_seconds = val;
  saveSettings();
  return val;
}
uint16_t get_FLOW_STOP_SECONDS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_RELAY_LATCHMODE_callback(TRegister* reg, uint16_t val) {
  relay_latchmode = val;
  saveSettings();
  return val;
}
uint16_t get_RELAY_LATCHMODE_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_HORN_LATCHMODE_callback(TRegister* reg, uint16_t val) {
  horn_latchmode = val;
  saveSettings();
  return val;
}
uint16_t get_HORN_LATCHMODE_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_OUTPUT_PULSE_MS_callback(TRegister* reg, uint16_t val) {
  output_pulse_ms = val;
  saveSettings();
  return val;
}
uint16_t get_OUTPUT_PULSE_MS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_OUTPUT_PULSE_OFF_MS_callback(TRegister* reg, uint16_t val) {
  output_pulse_off_ms = val;
  saveSettings();
  return val;
}
uint16_t get_OUTPUT_PULSE_OFF_MS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_UNITS_PER_EVENT_callback(TRegister* reg, uint16_t val) {
  units_per_event = val;
  saveSettings();
  return val;
}
uint16_t get_UNITS_PER_EVENT_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_OUTPUT_PULSES_PER_TRIGGER_callback(TRegister* reg, uint16_t val) {
  output_pulses_per_trigger = val;
  saveSettings();
  return val;
}
uint16_t get_OUTPUT_PULSES_PER_TRIGGER_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_OUTPUT_PULSES_FOR_PRIME_callback(TRegister* reg, uint16_t val) {
  output_pulses_for_prime = val;
  saveSettings();
  return val;
}
uint16_t get_OUTPUT_PULSES_FOR_PRIME_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_MIN_FLOW_callback(TRegister* reg, uint16_t val) {
  min_flow = val;
  saveSettings();
  return val;
}
uint16_t get_MIN_FLOW_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_ANALOG_THRESHOLD_callback(TRegister* reg, uint16_t val) {
  analog_threshold = val;
  saveSettings();
  return val;
}
uint16_t get_ANALOG_THRESHOLD_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_ANALOG_MAP_MIN_callback(TRegister* reg, uint16_t val) {
  analog_map_min = val;
  if (analog_map_min > 5000) {
    analog_map_min = 5000;
  }
  saveSettings();
  return val;
}
uint16_t get_ANALOG_MAP_MIN_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_ANALOG_MAP_MAX_callback(TRegister* reg, uint16_t val) {
  analog_map_max = val;
  if (analog_map_max > 5000) {
    analog_map_max = 5000;
  }
  saveSettings();
  return val;
}
uint16_t get_ANALOG_MAP_MAX_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_ANALOG_RANGE_MIN_callback(TRegister* reg, uint16_t val) {
  analog_range_min = val;
  saveSettings();
  return val;
}
uint16_t get_ANALOG_RANGE_MIN_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_ANALOG_RANGE_MAX_callback(TRegister* reg, uint16_t val) {
  analog_range_max = val;
  saveSettings();
  return val;
}
uint16_t get_ANALOG_RANGE_MAX_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t set_FLOWRATE_SECONDS_callback(TRegister* reg, uint16_t val) {
  flowRate_seconds = val;
  saveSettings();
  return val;
}
uint16_t get_FLOWRATE_SECONDS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}

uint16_t set_ADD_TOTAL_UNITS_callback(TRegister* reg, uint16_t val) {
  addUnits(val * 10);
  saveData();
  return val;
}
uint16_t get_ADD_TOTAL_UNITS_callback(TRegister* reg, uint16_t val) {
  //Serial.print("value = ");
  //Serial.println(val);
  return val;
}


uint16_t set_RESET_callback(TRegister* reg, uint16_t val) {
  if (val == 0) {
    Serial.println("RESET TYPE 0 - SAVE & RESTART ONLY");
    saveData();
    ESP.restart();  //reset ESP32
  }
  if (val == 1) {
    Serial.println("RESET TYPE 1 - RESET SETTINGS ONLY TO DEFAULTS");
    stop();
    saveData();
    resetDefaults();
  }
  if (val == 2) {
    Serial.println("RESET TYPE 2 - RESET DATA ONLY");
    stop();
    resetData();
  }
  if (val == 3) {
    Serial.println("RESET TYPE 3 - RESET WIFI ONLY AND RESTART");
    saveData();
    esp_wifi_restore();  // clear wifi credentials
    ESP.restart();       //reset ESP32
  }
  if (val == 4) {
    Serial.println("RESET TYPE 4 - RESET ALL TO DEFAULTS + WIFI AND RESTART");
    resetDefaults();
    resetData();
    esp_wifi_restore();  // clear wifi credentials
    ESP.restart();       //reset ESP32
  }
  if (val == 5) {
    Serial.println("RESET TYPE 5 - SAVE DATA AND RESTART WITH PORTAL (OTA)");
    boot_mode = 1;
    saveData();
    saveBootMode();
    ESP.restart();  //reset ESP32
  }
  if (val > 5) {
    Serial.print("Invalid reset type ");
    Serial.println(val);
    Serial.print("\n");
  }
  return val;
}


uint16_t get_RESET_callback(TRegister* reg, uint16_t val) {
  Serial.println("RESET TYPE 0 - RESTART ONLY");
  Serial.println("RESET TYPE 1 - RESET SETTINGS ONLY TO DEFAULTS");
  Serial.println("RESET TYPE 2 - RESET DATA ONLY");
  Serial.println("RESET TYPE 3 - RESET WIFI ONLY AND RESTART");
  Serial.println("RESET TYPE 4 - RESET ALL TO DEFAULTS + WIFI AND RESTART");
  Serial.println("RESET TYPE 5 - SAVE DATA AND RESTART WITH BLOCKING PORTAL (OTA)");
  return val;
}
