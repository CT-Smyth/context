//-----------------------------------------------------------------------------------------------
void stop() {
  Serial.println("STOP");
  output_pulses_todo = 0;
  pulse_on_in_progress = false;
  pulse_off_in_progress = false;
  units_queued = 0;
  active_flow = 0;
  setRelay(0);
  setHorn(0);
}


void addUnits(u_long val) {

  u_long lftime = u_long((lifetime_units_MSW * 65536) + lifetime_units_LSW);
  lftime = lftime + (val);

  u_long mswToAdd = lftime / 65536;
  u_long lswToAdd = (lftime - (mswToAdd * 65536));  //remainder

  lifetime_units_LSW = uint16_t(lswToAdd);
  lifetime_units_MSW = uint16_t(mswToAdd);
}


int senseButton() {
  int debounce = 0;

  for (int t = 0; t < 11; t++) {
    if (digitalRead(button_in_pin) == 0) {
      debounce++;
    }
  }

  if (debounce > 8) {
    if (buttonTime == 0) buttonTime = millis();

    if (button_state == 0) {
      button_state = 1;
      if (millis() < displayTime) {
        display_mode++;
      }
      if (display_mode > 5) {
        display_mode = 0;
      }
      default_display_mode = display_mode;
      saveDisplayMode();
      displayTime = millis() + (displayTimeout * 1000);
    }

    return (int((millis() - buttonTime) / 1000));
  } else {
    buttonTime = 0;
    button_state = 0;
    return 0;
  }
}

void checkButton() {
  buttonSeconds = senseButton();
  if (buttonSeconds == 1) {
    manPage();
    saveData();
    saveSettings();
  }
  if (buttonSeconds == 3) {
    //Serial.println("cancel job in progress");
    stop();
  }
  if (buttonSeconds == 10) {
    //Serial.println("Priming....");
    output_pulses_todo = output_pulses_for_prime;
  }
  if (buttonSeconds == 30) {
    Serial.println("RESET TYPE 3 - SAVE, ERASE WIFI, RESTART ONLY");
    saveData();
    esp_wifi_restore();  // clear wifi credentials
    ESP.restart();       //reset ESP32
  }
}


uint output_pulser() {
  u_long millis_now = millis();
  if (output_pulses_todo > 0) {
    //Serial.print("pulses to do : ");
    //Serial.println(output_pulses_todo);

    if ((pulse_on_in_progress == false) && (pulse_off_in_progress == false)) {  //start pulse
      setRelay(1);
      //Serial.println("pulse on");
      pulse_on_in_progress = true;
      pulse_off_in_progress = false;
      PulseTime = millis_now + output_pulse_ms;
    }

    if ((pulse_on_in_progress == true) && (millis_now > PulseTime)) {  //end 'on' segment of pulse
      setRelay(0);
      //Serial.println("pulse off");
      pulse_on_in_progress = false;
      pulse_off_in_progress = true;
      PulseTime = millis_now + (output_pulse_off_ms);
    }

    if ((pulse_off_in_progress == true) && (millis_now > PulseTime)) {  //end 'off' segment of pulse
      setRelay(0);
      //Serial.println("End pulse");
      pulse_on_in_progress = false;
      pulse_off_in_progress = false;
      output_pulses_todo = output_pulses_todo - 1;
      output_pulses++;
    }

    //Serial.print("\npulses to do : ");
    //Serial.print(output_pulses_todo);
  }  //else {
  //stop();
  //}
  return output_pulses_todo;
}


void setRelay(uint16_t newState) {
  bool pinstate = bool(newState);
  //Serial.print("SetRelay state :");           //DEBUG
  //Serial.println(pinstate);                   //DEBUG
  pinstate = pinstate + relay_set_by_modbus;  // override 'OFF' state if modbus coil control is 'ON'
  digitalWrite(relayPin, pinstate);
  mBus.Ists(RELAY_COIL, newState);
  relay_status = newState;

  if (system_initialized == true) {
    saveRelayState();
  }
}


void setHorn(uint16_t newState) {
  bool pinstate = bool(newState);
  //Serial.print("SetHorn state :");           //DEBUG
  //Serial.println(pinstate);                  //DEBUG
  pinstate = pinstate + horn_set_by_modbus;  // override 'OFF' state if modbus coil control is 'ON'
  digitalWrite(hornPin, pinstate);
  mBus.Ists(HORN_COIL, newState);
  horn_status = newState;

  if (system_initialized == true) {
    saveHornState();
  }
}

uint16_t getAnalog() {
  // if analog_range_max != 0 then the raw analog data will be mapped to the ouput data as follows:
  // from the range {range_min -> range_max} map to: {map_min -> map_max} Note that map_min and map_max are limited to 0-500.
  // the algorithm is:   for the value s, within range a1->a2, map to range b1->b2. result = b1+(s-a1)*(b2-b1)/(a2-a1)
  // note that the map-to values (b1->b2) can be reversed to invert the input so that higher inputs result in lower outputs.
  long analogAvg = 0;
  analogSetPinAttenuation(analog_pin, ADC_11db);  // set ADC range to 2.6v for more sensitivity
  for (int t = 0; t < 10; t++) {
    analogAvg = analogAvg + analogRead(analog_pin);
  }
  analogAvg = analogAvg / 10;
  analogAvg = (analogAvg + analog_sense) / 2;

  if (analog_range_max != 0) {  // Map the analog data to analog_map
    analogAvg = map(long(analogAvg), long(analog_range_min), long(analog_range_max), long(analog_map_min), long(analog_map_max));
  }
  return uint16_t(analogAvg);
}

void configure_interrupt_type() {
  //Serial.println("Changing interrupt type...");
  if (pin_mode == 0) {
    attachInterrupt(digitalPinToInterrupt(pulse_in_pin), pin_interrupt, RISING);
  }
  if (pin_mode == 1) {
    attachInterrupt(digitalPinToInterrupt(pulse_in_pin), pin_interrupt, FALLING);
  }
  if (pin_mode == 2) {
    attachInterrupt(digitalPinToInterrupt(pulse_in_pin), pin_interrupt, CHANGE);
  }
  if (pin_mode > 2) {
    detachInterrupt(digitalPinToInterrupt(pulse_in_pin));  // Disable the interrupt
  }
}

void resetDefaults() {  //reset to factory defaults
  Serial.println("Loading default settings");
  valid_flash = 12345;

  mode = 1;
  pin_mode = 0;
  sensor_pulses_per_unit = 150;
  save_interval = 1000;
  horn_units = 200;
  relay_units = 300;
  horn_seconds = 900;
  relay_seconds = 1500;
  flow_stop_seconds = 5;
  relay_latchmode = 0;
  horn_latchmode = 0;
  output_pulse_ms = 200;
  output_pulse_off_ms = 200;
  units_per_event = 10;
  output_pulses_per_trigger = 0;
  output_pulses_for_prime = 1;
  min_flow = 0;
  analog_threshold = 0;
  analog_map_min = 0;
  analog_map_max = 4095;
  analog_range_min = 0;
  analog_range_max = 4095;
  flowRate_seconds = 60;

  saveData();
  saveSettings();
  system_initialized = false;
  syncModbusSettings();
  system_initialized = true;
  manPage();
}

void resetData() {  //reset data only
  Serial.println("Resetting Stored Data");
  valid_flash = 12345;

  lifetime_units_from_flash_MSW = 0;
  lifetime_units_from_flash_LSW = 0;

  lifetime_units_MSW = 0;
  lifetime_units_LSW = 0;
  this_flow_units = 0;
  this_flow_duration = 0;
  largest_flow_units = 0;
  longest_flow_duration = 0;
  flow_rate = 0;
  output_pulses_todo = 0;
  sensor_pulses = 0;
  analog_sense = 0;
  units_queued = 0;
  flow_time_since_boot = 0;
  units_since_boot = 0;
  output_pulses = 0;

  saveData();
  syncModbusData();
  //manPage();
}

void dataPage() {
#ifdef SERIAL_VERBOSE
  Serial.print("\nHorn:");
  Serial.print(digitalRead(hornPin));
  Serial.print("  Relay:");
  Serial.print(digitalRead(relayPin));
  Serial.print("  Flow:");
  Serial.print(this_flow_units);
  Serial.print("  Time:");
  Serial.print(this_flow_duration);
  Serial.print("  Anlog:");
  Serial.print(analog_sense);
  Serial.print("  ToDo:");
  Serial.print(output_pulses_todo);
  Serial.print("  Total:");
  Serial.print(u_long((lifetime_units_MSW * 65536) + lifetime_units_LSW));
  Serial.print("  B-Units:");
  Serial.print(units_since_boot);
  Serial.print("  B-Flows:");
  Serial.println(flow_time_since_boot);
#endif
}

void manPage() {
#ifdef SERIAL_VERBOSE
  Serial.print("\n---------------------------------------------------------------------------------------");
  Serial.print("\n--Modbus IP enabled Process Controller ");
  Serial.print(VERSION_NUMBER);
  Serial.print("V5.7 MIT LIC CT Smyth cliff.smyth@tutanota.com--");
  Serial.print("\n---------------------------------------------------------------------------------------");
  Serial.print("\n\nListens for modbus on port 502 - Slave ID not important");
  Serial.print("\n\nRegister <numbers> are given as ZERO REFERENCED offsets to zero");
  Serial.print("\n\naction button (0/boot button on some devices) wakes display & cycles through display");
  Serial.print("\nmodes. 1 sec save data, 10 sec runs pulses_for_prime job, 30 sec for del wifi and reset.");
  Serial.print("\npressing button during boot startup (not holding during reset) launches config portal");
  Serial.print("\n---------------------------------------------------------------------------------------");
  Serial.print("\nMAC Address, IP and Hostname: ");
  Serial.print(WiFi.macAddress());
  Serial.print("----");
  Serial.print(WiFi.localIP().toString());
  Serial.print("----");
  Serial.print(WiFi.getHostname());
  Serial.print("\n---------------------------------------------------------------------------------------");
  Serial.print("\n\nSet HREG 99 as follows to RESET:");
  Serial.print("\nRESET TYPE 0 - RESTART ONLY");
  Serial.print("\nRESET TYPE 1 - RESET SETTINGS ONLY TO DEFAULTS");
  Serial.print("\nRESET TYPE 2 - RESET DATA ONLY");
  Serial.print("\nRESET TYPE 3 - RESET WIFI ONLY AND RESTART");
  Serial.print("\nRESET TYPE 4 - RESET ALL TO DEFAULTS + WIFI AND RESTART");
  Serial.print("\nRESET TYPE 5 - SAVE DATA AND RESTART WITH FULL BLOCKING WEB PORTAL (FOR OTA UPGRADE)");

  Serial.print("\n\nMODE setting descriptions:");
  Serial.print("\n0 is relay on pulse output, trigger horn on flow over x_seconds or y_units w/o flow_stop_seconds of pause");
  Serial.print("\n1 is trigger on flow over x_seconds or y_units without flow_stop_seconds of pause");
  Serial.print("\n2 is trigger on total flow time only since reset or boot (flow_time_since_boot)");
  Serial.print("\n3 is trigger on total flow units only since reset or boot (total_units_since_boot)");
  Serial.print("\n4 is trigger on total flow units or time  since reset or boot (total_units_since_boot, flow_time_since_boot)");
  Serial.print("\n5 is trigger on failure to achieve x_units in y_seconds while flowing (low flow)");
  Serial.print("\n6 is count / time only");

  Serial.print("\n\nPIN_MODE setting descriptions:");
  Serial.print("\npin modes based on pulses:");
  Serial.print("\n 0 is pulse rising");
  Serial.print("\n 1 is pulse falling");
  Serial.print("\n 2 is pulse change");

  Serial.print("\n\npin modes based on pin state / time");
  Serial.print("\n 3 is pin on (STEADY STATE) units are counted by mS per unit, mS are configured in the pulses_per_unit register");
  Serial.print("\n 4 is pin off (STEADY STATE) units are counted by mS per unit  mS are configured in the pulses_per_unit register");
  Serial.print("\nin modes 3,4,7,8 input pulses are elapsed mS. For units in seconds, set sensor_pulses_per_unit = 1000");

  Serial.print("\n\npin modes based on analog input");
  Serial.print("\n 5- analog signal 2.6v = 4095 (uses analog, threshold, min, and max settings to to set input pulse rate based on signal");
  Serial.print("\n 6- 2.6v=4095->mapped. 'cycle' starts when analog < threshold and stops when analog > limit. pulses per mS as in mode 3/4");
  Serial.print("\n 7- 2.6v=4095->mapped. 'cycle' starts when analog < threshold and stops when analog > limit. pulses based on analog as in (5)");

  Serial.print("\n\nNOTES FOR ANALOG SETTINGS -- !!CAUTION: If using mode 5 OR 7 the analog_map_max must be !=0 and mapping applied.");
  Serial.print("\n analog data is used as is (0-4095) if analog_range_max is set to zero.");
  Serial.print("\n if analog_range_max != 0 then the raw analog data will be mapped to the ouput data as follows:");
  Serial.print("\n from the range {range_min -> range_max} map to: {map_min -> map_max} Note that map_min and map_max are limited to 0-5000.");
  Serial.print("\n the algorithm is:   for the value s, within range a1->a2, map to range b1->b2. result = b1+(s-a1)*(b2-b1)/(a2-a1)");
  Serial.print("\n note that the map-to values (b1->b2) can be reversed to invert the input so that higher inputs result in lower outputs.");

  Serial.print("\n\nOTHER NOTES:");
  Serial.print("\nLATCH_MODE:   0 = non latch, 1 = persists, 2 = latches on, 3 = latches on, persists");
  Serial.print("\nflow_limit sets a minimum flow rate to count. flows below flow_limit will not be counted.");
  Serial.print("\nfor pulses in mode 1 (output_pulses_per_trigger != 0)");
  Serial.print("\npulses out = ((sensor_pulses / sensor_pulses_per_unit) / units_per_event) * output_pulses_per_trigger");

  Serial.print("\n\nAPPLICATION NOTES: the analog range is set by default to 11db which gives a range of up to 2.6v");
  Serial.print("\nfor 0-20ma sensing use a 100ohm resistor between GND and the ground end of the signal wire. attach the analog sense pin");
  Serial.print("\nto the sensor end of the resistor. this will give about 2v at 20mA, and read about 3150 on the ADC. To measure voltage,");
  Serial.print("\nmake a voltage divider (0-5v IN----1k--ADC--1k---GND),(0-12v IN----5k--ADC--1.2k---GND),(75v IN----68k--ADC--2.2k---GND)");
  Serial.print("\n\---------------------------------------------------------------------------------------");

  Serial.print("\n\nHREGS:             <HREG Number> (offset 0 )");
  Serial.print("\nOUTPUT_PULSES_TO_ADD    <0>-  output pulses to add to current job");
  Serial.print("\nUNITS_SINCE_BOOT        <1>-                 write to ADD UNITS to total or zero--> ");
  Serial.print(units_since_boot);
  Serial.print("\nFLOW_TIME_SINCE_BOOT    <2>-                 write to ADD UNITS to total or zero--> ");
  Serial.print(flow_time_since_boot);
  Serial.print("\nSHOW_DISPLAY            <3>-  minutes (1000 max) to enable display ");
  Serial.print("\nDISPLAY_MODE            <3>-  set 0-5 (5 is off) for deafault display mode ");
  Serial.print("\n\n\n---------------------The below settings will be saved on setting----------------------");
  Serial.print("\nMODE                    <9>-                                      Operating Mode--> ");
  Serial.print(mode);
  Serial.print("\nPIN_MODE                <10>-                                          pin mode --> ");
  Serial.print(pin_mode);
  Serial.print("\nSENSOR_PULSES_PER_UNIT  <11>-          Input pulses for each output unit counted--> ");
  Serial.print(sensor_pulses_per_unit);
  Serial.print("\nSAVE_INTERVAL           <12>-        Save interval. units between saves to flash--> ");
  Serial.print(save_interval);
  Serial.print("\nHORN_UNITS              <13>-                       units to trigger HORN output--> ");
  Serial.print(horn_units);
  Serial.print("\nRELAY_UNITS             <14>-                      units to trigger RELAY output--> ");
  Serial.print(relay_units);
  Serial.print("\nHORN_SECONDS            <15>-       seconds without reset to trigger HORN output--> ");
  Serial.print(horn_seconds);
  Serial.print("\nRELAY_SECONDS           <16>-      seconds without reset to trigger RELAY output--> ");
  Serial.print(relay_seconds);
  Serial.print("\nFLOW_STOP_SECONDS       <17>-   resets flow and time counts, non-latched outputs--> ");
  Serial.print(flow_stop_seconds);
  Serial.print("\nRELAY_LATCHMODE         <18>-0=non latch, 1=persists, 2=latch, 3=latch, persists--> ");
  Serial.print(relay_latchmode);
  Serial.print("\nHORN_LATCHMODE          <19>-0=non latch, 1=persists, 2=latch, 3=latch, persists--> ");
  Serial.print(horn_latchmode);
  Serial.print("\nOUTPUT_PULSE_MS         <20>-                      milliseconds for pulse length--> ");
  Serial.print(output_pulse_ms);
  Serial.print("\nOUTPUT_PULSE_OFF_MS     <21>-             milliseconds for pulse length off time--> ");
  Serial.print(output_pulse_off_ms);
  Serial.print("\nUNITS_PER_EVENT         <22>-               Units to trigger each output 'event'--> ");
  Serial.print(units_per_event);
  Serial.print("\nOUTPUT_PULSES_PER_TRIGGER <23>-      (0=off) pulses to que for each output event--> ");
  Serial.print(output_pulses_per_trigger);
  Serial.print("\nOUTPUT_PULSES_FOR_PRIME <24>-      Pulses to add to current job for pump priming--> ");
  Serial.print(output_pulses_for_prime);
  Serial.print("\nFLOW_LIMIT  (MODE 2or3) <25>-  Input_pulses=0 unless flow/time is met (0 = null)--> ");
  Serial.print(min_flow);
  Serial.print("\nANALOG_THRESHOLD        <26>-                        0-4096 threshold to trigger--> ");
  Serial.print(analog_threshold);
  Serial.print("\nANALOG_MIN              <27>-        Analog map outpiut range. limited to 0-5000--> ");
  Serial.print(analog_map_min);
  Serial.print("\nANALOG_MAX              <28>-        Analog map outpiut range. limited to 0-5000--> ");
  Serial.print(analog_map_max);
  Serial.print("\nANALOG_RANGE_MIN        <29>-                  constrain analoginput values here--> ");
  Serial.print(analog_range_min);
  Serial.print("\nANALOG_RANGE_MAX        <30>-                  constrain analoginput values here--> ");
  Serial.print(analog_range_max);
  Serial.print("\nFLOWRATE_SECONDS        <31>- number of seconds to use for calculating flow rate--> ");
  Serial.print(flowRate_seconds);
  Serial.print("\nADD_TOTAL_UNITS         <32>-   units to add for adjusting total units displayed--> ");

  Serial.print("\n\nRESET (TYPE 0-3)        <99>-           Set to initiate reset. see above for details");

  Serial.print("\n\nISTSs:         <ISTS Number> (offset 0 )");
  Serial.print("\nHORN_COIL_ISTS           <0>-                                 Horn status -->");
  Serial.print(digitalRead(hornPin));
  Serial.print("\nRELAY_COIL_ISTS          <1>-                                 Horn status -->");
  Serial.print(digitalRead(relayPin));

  Serial.print("\n\nCOILs:         <COIL Number> (offset 0 )");
  Serial.print("\nHORN_COIL               <0>-             Alarm control write 0 to clear latch");
  Serial.print("\nRELAY_COIL              <1>-             Relay control write 0 to clear latch");
  Serial.print("\nSTOP_COIL               <9>- write->stops current job  read->prints this page");
  Serial.print("\nSAVE_COIL              <10>-read->saves current data    write->saves settings");

  Serial.print("\n\nIREGs:         <IREG Number> (offset 0 )");
  Serial.print("\nLIFETIME_UNITS_MSW      <0>-upper 16 bits of total lifetime output pulses--> ");
  Serial.print(lifetime_units_MSW);
  Serial.print("\nLIFETIME_UNITS_LSW      <1>-lower 16 bits of total lifetime output pulses--> ");
  Serial.print(lifetime_units_LSW);
  Serial.print("\nTHIS_FLOW_UNITS         <2>-                           current flow units--> ");
  Serial.print(this_flow_units);
  Serial.print("\nTHIS_FLOW_DURATION      <3>-                        current flow duration--> ");
  Serial.print(this_flow_duration);
  Serial.print("\nLARGEST_FLOW_UNITS      <4>-                       most units in one flow--> ");
  Serial.print(largest_flow_units);
  Serial.print("\nLONGEST_FLOW_DURATION   <5>-                     Longest logged flow time--> ");
  Serial.print(longest_flow_duration);
  Serial.print("\nFLOW_RATE               <6>-   (pulses/sec / pulsesPerUnit) * flowRateSec--> ");
  Serial.print(flow_rate);
  Serial.print("\nOUTPUT_PULSES_TODO      <7>-              Number of output pulses pending--> ");
  Serial.print(output_pulses_todo);
  Serial.print("\nSENSOR_PULSES           <8>-                      UNCOUNTED sensor pulses--> ");
  Serial.print(sensor_pulses);
  Serial.print("\nANALOG_SENSE            <9>-                                 Analog input--> ");
  Serial.print(analog_sense);
  Serial.print("\nUNITS_QUEUED            <10>-              Units not yet consumed by jobs--> ");
  Serial.print(units_queued);
  Serial.print("\nWIFI_RSSI                <11>-                                            --> ");
  Serial.print(WiFi.RSSI());
  Serial.print("\nUPTIME_HRS               <12>-                                            --> ");
  Serial.print(millis() / 3600000);
  Serial.print("\n\n---------------------------------------------------------------------------------------");

  Serial.print("\n                     LIFETIME LOGGED UNITS >>> : ");
  Serial.print(u_long((lifetime_units_MSW * 65536) + lifetime_units_LSW));
  Serial.print(" units");
  Serial.print("\n---------------------------------------------------------------------------------------\n");
#endif
}

void displayData() {
#ifdef MY_OLED
  obdFill(&oled, OBD_WHITE, 1);

  if ((millis() < displayTime) || ((millis() / 3600000) < 1)) {
    char szTemp[32];
    u_long lftime = u_long((lifetime_units_MSW * 65536) + lifetime_units_LSW);
    int lftime_d = lftime / 10;
    lftime_d = lftime - (lftime_d * 10);  // get just the decimal part
    lftime = lftime / 10;                 // just get the whole part
    int lftimeShort = lftime % 1000;

    int thisFl = int(this_flow_units);
    int thisFl_d = thisFl / 10;
    thisFl_d = thisFl - (thisFl_d * 10);  // get just the decimal part
    thisFl = thisFl / 10;                 // just get the whole part

    int thisDur = int(this_flow_duration);

    int rate = int(flow_rate);
    int rate_d = rate / 10;
    rate_d = rate - (rate_d * 10);  // get just the decimal part
    rate = rate / 10;               // just get the whole part

    int anl = int(analog_sense);

    int uptime = millis() / 3600000;  //uptime hours

    uint8_t IP0 = WiFi.localIP()[0];
    uint8_t IP1 = WiFi.localIP()[1];
    uint8_t IP2 = WiFi.localIP()[2];
    uint8_t IP3 = WiFi.localIP()[3];

    if (ISR_flowrate_pulses != 0) {  // flow animantion
      animation = !animation;
    }
    textScroll = textScroll + 4;
    if (textScroll > 20) {
      textScroll = 0;
    }

#ifdef C3_42_OLED  //integrated 0.42" module with offsets

    if (display_mode == 0) {
      sprintf(szTemp, "%lu.%d", lftime, lftime_d);
      obdWriteString(&oled, 0, 32, 16, szTemp, FONT_8x8, OBD_BLACK, 1);

      sprintf(szTemp, "cur:%d.%d", thisFl, thisFl_d);
      obdWriteString(&oled, 0, 36, 24, szTemp, FONT_6x8, OBD_BLACK, 1);

      sprintf(szTemp, "sec:%d", thisDur);
      obdWriteString(&oled, 0, 36, 32, szTemp, FONT_6x8, OBD_BLACK, 1);

      sprintf(szTemp, "flw:%d.%d", rate, rate_d);
      obdWriteString(&oled, 0, 36, 40, szTemp, FONT_6x8, OBD_BLACK, 1);

      if (relay_status == 1) {
        obdWriteString(&oled, 0, 30, 40, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
      if (horn_status == 1) {
        obdWriteString(&oled, 0, 94, 40, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
    }

    if (display_mode == 1) {
      sprintf(szTemp, "%lu.%d", lftime, lftime_d);
      obdWriteString(&oled, 0, 32, 16, szTemp, FONT_8x8, OBD_BLACK, 1);

      if (animation) {
        sprintf(szTemp, "+%d sec", thisDur);
      } else {
        sprintf(szTemp, "*%d sec", thisDur);
      }
      obdWriteString(&oled, 0, 36, 24, szTemp, FONT_6x8, OBD_BLACK, 1);

      sprintf(szTemp, "flw:%d.%d", rate, rate_d);
      obdWriteString(&oled, 0, 36, 40, szTemp, FONT_6x8, OBD_BLACK, 1);

      if (relay_status == 1) {
        obdWriteString(&oled, 0, 30, 40, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
      if (horn_status == 1) {
        obdWriteString(&oled, 0, 94, 40, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
    }

    if (display_mode == 2) {
      sprintf(szTemp, "RSSI %ddB", WiFi.RSSI());
      obdWriteString(&oled, 0, 30, 16, szTemp, FONT_6x8, OBD_BLACK, 1);
      sprintf(szTemp, "up:%dhr", uptime);
      obdWriteString(&oled, 0, 30, 24, szTemp, FONT_6x8, OBD_BLACK, 1);
      sprintf(szTemp, "%s", WiFi.getHostname());
      obdWriteString(&oled, textScroll, 30, 32, szTemp, FONT_6x8, OBD_BLACK, 1);
      sprintf(szTemp, " x.x.%d.%d", IP2, IP3);
      obdWriteString(&oled, 0, 30, 40, szTemp, FONT_6x8, OBD_BLACK, 1);
    }

    if (display_mode == 3) {
      obdWriteString(&oled, 0, 40, 16, (char *)"LEVEL:", FONT_8x8, OBD_BLACK, 1);

      sprintf(szTemp, "%d", anl);
      obdWriteString(&oled, 0, 36, 32, szTemp, FONT_12x16, OBD_BLACK, 1);

      if (relay_status == 1) {
        obdWriteString(&oled, 0, 30, 40, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
      if (horn_status == 1) {
        obdWriteString(&oled, 0, 94, 40, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
    }

    if (display_mode == 4) {
      sprintf(szTemp, "%d.%d", lftimeShort, lftime_d);
      obdWriteString(&oled, 0, 32, 16, szTemp, FONT_12x16, OBD_BLACK, 1);

      if (animation) {
        sprintf(szTemp, "+%d.%d", rate, rate_d);
      } else {
        sprintf(szTemp, "*%d.%d", rate, rate_d);
      }
      obdWriteString(&oled, 0, 32, 32, szTemp, FONT_12x16, OBD_BLACK, 1);

      if (relay_status == 1) {
        obdWriteString(&oled, 0, 30, 48, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
      if (horn_status == 1) {
        obdWriteString(&oled, 0, 94, 48, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
    }

    if (display_mode > 4) {
      obdFill(&oled, OBD_WHITE, 1);
    }

#else  //generic OLED module

    if (display_mode == 0) {

      sprintf(szTemp, "%lu.%d", lftime, lftime_d);
      obdWriteString(&oled, 0, 0, 0, szTemp, FONT_12x16, OBD_BLACK, 1);

      sprintf(szTemp, "cur:%d.%d", thisFl, thisFl_d);
      obdWriteString(&oled, 0, 18, 24, szTemp, FONT_6x8, OBD_BLACK, 1);

      sprintf(szTemp, "%d seconds", thisDur);
      obdWriteString(&oled, 0, 18, 32, szTemp, FONT_6x8, OBD_BLACK, 1);

      sprintf(szTemp, "flw:%d.%d", rate, rate_d);
      obdWriteString(&oled, 0, 18, 48, szTemp, FONT_6x8, OBD_BLACK, 1);

      sprintf(szTemp, "level: %d", anl);
      obdWriteString(&oled, 0, 18, 56, szTemp, FONT_6x8, OBD_BLACK, 1);

      if (relay_status == 1) {
        obdWriteString(&oled, 0, 0, 56, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
      if (horn_status == 1) {
        obdWriteString(&oled, 0, 119, 56, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
    }

    if (display_mode == 1) {
      sprintf(szTemp, "%lu.%d", lftime, lftime_d);
      obdWriteString(&oled, 0, 0, 0, szTemp, FONT_12x16, OBD_BLACK, 1);

      if (animation) {
        sprintf(szTemp, "+ %d seconds", thisDur);
      } else {
        sprintf(szTemp, "* %d seconds", thisDur);
      }
      obdWriteString(&oled, 0, 24, 24, szTemp, FONT_6x8, OBD_BLACK, 1);

      obdWriteString(&oled, 0, 0, 40, (char *)"flow:", FONT_6x8, OBD_BLACK, 1);

      sprintf(szTemp, "%d.%d", rate, rate_d);
      obdWriteString(&oled, 0, 36, 40, szTemp, FONT_12x16, OBD_BLACK, 1);

      if (relay_status == 1) {
        obdWriteString(&oled, 0, 0, 56, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
      if (horn_status == 1) {
        obdWriteString(&oled, 0, 119, 56, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
    }

    if (display_mode == 2) {

      sprintf(szTemp, "RSSI %ddB", WiFi.RSSI());
      obdWriteString(&oled, 0, 30, 0, szTemp, FONT_6x8, OBD_BLACK, 1);

      sprintf(szTemp, "uptime %dhr", uptime);
      obdWriteString(&oled, 0, 30, 8, szTemp, FONT_6x8, OBD_BLACK, 1);

      sprintf(szTemp, "%s", WiFi.getHostname());
      obdWriteString(&oled, 0, 16, 24, szTemp, FONT_8x8, OBD_BLACK, 1);
      sprintf(szTemp, "%d.%d.%d.%d", IP0, IP1, IP2, IP3);
      obdWriteString(&oled, 0, 8, 40, szTemp, FONT_8x8, OBD_BLACK, 1);
    }

    if (display_mode == 3) {
      obdWriteString(&oled, 0, 0, 24, (char *)"level:", FONT_6x8, OBD_BLACK, 1);
      sprintf(szTemp, "%d", anl);
      obdWriteString(&oled, 0, 36, 24, szTemp, FONT_12x16, OBD_BLACK, 1);

      if (relay_status == 1) {
        obdWriteString(&oled, 0, 0, 56, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
      if (horn_status == 1) {
        obdWriteString(&oled, 0, 119, 56, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
    }

    if (display_mode == 4) {
      sprintf(szTemp, "%d.%d", lftimeShort, lftime_d);
      obdWriteString(&oled, 0, 32, 0, szTemp, FONT_12x16, OBD_BLACK, 1);

      if (animation) {
        sprintf(szTemp, "+%d.%d", rate, rate_d);
      } else {
        sprintf(szTemp, "*%d.%d", rate, rate_d);
      }
      obdWriteString(&oled, 0, 32, 24, szTemp, FONT_12x16, OBD_BLACK, 1);

      if (relay_status == 1) {
        obdWriteString(&oled, 0, 30, 48, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
      if (horn_status == 1) {
        obdWriteString(&oled, 0, 94, 48, (char *)"*", FONT_6x8, OBD_BLACK, 1);
      }
    }

    if (display_mode > 4) {
      obdFill(&oled, OBD_WHITE, 1);
    }
#endif
  }
#endif
}


// obdDrawLine(&oled, 0, 0, 0, OLED_HEIGHT - 1, OBD_BLACK, 0);
// obdDrawLine(&oled, 0, OLED_HEIGHT - 1, OLED_WIDTH - 1, OLED_HEIGHT - 1, OBD_BLACK, 0);
// obdDrawLine(&oled, OLED_WIDTH - 1, OLED_HEIGHT - 1, OLED_WIDTH - 1, 0, OBD_BLACK, 0);
// obdDrawLine(&oled, OLED_WIDTH - 1, 0, 0, 0,  OBD_BLACK, 0);

// obdDrawLine(&oled, 0, 0, 0, 39, OBD_BLACK, 0);
// obdDrawLine(&oled, 0, 39, 71, 39, OBD_BLACK, 0);
// obdDrawLine(&oled, 71,39, 71, 0, OBD_BLACK, 0);
// obdDrawLine(&oled, 71, 0, 0, 0, OBD_BLACK, 0);


// void lightSleep(uint64_t time_in_ms) {
// #ifdef HAL_ESP32_HAL_H_
//   esp_sleep_enable_timer_wakeup(time_in_ms * 1000);
//   esp_light_sleep_start();
// #else
//   delay(time_in_ms);
// #endif
// }
