// ESP32 based process controller.

// Especially designed for interfacing with common waterflow sensors, 0-20ma (4-20ma) level sensors, etc and controlling valve shutoffs, pumps, etc.
// Analog, switch, and pulse input types. Controls based on levels, time, and pulses. 2 Dedicated outputs - 1 for "horn" or whatever, and one for relay and pulse output.
// Communicates over modbus-IP for industrial control systems. Designed to be used with or without I2C OLED displays based on the SD1306 architecture.
// Could easily be adapted to modbus serial without a library change.
// Supports hidden wifi, OTA updates, and massive configurability.

// Pretty rough but robust and reliable - Ill work on cleaning it up for readability as i go.
// NOTE: ESP-IDF version used ** DOES NOT WORK ON WIFI CHANNELS ABOVE 13 *** also ESP32s are 2.4ghz only

// Listens for modbus on port 502 - Slave ID not important
// HOST_NAME = "esp32-xxxxxx";  //set hostname to <hostname>.local
// web portal at 192.168.4.1 if no wifi. 
// if connected to an AP, portal at above or at IP or at esp32-xxxxxx.local.  local wifi password is configureMe!

// double reset for portal:
// to force the config portal reset the unit within 10 seconds after the 5 quick status flashes on startup.
// the system will restart with the LED on for a couple of seconds to show that the portal has been started, 
// then normal operation will start (non-blocking portal remains active)

// cycle through default display modes by using the action button (0/boot button on some devices)
// or by repeating the double reset sequence until the desired screen is displayed.

// button wakes display & cycles through display modes. 1 sec saves data and 
// sends manpage to serial. 10 seconds runs priming pulses, 30 seconds for erase wifi and reset.

//connect to portal (192.168.4.1) to set up wifi, get IP address, hostname, etc

// Must be compiled with "minimal spiffs" for OTA to work

//________________________________TO__DO_________________________________


//  Set HREG 99 as follows to RESET: 
//  RESET TYPE 0 - RESTART ONLY 
//  RESET TYPE 1 - RESET SETTINGS ONLY TO DEFAULTS 
//  RESET TYPE 2 - RESET DATA ONLY 
//  RESET TYPE 3 - RESET WIFI ONLY AND RESTART 
//  RESET TYPE 4 - RESET ALL TO DEFAULTS + WIFI AND RESTART
//  RESET TYPE 5 - SAVE DATA AND RESTART WITH WEB PORTAL (FOR OTA UPGRADE)

//   MODE setting descriptions: 
//  0 is relay on pulse output, trigger horn on flow over x_seconds or y_units without flow_stop_seconds of pause
//  1 is trigger on flow over x_seconds or y_units without flow_stop_seconds of pause 
//  2 is trigger on total flow time only since reset or boot (flow_time_since_boot) 
//  3 is trigger on total flow units only since reset or boot (total_units_since_boot) 
//  4 is trigger on total flow units or time  since reset or boot (total_units_since_boot, flow_time_since_boot) 
//  5 is trigger on failure to achieve x_units in y_seconds while flowing (low flow) 
//  6 is count / time only 

//   PIN_MODE setting descriptions: 
//  pin modes based on pulses: 
//   0 is pulse rising 
//   1 is pulse falling 
//   2 is pulse change 

//   pin modes based on pin state / time 
//   3 is pin on (STEADY STATE) units are counted by mS per unit, mS are configured in the pulses_per_unit register 
//   4 is pin off (STEADY STATE) units are counted by mS per unit  mS are configured in the pulses_per_unit register 
//  in modes 3,4,7,8 input pulses are elapsed mS. For units in seconds, set sensor_pulses_per_unit = 1000 

//   pin modes based on analog input 
//   5- analog signal 2.6v = 4095 (uses analog, threshold, min, and max settings to to set input pulse rate based on signal 
//   6- 2.6v=4095->mapped. 'cycle' starts when analog < threshold and stops when analog > limit. pulses per mS as in mode 3/4 
//   7- 2.6v=4095->mapped. 'cycle' starts when analog < threshold and stops when analog > limit. pulses based on analog as in (5) 

//   NOTES FOR ANALOG SETTINGS -- !!CAUTION: If using mode 5 OR 7 the analog_map_max must be !=0 and mapping applied. 
//   analog data is used as is (0-4095) if analog_range_max is set to zero. 
//   if analog_range_max != 0 then the raw analog data will be mapped to the ouput data as follows: 
//   from the range {range_min -> range_max} map to: {map_min -> map_max} Note that map_min and map_max are limited to 0-5000. 
//   the algorithm is:   for the value s, within range a1->a2, map to range b1->b2. result = b1+(s-a1)*(b2-b1)/(a2-a1) 
//   note that the map-to values (b1->b2) can be reversed to invert the input so that higher inputs result in lower outputs. 

//   OTHER NOTES: 
//  LATCH_MODE:   0 = non latch, 1 = persists, 2 = latches on, 3 = latches on, persists 
//  flow_limit sets a minimum flow rate to count. flows below flow_limit will not be counted. 
//  for pulses in mode 1 (output_pulses_per_trigger != 0) 
//  pulses out = ((sensor_pulses / sensor_pulses_per_unit) / units_per_event) * output_pulses_per_trigger

//   APPLICATION NOTES: the analog range is set by default to 11db which gives a range of up to 2.6v 
//  for 0-20ma sensing use a 100ohm resistor between GND and the ground end of the signal wire. attach the analog sense pin 
//  to the sensor end of the resistor. this will give about 2v at 20mA, and read about 3150 on the ADC. To measure voltage, 
//  make a voltage divider (0-5v IN----1k--ADC--1k---GND),(0-12v IN----5k--ADC--1.2k---GND),(75v IN----68k--ADC--2.2k---GND) 
//    --------------------------------------------------------------------------------------- 

//   HREGS:             <HREG Number> (offset 0 ) 
//  OUTPUT_PULSES_TO_ADD    <0>-      (write only). output pulses to add to current job 
//  UNITS_SINCE_BOOT        <1>-                 write to ADD UNITS to total or zero-->  
//  FLOW_TIME_SINCE_BOOT    <2>-                 write to ADD UNITS to total or zero-->
//  SHOW_DISPLAY            <3>-                minutes (1000 max) to enable display-->
//  DISPLAY_MODE            <4>-        set 0-5 (5 is off) for deafault display mode-->     
//  ---------------------The below settings will be saved on setting---------------------- 
//  MODE                    <9>-                                      Operating Mode-->  
//  PIN_MODE                <10>-                                          pin mode -->  
//  SENSOR_PULSES_PER_UNIT  <11>-          Input pulses for each output unit counted-->  
//  SAVE_INTERVAL           <12>-        Save interval. units between saves to flash-->  
//  HORN_UNITS              <13>-                       units to trigger HORN output-->  
//  RELAY_UNITS             <14>-                      units to trigger RELAY output-->  
//  HORN_SECONDS            <15>-       seconds without reset to trigger HORN output-->  
//  RELAY_SECONDS           <16>-      seconds without reset to trigger RELAY output-->  
//  FLOW_STOP_SECONDS       <17>-   resets flow and time counts, non-latched outputs-->  
//  RELAY_LATCHMODE         <18>-0=non latch, 1=persists, 2=latch, 3=latch, persists-->  
//  HORN_LATCHMODE          <19>-0=non latch, 1=persists, 2=latch, 3=latch, persists-->  
//  OUTPUT_PULSE_MS         <20>-                      milliseconds for pulse length-->  
//  OUTPUT_PULSE_OFF_MS     <21>-             milliseconds for pulse length off time-->  
//  UNITS_PER_EVENT         <22>-               Units to trigger each output 'event'-->  
//  OUTPUT_PULSES_PER_TRIGGER <23>-      (0=off) pulses to que for each output event-->  
//  OUTPUT_PULSES_FOR_PRIME <24>-      Pulses to add to current job for pump priming-->  
//  FLOW_LIMIT  (MODE 2or3) <25>-  Input_pulses=0 unless flow/time is met (0 = null)-->  
//  ANALOG_THRESHOLD        <26>-                        0-4096 threshold to trigger-->  
//  ANALOG_MIN              <27>-        Analog map outpiut range. limited to 0-5000-->  
//  ANALOG_MAX              <28>-        Analog map outpiut range. limited to 0-5000-->  
//  ANALOG_RANGE_MIN        <29>-                  constrain analoginput values here-->  
//  ANALOG_RANGE_MAX        <30>-                  constrain analoginput values here-->  
//  FLOWRATE_SECONDS        <31>- number of seconds to use for calculating flow rate-->
//  ADD_TOTAL_UNITS         <32>-   units to add for adjusting total units displayed-->  

//  RESET (TYPE 0-3)        <99>-           Set to initiate reset. see above for details 

//   ISTSs:         <ISTS Number> (offset 0 ) 
//  HORN_COIL_ISTS           <0>-                                 Horn status --> 
//  RELAY_COIL_ISTS          <1>-                                 Horn status --> 

//   COILs:         <COIL Number> (offset 0 ) 
//  HORN_COIL               <0>-             Alarm control write 0 to clear latch 
//  RELAY_COIL              <1>-             Relay control write 0 to clear latch 
//  STOP_COIL               <9>- write->stops current job  read->prints this page 
//  SAVE_COIL              <10>-read->saves current data    write->saves settings 

//   IREGs:         <IREG Number> (offset 0 ) 
//  LIFETIME_UNITS_MSW      <0>-upper 16 bits of total lifetime output pulses-->  
//  LIFETIME_UNITS_LSW      <1>-lower 16 bits of total lifetime output pulses-->  
//  THIS_FLOW_UNITS         <2>-                           current flow units-->  
//  THIS_FLOW_DURATION      <3>-                        current flow duration-->  
//  LARGEST_FLOW_UNITS      <4>-                       most units in one flow-->  
//  LONGEST_FLOW_DURATION   <5>-                     Longest logged flow time-->  
//  FLOW_RATE               <6>-   (pulses/sec / pulsesPerUnit) * flowRateSec-->  
//  OUTPUT_PULSES_TODO      <7>-              Number of output pulses pending-->  
//  SENSOR_PULSES           <8>-                      UNCOUNTED sensor pulses-->  
//  ANALOG_SENSE            <9>-                                 Analog input-->  
//  UNITS_QUEUED            <10>-              Units not yet consumed by jobs-->
//  WIFI_RSSI               <11>-                                            -->  
//  UPTIME_HRS              <12>-                                            -->    

//    --------------------------------------------------------------------------------------- 
//pin definitions
// #ifdef C3_42_OLED
// #define ledOn 0
// #define ledPin 8
// #define hornPin 4
// #define relayPin 3
// #define pulse_in_pin 2
// #define button_in_pin 9
// #define analog_pin 1
// #define SDA_PIN 5 //INTERNAL
// #define SCL_PIN 6 //INTERNAL
// #endif

// #ifdef WEMOS_MINI_32
// #define ledOn 1
// #define ledPin 2
// #define hornPin 4
// #define relayPin 0
// #define pulse_in_pin 18
// #define button_in_pin 5
// #define analog_pin 35
// #define SDA_PIN -1  // Use -1 for the Wire library default pins
// #define SCL_PIN -1  // Use -1 for the Wire library default pins
// #endif

// #ifdef S2_mini
// #define ledOn 1
// #define ledPin 15  // ESP32-S2 lolin s2 mini
// #define hornPin 7
// #define relayPin 5
// #define pulse_in_pin 3
// #define button_in_pin 0
// #define analog_pin 1
// #define SDA_PIN 33  // Use -1 for the Wire library default pins
// #define SCL_PIN 35  // Use -1 for the Wire library default pins
// #endif
//    --------------------------------------------------------------------------------------- 

// //COILs
// const int HORN_COIL = 0;
// const int RELAY_COIL = 1;
// //const int LATCH_RESET_COIL = 2;

// const int STOP_STATUS_COIL = 9;
// const int SAVE_COIL = 10;

// //IREGs
// const int LIFETIME_UNITS_MSW = 0;
// const int LIFETIME_UNITS_LSW = 1;

// const int THIS_FLOW_UNITS = 2;
// const int THIS_FLOW_DURATION = 3;
// const int LARGEST_FLOW_UNITS = 4;
// const int LONGEST_FLOW_DURATION = 5;
// const int FLOW_RATE = 6;

// const int OUTPUT_PULSES_TODO = 7;
// const int SENSOR_PULSES = 8;
// const int ANALOG_SENSE = 9;
// const int UNITS_QUEUED = 10;
// const int WIFI_RSSI = 11;
// const int UPTIME_HRS = 12;

// //HREGs
// const int OUTPUT_PULSES_TO_ADD = 0;
// const int UNITS_SINCE_BOOT = 1;
// const int FLOW_TIME_SINCE_BOOT = 2;
// const int SHOW_DISPLAY = 3;  //show display for n minutes (max 120)
// const int DISPLAY_MODE = 4; 
// const int RESET_HREG = 99;
// //Parameters to save: these values loaded from flash
// const int MODE = 9;
// const int PIN_MODE = 10;
// const int SENSOR_PULSES_PER_UNIT = 11;
// const int SAVE_INTERVAL = 12;
// const int HORN_UNITS = 13;
// const int RELAY_UNITS = 14;
// const int HORN_SECONDS = 15;
// const int RELAY_SECONDS = 16;
// const int FLOW_STOP_SECONDS = 17;
// const int RELAY_LATCHMODE = 18;
// const int HORN_LATCHMODE = 19;
// const int OUTPUT_PULSE_MS = 20;
// const int OUTPUT_PULSE_OFF_MS = 21;
// const int UNITS_PER_EVENT = 22;
// const int OUTPUT_PULSES_PER_TRIGGER = 23;
// const int OUTPUT_PULSES_FOR_PRIME = 24;
// const int MIN_FLOW = 25;
// const int ANALOG_THRESHOLD = 26;
// const int ANALOG_MAP_MIN = 27;
// const int ANALOG_MAP_MAX = 28;
// const int ANALOG_RANGE_MIN = 29;
// const int ANALOG_RANGE_MAX = 30;  // IF Zero, Mapping is off.
// const int FLOWRATE_SECONDS = 31;
// const int ADD_TOTAL_UNITS = 32;

// // data variables
// uint16_t lifetime_units_MSW;
// uint16_t lifetime_units_LSW;
// uint16_t this_flow_units;
// uint16_t this_flow_duration;
// uint16_t largest_flow_units;
// uint16_t longest_flow_duration;
// uint16_t flow_rate;
// uint16_t output_pulses_todo;
// volatile u_long ISR_sensor_pulses;    //used in ISR
// volatile u_long ISR_flowrate_pulses;  //used in ISR
// uint16_t sensor_pulses;
// uint16_t analog_sense = 2000;
// uint16_t units_queued;

// uint16_t flow_time_since_boot = 0;
// uint16_t units_since_boot = 0;

// // Settings variables (save on write from modbus):
// uint16_t mode = 1;
// uint16_t pin_mode = 0;
// uint16_t sensor_pulses_per_unit = 150;
// uint16_t save_interval = 1000;
// uint16_t horn_units = 0;
// uint16_t relay_units = 0;
// uint16_t horn_seconds = 0;
// uint16_t relay_seconds = 0;
// uint16_t flow_stop_seconds = 5;
// uint16_t relay_latchmode = 0;
// uint16_t horn_latchmode = 0;
// uint16_t output_pulse_ms = 200;
// uint16_t output_pulse_off_ms = 200;
// uint16_t units_per_event = 10;
// uint16_t output_pulses_per_trigger = 0;
// uint16_t output_pulses_for_prime = 10;  ///
// uint16_t min_flow = 0;
// uint16_t analog_threshold = 0;
// uint16_t analog_map_min = 0;
// uint16_t analog_map_max = 4095;  // set to reflect raw data / 10 by default
// uint16_t analog_range_min = 0;
// uint16_t analog_range_max = 4095;
// uint16_t flowRate_seconds = 60;  // for units per minute

// //misc things to store in flash:
// uint16_t valid_flash;
// uint16_t lifetime_units_from_flash_MSW;
// uint16_t lifetime_units_from_flash_LSW;
// uint16_t boot_mode = 0;

// uint16_t horn_status;
// uint16_t relay_status;
// uint16_t oneSecondTimer = 0;
// uint16_t oneHundred_msTimer = 0;
// uint16_t oneMinuteTimer = 0;
// uint16_t double_reset_timer = 0;

// u_long buttonTime;
// u_long PulseInTime;
// u_long ten_ms_heartbeat;
// u_long PulseTime;
// u_long total_flow_time_at_start;
// u_long flowResetTime;
// u_long rawPulseTime;
// u_long last_save;
// u_long blink;
// u_long displayTime;  //display on for 10 min
// u_long flashRate;

// uint8_t display_mode = 0;
// uint8_t default_display_mode = 0;
// uint8_t textScroll = 0;

// int buttonSeconds;

// bool pulse_on_in_progress = false;
// bool pulse_off_in_progress = false;
// bool call_for_pulse = false;
// bool active_pulse_train = false;
// bool active_flow = false;
// bool system_initialized = false;
// bool horn_set_by_modbus = false;
// bool relay_set_by_modbus = false;
// bool cycle_state = false;
// bool button_state = false;
// bool animation = false;

//---------------------------modbus API reference:----------------------------

//------------------CALLBACKS:
// mb.onConnect(connect_callback);   // Add callback on connection event. must return 1 to allow connection
// mb.onRequest(PreRequest_callback);
// mb.onRequestSuccess(PostRequest_callback);

// mBus.onSetHreg(regstr, reg_write_callback); or (base_regstr, write_callback_for_multiple_regstrs, number_of_regstrs);
// mBus.onSetIreg(regstr, reg_write_callback); or (base_regstr, write_callback_for_multiple_regstrs, number_of_regstrs);
// mBus.onSetIsts(regstr, reg_write_callback); or (base_regstr, write_callback_for_multiple_regstrs, number_of_regstrs);
// mBus.onSetCoil(coil, coil_write_callback); or (base_coil, write_callback_for_multiple_coils, number_of_coils);

// mBus.onGetHreg(regstr, reg_read_callback); or (base_regstr, read_callback_for_multiple_regstrs, number_of_regstrs);
// mBus.onGetIreg(regstr, reg_read_callback); or (base_regstr, read_callback_for_multiple_regstrs, number_of_regstrs);
// mBus.onGetIsts(regstr, reg_read_callback); or (base_regstr, read_callback_for_multiple_regstrs, number_of_regstrs);
// mBus.onGetCoil(coil, coil_read_callback); or (base_coil, read_callback_for_multiple_coils, number_of_coils);


// Add local register
//Modbus Registers Offsets

// const int EXAMPLE_COIL = 0;   //(1)  NAME_OF_COIL = <coil # offset from 0>
// const int EXAMPLE_HREG = 5;   //(6) NAME_OF_REGISTER = <register # offset from 0>
// const int EXAMPLE_IREG = 2;   //(3) NAME_OF_REGISTER = <register # offset from 0>
// const int EXAMPLE_ISTS = 9;   //(10) NAME_OF_REGISTER = <register # offset from 0>

// mBus.server();  // setup the Modbus object

// #define _EXAMPLE_COIL "description (COIL 0)"
// #define _EXAMPLE_HREG "description (HREG 5)"
// #define _EXAMPLE_IREG "description (IREG 2)"
// #define _EXAMPLE_ISTS "description (ISTS 9)"

// mBus.addCoil(EXAMPLE_COIL);
// mBus.addHreg(EXAMPLE_HREG);
// mBus.addIreg(EXAMPLE_IREG, 0); (<NAME>, <optional initial value>);
// mBus.addIsts(EXAMPLE_ISTS);

//setup with callbacks:
// const int EXAMPLE_HREG = 5;

// mBus.addHreg(EXAMPLE_HREG);
// mBus.onSetHreg(EXAMPLE_HREG, example_write_callback)
// mBus.onGetHreg(EXAMPLE_HREG, example_read_callback)



// bool addHreg(uint16_t offset, uint16_t value = 0, uint16_t numregs = 1);
// bool addCoil(uint16_t offset, bool value = false, uint16_t numregs = 1);
// bool addIsts(uint16_t offset, bool value = false, uint16_t numregs = 1);
// bool addIreg(uint16_t offset, uint16_t value = 0, uint16_t nemregs = 1);

//     -offset- Address of the first register to add
//     -value- Initial value to be assigned to register(s)
//     -numregs- Count of registers to be created



// Adding new register(s) and assigning value(s). If [some] registers already exists value will be updated. Returns true on success. false if operation is failed for some reason.
// Write local reg

// bool Hreg(uint16_t offset, uint16_t value);
// bool Coil(uint16_t offset, bool value);
// bool Ists(uint16_t offset, bool value);
// bool Ireg(uint16_t offset, uint16_t value);

//     -offset- Address of the register
//     -value- Value to be assigned to register

// Returns true on success. false if register not previousely added or other error.
// Read local reg

// uint16_t Hreg(uint16_t offset);
// bool Coil(uint16_t offset);
// bool Ists(uint16_t offset);
// uint16_t Ireg(uint16_t offset);

//     -offset- Address of the register to read
