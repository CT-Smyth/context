// BOARDS file ESP32 3.05
#include <WiFi.h>              //builtin
#include <WiFiManager.h>       //2.0.17 https://github.com/tzapu/WiFiManager
#include <Preferences.h>       //included
#include <ModbusIP_ESP8266.h>  //4.10 https://github.com/emelianov/modbus-esp8266
#include <EEPROM.h>            //builtin For storing the firmware version
#include <FS.h>                //builtin
#include <ESPmDNS.h>
#include <OneBitDisplay.h>
#include "esp_task_wdt.h"

#define VERSION_NUMBER "V5.8"

#define WDT_TIMEOUT 30        //sec
#define DOUBLE_RESET_TIME 10  //sec

#define C3_42_OLED  // enable this if you're using the .42" oled and not the standard .96" or 1.3"
//#define WEMOS_MINI_32  //S3 D1 Mini with generic 128x64 OLED
//#define S2_mini  // WEMOS s2 mini

//#define SERIAL_VERBOSE
#define USE_MODBUS

// remark out MY_OLED for Non - LED deployment
#define MY_OLED OLED_128x64  //72x40 displays in center of 128x64 field
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// if your system doesn't have enough RAM for a back buffer, comment out
// this line (e.g. ATtiny85)
#define USE_BACKBUFFER

#ifdef USE_BACKBUFFER
static uint8_t ucBackBuffer[1024];
#else
static uint8_t *ucBackBuffer = NULL;
#endif

//pin definitions
#ifdef C3_42_OLED
#define ledOn 0
#define ledPin 8
#define hornPin 4
#define relayPin 3
#define pulse_in_pin 2
#define button_in_pin 9
#define analog_pin 1
#define SDA_PIN 5 //INTERNAL
#define SCL_PIN 6 //INTERNAL
#endif

#ifdef WEMOS_MINI_32
#define ledOn 1
#define ledPin 2
#define hornPin 4
#define relayPin 0
#define pulse_in_pin 18
#define button_in_pin 5
#define analog_pin 35
#define SDA_PIN -1  // Use -1 for the Wire library default pins
#define SCL_PIN -1  // Use -1 for the Wire library default pins
#endif

#ifdef S2_mini
#define ledOn 1
#define ledPin 15  // ESP32-S2 lolin s2 mini
#define hornPin 7
#define relayPin 5
#define pulse_in_pin 3
#define button_in_pin 0
#define analog_pin 1
#define SDA_PIN 33  // Use -1 for the Wire library default pins
#define SCL_PIN 35  // Use -1 for the Wire library default pins
#endif


#define RESET_PIN -1  //disabled
#define OLED_ADDR -1  //detect I2C address
#define FLIP180 0
#define INVERT 0
#define USE_HW_I2C 1  // Bit-Bang the I2C bus


//wifiManager
WiFiManager wm;

//Display
OBDISP oled;

//Network Options
//#define STATIC_IP
#define MDNS_RESPONDER

#ifdef STATIC_IP
IPAddress staticIP(192, 168, 3, 250);
IPAddress gateway(192, 168, 3, 1);
IPAddress subnet(255, 255, 224, 0);
IPAddress primaryDNS(192, 168, 3, 1);  // (optional)
IPAddress secondaryDNS(0, 0, 0, 0);
#endif

//COILs
const int HORN_COIL = 0;
const int RELAY_COIL = 1;
//const int LATCH_RESET_COIL = 2;

const int STOP_STATUS_COIL = 9;
const int SAVE_COIL = 10;

//IREGs
const int LIFETIME_UNITS_MSW = 0;
const int LIFETIME_UNITS_LSW = 1;
const int THIS_FLOW_UNITS = 2;
const int THIS_FLOW_DURATION = 3;
const int LARGEST_FLOW_UNITS = 4;
const int LONGEST_FLOW_DURATION = 5;
const int FLOW_RATE = 6;
const int OUTPUT_PULSES_TODO = 7;
const int SENSOR_PULSES = 8;
const int ANALOG_SENSE = 9;
const int UNITS_QUEUED = 10;
const int WIFI_RSSI = 11;
const int UPTIME_HRS = 12;
const int OUTPUT_PULSES = 13;

//HREGs
const int OUTPUT_PULSES_TO_ADD = 0;
const int UNITS_SINCE_BOOT = 1;
const int FLOW_TIME_SINCE_BOOT = 2;
const int SHOW_DISPLAY = 3;  //show display for n minutes (max 120)
const int DISPLAY_MODE = 4;
const int RESET_HREG = 99;
//Parameters to save: these values loaded from flash
const int MODE = 9;
const int PIN_MODE = 10;
const int SENSOR_PULSES_PER_UNIT = 11;
const int SAVE_INTERVAL = 12;
const int HORN_UNITS = 13;
const int RELAY_UNITS = 14;
const int HORN_SECONDS = 15;
const int RELAY_SECONDS = 16;
const int FLOW_STOP_SECONDS = 17;
const int RELAY_LATCHMODE = 18;
const int HORN_LATCHMODE = 19;
const int OUTPUT_PULSE_MS = 20;
const int OUTPUT_PULSE_OFF_MS = 21;
const int UNITS_PER_EVENT = 22;
const int OUTPUT_PULSES_PER_TRIGGER = 23;
const int OUTPUT_PULSES_FOR_PRIME = 24;
const int MIN_FLOW = 25;
const int ANALOG_THRESHOLD = 26;
const int ANALOG_MAP_MIN = 27;
const int ANALOG_MAP_MAX = 28;
const int ANALOG_RANGE_MIN = 29;
const int ANALOG_RANGE_MAX = 30;  // IF Zero, Mapping is off.
const int FLOWRATE_SECONDS = 31;
const int ADD_TOTAL_UNITS = 32;

// data variables
uint16_t lifetime_units_MSW;
uint16_t lifetime_units_LSW;
uint16_t this_flow_units;
uint16_t this_flow_duration;
uint16_t largest_flow_units;
uint16_t longest_flow_duration;
uint16_t flow_rate;
uint16_t output_pulses_todo;
volatile u_long ISR_sensor_pulses;    //used in ISR
volatile u_long ISR_flowrate_pulses;  //used in ISR
uint16_t sensor_pulses;
uint16_t analog_sense = 2000;
uint16_t units_queued;
uint16_t flow_time_since_boot = 0;
uint16_t units_since_boot = 0;
uint16_t output_pulses;

// Settings variables (save on write from modbus):
uint16_t mode = 1;
uint16_t pin_mode = 0;
uint16_t sensor_pulses_per_unit = 150;
uint16_t save_interval = 1000;
uint16_t horn_units = 0;
uint16_t relay_units = 0;
uint16_t horn_seconds = 0;
uint16_t relay_seconds = 0;
uint16_t flow_stop_seconds = 5;
uint16_t relay_latchmode = 0;
uint16_t horn_latchmode = 0;
uint16_t output_pulse_ms = 200;
uint16_t output_pulse_off_ms = 200;
uint16_t units_per_event = 10;
uint16_t output_pulses_per_trigger = 0;
uint16_t output_pulses_for_prime = 10;  ///
uint16_t min_flow = 0;
uint16_t analog_threshold = 0;
uint16_t analog_map_min = 0;
uint16_t analog_map_max = 4095;  // set to reflect raw data / 10 by default
uint16_t analog_range_min = 0;
uint16_t analog_range_max = 4095;
uint16_t flowRate_seconds = 60;  // for units per minute

//misc things to store in flash:
uint16_t valid_flash;
uint16_t lifetime_units_from_flash_MSW;
uint16_t lifetime_units_from_flash_LSW;
uint16_t boot_mode = 0;

uint16_t horn_status;
uint16_t relay_status;
uint16_t oneSecondTimer = 0;
uint16_t oneHundred_msTimer = 0;
uint16_t oneMinuteTimer = 0;
uint16_t doubleResetTimer = 0;
uint16_t displayTimeout = 300;  //5 minutes

u_long buttonTime;
u_long PulseInTime;
u_long ten_ms_heartbeat;
u_long PulseTime;
u_long total_flow_time_at_start;
u_long flowResetTime;
u_long rawPulseTime;
u_long last_save;
u_long blink;
u_long displayTime;  //display on for 10 min
u_long flashRate;

uint8_t display_mode = 0;
uint8_t default_display_mode = 0;
uint8_t textScroll = 0;

int buttonSeconds;

bool pulse_on_in_progress = false;
bool pulse_off_in_progress = false;
bool call_for_pulse = false;
bool active_pulse_train = false;
bool active_flow = false;
bool system_initialized = false;
bool horn_set_by_modbus = false;
bool relay_set_by_modbus = false;
bool cycle_state = false;
bool button_state = false;
bool animation = false;

String WPASS;
String WSSID;


Preferences keystore;

#ifdef USE_MODBUS
ModbusIP mBus;
#endif


void setup() {

  pinMode(ledPin, OUTPUT);
  pinMode(hornPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(pulse_in_pin, INPUT);
  pinMode(button_in_pin, INPUT_PULLUP);

  Serial.begin(115200);
  EEPROM.begin(128);

  startWiFi();
#ifdef USE_MODBUS
  setupModbus();
#endif
  watchdogTimerSetup();
  config_and_init();
}

//Begin Functions and main loop
void IRAM_ATTR pin_interrupt() {
  ISR_sensor_pulses++;
  ISR_flowrate_pulses++;
}



// ------------------------------------------------For PIN_MODE
// pin modes based on pulses:
// 0- is count pulse rising
// 1- is count pulse falling
// 2- is count pulse on change

// pin modes based on pin state / time
// 3- is pin on (STEADY STATE) pulse output is 1pulse/mS when active, mS per unit are configured in the pulses_per_unit register
// 4- is pin off (STEADY STATE) pulse output is 1pulse/mS when active, mS per unit are configured in the pulses_per_unit register

// pin modes based on analog input
// 5- is analog signal 2.6v=4095->mapped-uses analog, threshold, min, and max settings to to set input pulse rate based on signal
// 6- is 2.6v=4095->mapped. "cycle" starts when analog < threshold and stops when analog > limit. pulses per mS as in mode 3/4
// 7- is 2.6v=4095->mapped. "cycle" starts when analog < threshold and stops when analog > limit. pulses based on analog as in 5
// analog data is used as is (0-4095) if analog_range_max is set to zero.
// if analog_range_max != 0 then the raw analog data will be mapped to the ouput data as follows:
// from the range {range_min -> range_max} map to: {map_min -> map_max} Note that map_min and map_max are limited to 0-500.
// the algorithm is:   for the value s, within range a1->a2, map to range b1->b2. result = b1+(s-a1)*(b2-b1)/(a2-a1)
// note that the map-to values (b1->b2) can be reversed to invert the input so that higher inputs result in lower outputs.

uint16_t getInputs(u_long millis_now) {
  uint16_t raw_pulses;
  bool pulse_pin_state;
  uint16_t analog_data = analog_sense;

  pulse_pin_state = digitalRead(pulse_in_pin);

  switch (pin_mode) {
    case 3:
      if (pulse_pin_state == 1) {
        raw_pulses = millis_now - rawPulseTime;
        rawPulseTime = millis_now;
        ISR_flowrate_pulses = ISR_flowrate_pulses + raw_pulses;
        return raw_pulses;
      } else {
        return 0;
      }
      break;

    case 4:
      if (pulse_pin_state == 0) {
        raw_pulses = millis_now - rawPulseTime;
        rawPulseTime = millis_now;
        ISR_flowrate_pulses = ISR_flowrate_pulses + raw_pulses;
        return raw_pulses;
      } else {
        return 0;
      }
      break;

    case 5:
      if (analog_data >= analog_threshold) {  // if analog data exceeds the threshold setting, count flows
        pulse_pin_state = 1;
        ISR_flowrate_pulses = ISR_flowrate_pulses + analog_data;
        return analog_data;
      } else {  // no flow (by threshold)
        pulse_pin_state = 0;
        return 0;
      }
      break;

    case 6:
      if (analog_data < analog_threshold) {  //start cycle
        cycle_state = 1;
      }
      if (analog_data > analog_range_max) {  //end cycle
        cycle_state = 0;
      }
      //generate pulses based on millis while cycle is on.
      if (cycle_state == 1) {
        pulse_pin_state = 1;
        raw_pulses = millis_now - rawPulseTime;
        rawPulseTime = millis_now;
        ISR_flowrate_pulses = ISR_flowrate_pulses + raw_pulses;
        return raw_pulses;
      } else {
        pulse_pin_state = 0;
        return 0;
      }
      break;

    case 7:
      if (analog_data < analog_threshold) {  //start cycle
        cycle_state = 1;
      }
      if (analog_data > analog_range_max) {  //end cycle
        cycle_state = 0;
      }
      //generate pulses based on tha analog signal while cycle is on.
      if (cycle_state == 1) {
        pulse_pin_state = 1;
        ISR_flowrate_pulses = ISR_flowrate_pulses + analog_data;
        return analog_data;

      } else {
        pulse_pin_state = 0;
        return 0;
      }
      break;

    default:
      raw_pulses = ISR_sensor_pulses;
      ISR_sensor_pulses = 0;
      return raw_pulses;
      break;
  }
}

uint16_t processRawPulses(uint16_t raw_pulses, u_long millis_now) {
  rawPulseTime = millis();

  if ((min_flow > 0)) {
    if (flow_rate <= min_flow) {  // cut off any pulses at flow rates below flow_limit
      raw_pulses = 0;
    }
  }

  if (raw_pulses != 0) {  //If flow

    if (active_flow == 0) {  //if new flow
      active_flow = 1;
      PulseInTime = millis_now;
      total_flow_time_at_start = flow_time_since_boot;
      this_flow_units = 0;
      this_flow_duration = 0;
    }

    flowResetTime = millis_now + (flow_stop_seconds * 1000);
    this_flow_duration = (millis_now - PulseInTime) / 1000;  //in seconds

    if (this_flow_duration > longest_flow_duration) {  //check duration for largest
      longest_flow_duration = this_flow_duration;
    }

    sensor_pulses = sensor_pulses + raw_pulses;  //add new pulses
    flow_time_since_boot = total_flow_time_at_start + this_flow_duration;

  } else if (flowResetTime < millis_now) {  //if flow has ended
    active_flow = 0;
  }

  // check if sensor_pulses meets a full unit, process accordingly
  if (sensor_pulses >= sensor_pulses_per_unit) {

    uint16_t newUnits = sensor_pulses / sensor_pulses_per_unit;


    sensor_pulses = sensor_pulses - (newUnits * sensor_pulses_per_unit);

    addUnits(newUnits);

    u_long u_queued = units_queued + newUnits;
    u_long this_flow_u = this_flow_units + newUnits;
    u_long u_since_boot = units_since_boot + newUnits;

    if (u_queued > 65534) {
      units_queued = 65535;
    } else {
      units_queued = u_queued;
    }

    if (this_flow_u > 65534) {
      this_flow_units = 65535;
    } else {
      this_flow_units = this_flow_u;
    }

    if (u_since_boot > 65534) {
      units_since_boot = 65535;
    } else {
      units_since_boot = u_since_boot;
    }

    if (this_flow_units > largest_flow_units) {  //check flow for largest
      largest_flow_units = this_flow_units;
    }
  }
  return units_queued;
}

// mode 0 is pulses on relay output based on output_pulses_per_trigger, horn output as in mode 1. Latching
// and persistance on relay is ignored.
// mode 1 is trigger on flow over x_seconds or y_units without flow_stop_seconds of pause -- if flow_limit != 0: only valid if (this_flow_rate > flow_limit)
// mode 2 is trigger on total flow time only since reset or boot (flow_time_since_boot)
// mode 3 is trigger on total flow units only since reset or boot (total_units_since_boot)
// mode 4 is trigger on total flow units or time  since reset or boot (total_units_since_boot, flow_time_since_boot)
// mode 5 is trigger on failure to achieve x_units in y_seconds while flowing (low flow) -- if flow_limit != 0: only valid if (this_flow_rate > flow_limit)
// mode 6 is count only. Horn and relay outputs are set by modbus only.
// for mode 2,3,4 counts can be set / reset by setting the flow_time_since_boot and units_since_boot HREGs via modbus
// In mode 1 only:  If ouput pulses per trigger is  > 0, then relay outputs will be based
// on the pulse->unit->event->pulses caclculation, and relay_units will be ignored. Latching
// and persistance on relay is also ignored.

uint16_t processActions(uint16_t queued_units_to_process, u_long millis_now) {

  switch (mode) {
    case 0:
      //if output pulses per trigger = 0 then output is not pulsed, otherwise schedule pulses and consume inputs
      if (output_pulses_per_trigger > 0) {
        if (queued_units_to_process >= units_per_event) {  // Trigger an output event
          //Serial.println("output pulses event triggered");
          queued_units_to_process = queued_units_to_process - units_per_event;
          output_pulses_todo = output_pulses_todo + output_pulses_per_trigger;
        }
      }

      if ((horn_units > 0) && (this_flow_units >= horn_units) && (active_flow == 1) && (horn_status == 0)) {  // Trigger an output event
        //Serial.println("horn units event triggered");                                                         //DEBUG
        setHorn(1);
      }

      if ((horn_seconds > 0) && (this_flow_duration >= horn_seconds) && (active_flow == 1) && (horn_status == 0)) {
        setHorn(1);
      }

      if ((active_flow == 0) && (horn_status == 1) && (horn_latchmode < 2) && (horn_set_by_modbus == 0)) {  // if horn is on and flow stops
        setHorn(0);
        //queued_units_to_process = 0;
      }
      break;

    case 1:
      if ((relay_units > 0) && (this_flow_units >= relay_units) && (active_flow == 1) && (relay_status == 0)) {  // Trigger an output event
        //Serial.println("relay units event triggered");                                                           //DEBUG
        setRelay(1);
      }

      if ((relay_seconds > 0) && (this_flow_duration >= relay_seconds) && (active_flow == 1) && (relay_status == 0)) {
        setRelay(1);
      }

      if ((active_flow == 0) && (relay_status == 1) && (relay_latchmode < 2) && (relay_set_by_modbus == 0)) {  // if relay is on and flow stops
        setRelay(0);
        queued_units_to_process = 0;
      }

      if ((horn_units > 0) && (this_flow_units >= horn_units) && (active_flow == 1) && (horn_status == 0)) {  // Trigger an output event
        //Serial.println("horn units event triggered");                                                         //DEBUG
        setHorn(1);
      }

      if ((horn_seconds > 0) && (this_flow_duration >= horn_seconds) && (active_flow == 1) && (horn_status == 0)) {
        setHorn(1);
      }

      if ((active_flow == 0) && (horn_status == 1) && (horn_latchmode < 2) && (horn_set_by_modbus == 0)) {  // if horn is on and flow stops
        setHorn(0);
        queued_units_to_process = 0;
      }
      break;

    case 2:
      queued_units_to_process = 0;
      if ((relay_seconds > 0) && (flow_time_since_boot >= relay_seconds) && (active_flow == 1) && (relay_status == 0)) {
        setRelay(1);
      }
      if ((horn_seconds > 0) && (flow_time_since_boot >= horn_seconds) && (active_flow == 1) && (horn_status == 0)) {
        setHorn(1);
      }
      queued_units_to_process = 0;
      break;

    case 3:
      queued_units_to_process = 0;
      if ((relay_units > 0) && (units_since_boot >= relay_units) && (active_flow == 1) && (relay_status == 0)) {
        setRelay(1);
      }
      if ((horn_units > 0) && (units_since_boot >= horn_units) && (active_flow == 1) && (horn_status == 0)) {
        setHorn(1);
      }
      queued_units_to_process = 0;
      break;

    case 4:
      queued_units_to_process = 0;
      if ((relay_seconds > 0) && (flow_time_since_boot >= relay_seconds) && (active_flow == 1) && (relay_status == 0)) {
        setRelay(1);
      }
      if ((horn_seconds > 0) && (flow_time_since_boot >= horn_seconds) && (active_flow == 1) && (horn_status == 0)) {
        setHorn(1);
      }
      if ((relay_units > 0) && (units_since_boot >= relay_units) && (active_flow == 1) && (relay_status == 0)) {
        setRelay(1);
      }
      if ((horn_units > 0) && (units_since_boot >= horn_units) && (active_flow == 1) && (horn_status == 0)) {
        setHorn(1);
      }
      queued_units_to_process = 0;
      break;

    case 5:
      queued_units_to_process = 0;
      if ((relay_seconds > 0) && (this_flow_duration >= relay_seconds) && (this_flow_units < relay_units) && (active_flow == 1) && (relay_status == 0)) {
        setRelay(1);
      }
      if ((horn_seconds > 0) && (this_flow_duration >= horn_seconds) && (this_flow_units < horn_units) && (active_flow == 1) && (horn_status == 0)) {
        setHorn(1);
      }
      queued_units_to_process = 0;
      break;

    default:  //just count - no control
      queued_units_to_process = 0;
      break;
  }


  if (mode != 0) {
    if ((active_flow == 0) && (relay_status == 1) && (relay_latchmode < 2) && (relay_set_by_modbus == 0)) {  // if relay is on and flow stops
      setRelay(0);                                                                                           //
      queued_units_to_process = 0;
    }

    if ((active_flow == 0) && (horn_status == 1) && (horn_latchmode < 2) && (horn_set_by_modbus == 0)) {  // if horn is on and flow stops
      setHorn(0);
      queued_units_to_process = 0;
    }
  }

  checkForSave();
  return queued_units_to_process;
}


void checkForSave() {
  if (((lifetime_units_LSW - lifetime_units_from_flash_LSW) >= save_interval) || ((lifetime_units_LSW != lifetime_units_from_flash_LSW) && ((millis() - last_save) > 86400000))) {  //save based on elapsed units
    saveData();
  }
}

void oneMinute() {
  if (WiFi.status() != WL_CONNECTED) {
    if (WSSID != "") {
      WiFi.begin(WSSID, WPASS);
    }
    //Serial.println("----wifi not connected----");
  } else {
    //Serial.println("----wifi connection stable!----");
  }
}

void oneSecond() {
  flow_rate = ISR_flowrate_pulses - (ISR_flowrate_pulses / 11);
  flow_rate = (flow_rate * flowRate_seconds) / sensor_pulses_per_unit;  // calculate in units per <time in seconds>

  displayData();
  ISR_flowrate_pulses = 0;
  esp_task_wdt_reset();

  if (doubleResetTimer <= (DOUBLE_RESET_TIME + 1)) {
    doubleResetTimer++;
  }
  if (doubleResetTimer == DOUBLE_RESET_TIME) {
    boot_mode = 0;
    saveBootMode();
    Serial.println("Double reset detection timeout. boot mode set to zero");
  }
}


void oneHundred_ms() {
  uint16_t rawData = getInputs(ten_ms_heartbeat);                           // collect sensor input based on pin_mode
  uint16_t units_to_process = processRawPulses(rawData, ten_ms_heartbeat);  // process sensor input
  units_queued = processActions(units_to_process, ten_ms_heartbeat);        // process units based on mode
  checkButton();
  analog_sense = getAnalog();

#ifdef USE_MODBUS
  syncModbusData();
#endif
}


//-----------------------------------------------------------------------------------------------
void loop() {
  if (millis() > ten_ms_heartbeat + 10) {
    ten_ms_heartbeat = millis();
    oneSecondTimer++;
    oneHundred_msTimer++;
    oneMinuteTimer++;

    if (oneHundred_msTimer > 9) {
      oneHundred_msTimer = 0;
      oneHundred_ms();
    }

    if (oneSecondTimer > 99) {
      oneSecondTimer = 0;
      oneSecond();
    }

    if (oneMinuteTimer > 5999) {
      oneMinuteTimer = 0;
      oneMinute();
    }

    if (output_pulses_todo > 0) {
      output_pulser();
    }
#ifdef USE_MODBUS
    mBus.task();
#endif


    wm.process();
  }

  if ((blink > millis()) || ((digitalRead(pulse_in_pin) != 0) && (active_flow != 0))) {
    digitalWrite(ledPin, ledOn);
  } else {
    digitalWrite(ledPin, !ledOn);
  }

  delay(0);  //allow service routines for single core processors
}
