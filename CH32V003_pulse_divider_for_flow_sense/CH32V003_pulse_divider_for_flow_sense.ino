/*
  CH32V003 Flow Sensor Pulse Divider
  ----------------------------------
  Reads pulses from a Hall-effect sensor, toggles an output every PULSES_PER_TOGGLE pulses,
  and drives a PWM LED to indicate input activity. (5 PULSES_PER_TOGGLE = 10 pulses rising to rising on output)

  Features:
    - The LED output (PC4) reflects input activity for 5 seconds after the last detected pulse.
    - The Q1 output toggles state every `PULSES_PER_TOGGLE` valid rising edges.
    - PWM brightness of the LED is proportional to pulse count within the current cycle.

  Hardware mapping:
    PC4 -> LED / PWM output (TIM1_CH4)
    PC2 -> Hall-effect sensor input (active-low or toggling)
    PD6 -> Q1 output (toggles every N pulses)
    PC1 -> RX (not used)
    PC4 -> TX (shared pin, ensure no conflict if serial is used)

  Notes:
    - Uses hardware PWM (TIM1_CH4) for LED brightness control.
    - Input is sampled via polling (no interrupts) for simplicity and reliability at <1 kHz pulse rates.
    - LED automatically turns off 5 seconds after the last detected input transition.
*/


// -------------------- Pin and Configuration Constants --------------------
constexpr uint8_t LED_pin = PC4;           // PWM LED output (TIM1_CH4)
constexpr uint8_t HALL_pin = PC2;          // Hall sensor input pin
constexpr uint8_t Q1_pin = PD6;            // Output toggled after N pulses
constexpr uint8_t TX_pin = PC4;            // TX (not used)
constexpr uint8_t RX_pin = PC1;            // RX (not used)
constexpr uint8_t PULSES_PER_TOGGLE = 5;  // Number of pulses before toggling output


// -------------------- State Variables --------------------
bool ledState = 0;     // LED enable flag (on/off)
bool lastState = 0;    // Previous Hall sensor state
bool hallState = 0;    // Current Hall sensor state

int hallCount = 10;    // Pulse counter (starts >0 so LED begins lit)
int pulsesToDo = 0;    // Reserved for future logic (currently unused)

u_long LEDtime = 0;    // Timestamp when LED should turn off (ms)


// -------------------- Setup --------------------
void setup() {
  // Configure GPIO directions
  pinMode(LED_pin, OUTPUT);
  pinMode(Q1_pin, OUTPUT);
  pinMode(HALL_pin, INPUT_PULLUP);

  // --- Enable peripheral clocks for GPIOC and TIM1 ---
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1;

  // --- Configure PC4 as Alternate Function Push-Pull (TIM1_CH4) ---
  GPIOC->CFGLR &= ~(0xF << (4 * 4));  // Clear mode bits for PC4
  GPIOC->CFGLR |=  (0xB << (4 * 4));  // 0xB = 1011b → 50 MHz AF Push-pull

  // --- Set up TIM1 for PWM output on Channel 4 (PC4) ---
  TIM1->PSC = 47;           // Prescaler → 48 MHz / (47 + 1) = 1 MHz timer clock
  TIM1->ATRLR = 1000 - 1;   // Auto-reload = 1000 → PWM period = 1 kHz
  TIM1->CH4CVR = 0;         // Initial duty cycle = 0%

  // Configure PWM mode 1 (OC4M = 110) with preload enabled
  TIM1->CHCTLR2 &= ~(TIM_OC4M | TIM_OC4PE);
  TIM1->CHCTLR2 |= (0x6 << 12) | TIM_OC4PE;

  // Enable channel output and timer main output
  TIM1->CCER  |= TIM_CC4E;   // Enable CH4 output
  TIM1->CTLR1 |= TIM_ARPE;   // Enable auto-reload preload
  TIM1->BDTR  |= TIM_MOE;    // Main output enable (required for advanced timers)

  // Start PWM generation
  TIM1->SWEVGR |= TIM_UG;    // Force update event to load registers
  TIM1->CTLR1  |= TIM_CEN;   // Enable timer counter

  // Initialize LED timer and Hall state
  LEDtime = millis() + 5000;          
  hallState = digitalRead(HALL_pin);  
  lastState = hallState;
}


// -------------------- Main Loop --------------------
void loop() {
  // --- Read Hall sensor and detect transitions ---
  hallState = digitalRead(HALL_pin);
  if (hallState != lastState) {           // Triggered on any state change

    LEDtime = millis() + 5000;            // Extend LED active window by 5s

    if (hallState == 1) {                 // Rising edge detected
      hallCount++;

      // Toggle output every N pulses
      if (hallCount >= PULSES_PER_TOGGLE) {
        digitalWrite(Q1_pin, !digitalRead(Q1_pin));  // Toggle output
        hallCount = 0;
      }
    }

    lastState = hallState;                // Save state for next iteration
  }

  // --- LED timeout logic ---
  // LED stays on for 5 seconds after the last pulse; then shuts off
  if (LEDtime < millis()) {
    ledState = 0;
  } else {
    ledState = 1;
  }

  // --- Update LED brightness via PWM ---
  // Brightness is proportional to hallCount when LED is active
  TIM1->CH4CVR = 100 * hallCount * ledState;  // 0–1000 scale (since ARR = 999)
}

