// constants won't change. They're used here to set pin numbers:
const int pulsePin = 2; 
const int divBy10pin = 3;
const int divBy100pin = 4; 
const int passthruPin = 5;
const int flowPin_1 = 6;
const int flowPin_2 = 7;
const int longFlowPin = 8;

const int ledPin = 13; 

const unsigned long interval_1 = 1000; //1 second interval
const unsigned long interval_2 = 10000; //10 second interval ---- Also the interval used to discontinue "long flow" after flow stops
const unsigned long longFlow = 3600000; //1 Hour will activate the "long Flow" pin

int pulseState = 0;  
int oldPulseState = 0; //Last state of button

int count_10 = 0; //pulse counter variable
int count_100 = 0; //pulse counter variable
int count_1000 = 0; //pulse counter variable

unsigned long delay_1 = 0; 
unsigned long delay_2 = 0; 
unsigned long timer = millis();
unsigned long flowDuration = 0;
unsigned long flowStart = 0;
unsigned long lastFlow = timer;

//--------------------------------------------------------Initialization
void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(divBy10pin, OUTPUT);
  pinMode(divBy100pin, OUTPUT);
  pinMode(passthruPin, OUTPUT);

  pinMode(flowPin_1, OUTPUT);
  pinMode(flowPin_2, OUTPUT);
  pinMode(longFlowPin, OUTPUT);

  pinMode(pulsePin, INPUT_PULLUP);

  Serial.begin(9600);

  Serial.println();
  Serial.println();
  Serial.println("----------------------------------------------------");
  Serial.println("-------   Pulse divider with flow and paasthru  ----");
  Serial.println("--------------    counts rising edge.   ------------");
  Serial.println("--outputs rise on 5,50,500 fall on 10,100,1000 etc--");
  Serial.println("----------------------------------------------------");
  Serial.println();
  Serial.print(" Pulse input pin (w/ pullup, triggers on ground) D");
  Serial.println(pulsePin);
  Serial.println();
  Serial.println("Pulse Output pins for 10,100,passthru  = D3,D4,D5");
  Serial.println();
  Serial.println("  Pins for 1,10s delay from last flow are D6,D7 ");
  Serial.println("   Pin for longFlow (60 mins) alert is D8 ");
  Serial.println("----------------------------------------------------");
  Serial.println();
  Serial.println();
}

//--------------------------------------------------------Subroutines

void emitPulse_div10() {
  digitalWrite(ledPin, !(digitalRead(ledPin)));
  digitalWrite(divBy10pin, !(digitalRead(divBy10pin)));
  count_10 = 0;
}

void emitPulse_div100() {
  digitalWrite(divBy100pin, !(digitalRead(divBy100pin)));
  count_100 = 0;
}

//--------------------------------------------------------Main Loop
void loop() {
  pulseState = digitalRead(pulsePin); // read the state of the sensor value:
  digitalWrite (passthruPin, pulseState); //non-inverting

  
  if (pulseState != oldPulseState){ // check if the pulsePin state has changed. 
  //if (1){ //DEBUG ONLY

    //Reset timers and activate flow pins
    delay_1 = millis() + interval_1; 
    delay_2 = millis() + interval_2; 
    lastFlow = millis(); // flow was last sensed at (now) time

    digitalWrite(flowPin_1, 1); //flow is happening
    digitalWrite(flowPin_2, 1); //flow is happening

    if(oldPulseState == 0) { //catch only rising pulse edge instead of any change for counting purposes

      count_10 ++;
      count_100 ++;
      count_1000 ++;

      if (count_10 >= 5){
        emitPulse_div10();
      }

      if (count_100 >= 50){
        emitPulse_div100();
      }

    }

  } else { //if the pulse state has not changed

      if  (millis() >= delay_2){ //if interval_2 has passed since last flow (no ongoing flow)
        flowStart = millis(); // set flowStart to now to make it clear there are no long flows happening. If there is a flow, it started (now).
        digitalWrite(longFlowPin, 0); //Reset the long flow pin
      }

  }

  oldPulseState = pulseState; //resync pulse state

  //deactivate flow pins if time since last pulse has expired:
  if (millis() >= delay_1){
    digitalWrite(flowPin_1, 0);
  }

  if (millis() >= delay_2){
    digitalWrite(flowPin_2, 0);
  }

  if ((millis() - flowStart) >= longFlow){ //if flow has been constant (no interruptions > delay_1) for at least (longFlow) milliseconds....                                 
    digitalWrite(longFlowPin, 1); //turn on the long flow pin
  }

}







