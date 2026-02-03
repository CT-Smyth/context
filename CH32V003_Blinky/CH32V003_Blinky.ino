
// PD6 198
// PA2 192
// PD4 199
// PC4 194
// PC2 44
// PC1 3
// A0 192
// A1 193

// the setup function runs once when you press reset or power the board
int test_pin = PC4;


void setup() {
  //delay (5000);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(test_pin, OUTPUT);
}

// the loop function runs over and over again forever
// void loop() {
//   //pinMode(PC1, INPUT_PULLUP);
//   digitalWrite(PC1, HIGH);  // turn the LED on (HIGH is the voltage level)

//   delayMicroseconds(300 - pwmTime / 10); 
//                        // wait for a second
//  // pinMode(PC1, OUTPUT);
//   digitalWrite(PC1, LOW);   // turn the LED off by making the voltage LOW
//   delayMicroseconds(pwmTime);
//   //delayMicroseconds(300);
//   //Serial.print("oogabooga!");                      // wait for a second
//   pwmTime = pwmTime + 30;
//   if (pwmTime > 3000) {pwmTime = 0;}
// }

void loop() {
  digitalWrite(test_pin, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay (10);
  digitalWrite(test_pin, LOW);   // turn the LED off by making the voltage LOW
  delay (100);
}
