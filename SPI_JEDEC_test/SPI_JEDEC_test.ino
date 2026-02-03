#include <SPI.h>
const int CS = D2;
const int otherCS = D5;
#define SERIAL_PORT Serial

void setup() {
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, 0);

  Serial.begin(115200);
  //delay(1000);
  while (!SERIAL_PORT) {};

  pinMode(otherCS, OUTPUT);  //set other CS pin high
  digitalWrite(otherCS, 1);

  SPI.begin();
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  delay(100);

  Serial.println("Reading JEDEC ID...");

  digitalWrite(CS, LOW);
  SPI.transfer(0x9F);
  uint8_t mfg = SPI.transfer(0);
  uint8_t memtype = SPI.transfer(0);
  uint8_t capacity = SPI.transfer(0);
  digitalWrite(CS, HIGH);

  Serial.print("Manufacturer ID: 0x");
  Serial.println(mfg, HEX);
  Serial.print("Memory Type: 0x");
  Serial.println(memtype, HEX);
  Serial.print("Capacity: 0x");
  Serial.println(capacity, HEX);
}
void loop() {
  Serial.println("Reading JEDEC ID...");

  digitalWrite(CS, LOW);
  SPI.transfer(0x9F);
  uint8_t mfg = SPI.transfer(0);
  uint8_t memtype = SPI.transfer(0);
  uint8_t capacity = SPI.transfer(0);
  digitalWrite(CS, HIGH);

  Serial.print("Manufacturer ID: 0x");
  Serial.println(mfg, HEX);
  Serial.print("Memory Type: 0x");
  Serial.println(memtype, HEX);
  Serial.print("Capacity: 0x");
  Serial.println(capacity, HEX);

  digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
  
  delay(100);
}
