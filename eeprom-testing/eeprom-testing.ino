#define USE_LGT_EEPROM_API
#include <EEPROM.h>

byte b1;
byte b2;
byte b3;
byte b4;
byte b5;
byte b6;
byte b7;
byte b8;
byte b9;
int pulses;
int pulsesPerCount = 1000;
int count;
int countToStore = 10;
unsigned long millisInSecond = 1000;
unsigned long StorageIntervalInSeconds = 14400; //14400 is 4 hours
unsigned long lastMillis;
unsigned long storeInterval = (millisInSecond * StorageIntervalInSeconds);





void initFlash(){
  Serial.print("\n\n Initializing Flash....");
    for (int i = 0; i < EEPROM.size(); i += 10){
      EEPROM.write(i, 0);
      digitalWrite(13, (!digitalRead(13)));
    } 
    EEPROM.write (10,255);
  Serial.println("...Initialized.");
  writeFlash();
}


int getLastAddress(){
  //Serial.println("identifying last entry address");
   int address;
   int value;
   for ( address = 1000; address >10; address -= 10){
      value = EEPROM.read(address);
      // Serial.print(address);
      // Serial.print(" = ");
      // Serial.println(value);
      if (value == 255) return (address);
   } 
   return 10;
}

int writeFlash(){
   int address = getLastAddress();
   address = address + 10;
   if (address > 1000){initFlash();address = 10;}
   Serial.print("\nWriting to address ");
   Serial.println(address);
   EEPROM.write(address-1,b1);
   EEPROM.write(address-2,b2);
   EEPROM.write(address-3,b3);
   EEPROM.write(address-4,b4);
   EEPROM.write(address-5,b5);
   EEPROM.write(address-6,b6);
   EEPROM.write(address-7,b7);
   EEPROM.write(address-8,b8);
   EEPROM.write(address-9,b9);
   EEPROM.write(address,255);
}

int readFlash(){
   int address = getLastAddress();
   Serial.print("\nReading from address ");
   Serial.println(address);
   EEPROM.write(address,255);
   b1 = EEPROM.read(address-1);
   b2 = EEPROM.read(address-2);
   b3 = EEPROM.read(address-3);
   b4 = EEPROM.read(address-4);
   b5 = EEPROM.read(address-5);
   b6 = EEPROM.read(address-6);
   b7 = EEPROM.read(address-7);
   b8 = EEPROM.read(address-8);
   b9 = EEPROM.read(address-9);
}

void onPulse(){
  //Serial.println(millis() - lastMillis);
  //Serial.println(storeInterval);

  pulses ++;

  if (pulses >= pulsesPerCount){
    pulses = 0;
    count ++;
    b1 ++;

    if (b1>9) {b1 = 0; b2 ++;}
    if (b2>9) {b2 = 0; b3 ++;}
    if (b3>9) {b3 = 0; b4 ++;}
    if (b4>9) {b4 = 0; b5 ++;}
    if (b5>9) {b5 = 0; b6 ++;}
    if (b6>9) {b6 = 0; b7 ++;}
    if (b7>9) {b7 = 0; b8 ++;}
    if (b8>9) {b8 = 0; b9 ++;}

    if (b9>9) {
      b9 = 0;
      b8 = 0;
      b7 = 0;
      b6 = 0;
      b5 = 0;
      b4 = 0;
      b3 = 0;
      b2 = 0;
      b1 = 0;
    }



    if(count >= countToStore){
      lastMillis = millis();
      count = 0;
      writeFlash();
    }

      Serial.print(b9);
      Serial.print(b8);
      Serial.print(b7);
      Serial.print(b6);
      Serial.print(b5);
      Serial.print(b4);
      Serial.print(b3);
      Serial.print(b2);
      Serial.print(b1);
      Serial.println("\n\n\n");

  }


  

}

void setup()
{
  Serial.begin(250000);
  if(EEPROM.read(EEPROM.size() - 1) != 0) initFlash(); //verify first init
  readFlash();
  Serial.print(b9);
  Serial.print(b8);
  Serial.print(b7);
  Serial.print(b6);
  Serial.print(b5);
  Serial.print(b4);
  Serial.print(b3);
  Serial.print(b2);
  Serial.print(b1);
  Serial.println("\n\n\n");
}

void loop()
{

  onPulse();


  if ((millis() - lastMillis) > storeInterval){
      lastMillis = millis();
      count = 0;
      writeFlash();
  }


  delay(1);
}