#include "P25Qxx.h"

const int CS = D2; // your CS pin

P25Qxx flash(CS); // use default SPISettings(8MHz, MSB, MODE0)

void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  flash.begin(); // if you need custom pins: flash.begin(miso_pin, sck_pin, mosi_pin);

  Serial.println("JEDEC:");
  auto id = flash.readJEDEC();
  Serial.printf(" JEDEC: %02X %02X %02X\n", id.manufacturer, id.type, id.capacity);
  Serial.println(flash.identifyString());

  // Simple read example (read first 256 bytes)
  uint8_t buf[256];
  flash.readData(0, buf, sizeof(buf));
  Serial.println("First 16 bytes:");
  for (int i=0;i<16;i++) Serial.printf("%02X ", buf[i]);
  Serial.println();

  // Erase first sector (4K) then write small pattern
  flash.setEraseType(P25Qxx::ERASE_4K);
  Serial.println("Erasing sector 0...");
  if (!flash.erase(0)) Serial.println("Erase timeout!");
  else Serial.println("Erase OK");

  // write "Hello" into addr 0
  const char* msg = "Hello SPI FLASH!";
  if (!flash.writeBuffer(0, (const uint8_t*)msg, strlen(msg)+1)) Serial.println("Write failed");
  else Serial.println("Write OK");

  Serial.println("Reading back:");
  memset(buf,0,sizeof(buf));
  flash.readData(0, buf, 32);
  Serial.println((char*)buf);
}

void loop() {
  // nothing
}
