#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("I2C SCANNER: SDA=A4 SCL=A5, старт через 3 с...");
  delay(3000);
  Serial.println("сканирую адреса 1..126");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("НАЙДЕН: 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("скан завершён");
}

void loop() { delay(1000); }
