#include <Wire.h>
#define slaveAddr 5
uint16_t dataToSend = 10;

void setup() {
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  uint16_t conversion = map(dataToSend, -32767, 32768, 0, 65535);
  uint16_t MSB = highByte(conversion);
  uint16_t LSB = lowByte(conversion);

  Wire.beginTransmission(slaveAddr);
  Wire.write(MSB);
  Wire.write(LSB);
  Wire.endTransmission();


  Wire.requestFrom(slaveAddr, 1);
  if (Wire.available()) {
    int receivedData = Wire.read();
    Serial.print("Data from slave ");
    Serial.println(receivedData);
  }
}
