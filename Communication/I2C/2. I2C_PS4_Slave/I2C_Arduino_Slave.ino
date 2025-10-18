#include <Wire.h>
#define slave_addr 5
int dataToSend = 50;

void requestEvent() {
  Wire.write(dataToSend);
}

void receiveEvent(int bytes) {

  if (Wire.available() >= 2) {
    uint16_t MSB = Wire.read();
    uint16_t LSB = Wire.read();
    uint16_t result = (MSB << 8) | LSB;
    signed int receivedData = map(result, 0, 65535, -32767, 32768);

    if (receivedData >= 0) {
      receivedData += 1;
    }
    Serial.println(receivedData);
  }
}

void setup() {

  Serial.begin(9600);
  Wire.begin(slave_addr);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
}
