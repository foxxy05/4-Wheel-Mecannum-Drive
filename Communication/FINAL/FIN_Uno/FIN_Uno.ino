// Arduino-Slave
#include <Wire.h>
#define slaveAddr 10
#define arraySize 5
int8_t receivedData[arraySize];

void printPS(){
  Serial.print("LX : ");  Serial.print(receivedData[0]);
  Serial.print("   ||   LY : ");    Serial.print(receivedData[1]);
  Serial.print("   ||   RX : ");    Serial.print(receivedData[2]);
  Serial.print("   ||   RY : ");    Serial.print(receivedData[3]);
  Serial.print("   ||   L : ");     Serial.print((receivedData[4] & 0b10000000) ? 1 : 0);
  Serial.print("   ||   R : ");     Serial.print((receivedData[4] & 0b01000000) ? 1 : 0);
  Serial.print("   ||   U : ");     Serial.print((receivedData[4] & 0b00100000) ? 1 : 0);
  Serial.print("   ||   D : ");     Serial.print((receivedData[4] & 0b00010000) ? 1 : 0);
  Serial.print("   ||   Sq : ");    Serial.print((receivedData[4] & 0b00001000) ? 1 : 0);
  Serial.print("   ||   Ci : ");    Serial.print((receivedData[4] & 0b00000100) ? 1 : 0);
  Serial.print("   ||   Tr : ");    Serial.print((receivedData[4] & 0b00000010) ? 1 : 0);
  Serial.print("   ||   Cr : ");    Serial.println((receivedData[4] & 0b00000001) ? 1 : 0);
}

void receiveEvent(int howMany) {
  int i = 0;
  while (Wire.available() && i < 5) {
    receivedData[i] = Wire.read();  // one byte at a time
    i++;
  }
  for (int i = 0; i < 4; i++) {
    receivedData[i] = map(receivedData[i], 0, 255, -127, 127);
    if (receivedData[i] >= 0) {
      receivedData[i] += 1;
    }
  }
  printPS();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(10);
  Serial.println("I2C Slave Ready!");
  Wire.onReceive(receiveEvent);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10);
}
