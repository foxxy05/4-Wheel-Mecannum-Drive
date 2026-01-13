// ESP32-Master
#include <PS4Controller.h>
#include <Wire.h>
#define slaveAddr 10
int8_t data[5];  //Lx Ly Rx Ry Button

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Serial.println("I2C Master Ready!");
  PS4.begin();
  Serial.println("PS4 Ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (PS4.isConnected()) {
    data[0] = map(PS4.LStickX(), -127, 127, 0, 255);
    data[1] = map(PS4.LStickY(), -127, 127, 0, 255);
    data[2] = map(PS4.RStickX(), -127, 127, 0, 255);
    data[3] = map(PS4.RStickY(), -127, 127, 0, 255);
    data[4] = PS4.Touchpad() ? 1 : 0;  // button flag

    Wire.beginTransmission(slaveAddr);
    Wire.write((uint8_t*)data, sizeof(data));
    Wire.endTransmission();

    Serial.print("LX : ");  Serial.print(data[0]);
    Serial.print("   ||   LY : ");  Serial.print(data[1]);
    Serial.print("   ||   RX : ");  Serial.print(data[2]);
    Serial.print("   ||   RY : ");  Serial.print(data[3]);
    Serial.print("   ||   Button : ");  Serial.println(data[4]);
  }
  delay(50);
}
