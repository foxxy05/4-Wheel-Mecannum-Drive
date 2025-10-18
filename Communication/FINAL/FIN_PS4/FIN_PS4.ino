// ESP32-Master
#include <PS4Controller.h>
#include <Wire.h>
#define slaveAddr 10
uint8_t data[5];  //Lx Ly Rx Ry Buttons

void recievePS() {
  if (PS4.isConnected()) {
    data[0] = map(PS4.LStickX(), -127, 127, 0, 255);
    data[1] = map(PS4.LStickY(), -127, 127, 0, 255);
    data[2] = map(PS4.RStickX(), -127, 127, 0, 255);
    data[3] = map(PS4.RStickY(), -127, 127, 0, 255);

    data[4] = 0;  //L R U D  Sq Ci Tr Cr
    if (PS4.Left())       data[4] |= (1 << 7);
    if (PS4.Right())      data[4] |= (1 << 6);
    if (PS4.Up())         data[4] |= (1 << 5);
    if (PS4.Down())       data[4] |= (1 << 4);
    if (PS4.Square())     data[4] |= (1 << 3);
    if (PS4.Circle())     data[4] |= (1 << 2);
    if (PS4.Triangle())   data[4] |= (1 << 1);
    if (PS4.Cross())      data[4] |= (1 << 0);
  }
}

void printPS() {
  Serial.print("LX : ");
  Serial.print(data[0]);
  Serial.print("   ||   LY : ");
  Serial.print(data[1]);
  Serial.print("   ||   RX : ");
  Serial.print(data[2]);
  Serial.print("   ||   RY : ");
  Serial.print(data[3]);
  Serial.print("   ||   Button : ");
  Serial.println(data[4]);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Serial.println("I2C Master Ready!");
  PS4.begin();
  Serial.println("PS4 Ready!");
  PS4.attach(printPS);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(slaveAddr);
  Wire.write((uint8_t*)data, sizeof(data));
  Wire.endTransmission();
  printPS();

  delay(50);
}
