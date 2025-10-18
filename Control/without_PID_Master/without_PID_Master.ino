// (MASTER) Teensy/Arduino-Equation
// BTS Setup
#include <BTS7960.h>
#define maxPWM 50
// #define minPWM 0
// Add proper pins for PWM and enable according to board of choice
// Directly add the PWM and enable pins when declaring the object
BTS7960 LF(12, 11);
BTS7960 RF(9, 10);
BTS7960 LR(5, 6);
BTS7960 RR(7, 8);

//Communication
#include <Wire.h>
// #define lx 0.32  // half-length
// #define ly 0.2   // hald-width
// #define rad 0.55 // radius of the wheels
// #define constVector 0.70710  // 1/sqrt(2)
// #define constVector 1.28564
#define constVector 1
#define L 1  // net-length (lx+ly)
#define buffer 5
// #define constVector 1
#define slaveAddr 10
#define arraySize 4  // LX  LY  L2  R2
int8_t receivedData[arraySize] = { 0 };

// Navigation Variables
int16_t wLF = 0, wRF = 0, wLR = 0, wRR = 0;
int16_t Vx = 0, Vy = 0;
int16_t omega = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("I2C Master Ready!");

  // Setting the enable as HIGH for each BTS
  LF.setEnable(true);
  RF.setEnable(true);
  LR.setEnable(true);
  RR.setEnable(true);
}

void loop() {
  wLF = 0;
  wRF = 0;
  wLR = 0;
  wRR = 0;
  Vx = 0, Vy = 0;
  omega = 0;

  // currentAngle = euler.x();
  requestPS4();

  Vy = receivedData[0];  //Y-Component of the Joystick is the X component of the Chassis
  Vx = receivedData[1];
  omega = receivedData[2] + receivedData[3];

  wLF = constrain(constVector * (Vx + Vy - omega), -maxPWM, maxPWM);
  wRF = constrain(constVector * (Vx - Vy + omega), -maxPWM, maxPWM);
  wLR = constrain(constVector * (Vx - Vy - omega), -maxPWM, maxPWM);
  wRR = constrain(constVector * (Vx + Vy + omega), -maxPWM, maxPWM);

  // int16_t maxVal = max(max(abs(wLF), abs(wRF)), max(abs(wLR), abs(wRR)));
  // if (maxVal > 255) {
  //   wLF = wLF * 255 / maxVal;
  //   wRF = wRF * 255 / maxVal;
  //   wLR = wLR * 255 / maxVal;
  //   wRR = wRR * 255 / maxVal;
  // }

  LF.rotate(wLF);
  RF.rotate(wRF);
  LR.rotate(wLR);
  RR.rotate(wRR);

  // printEq();
  printPS();
  delay(50);
}

void requestPS4() {
  Wire.requestFrom(slaveAddr, sizeof(receivedData));

  int i = 0;
  while (Wire.available() && i < arraySize) {
    uint8_t raw = Wire.read();
    if (i == 0) receivedData[0] = map(raw, 0, 255, -127, 127);       // LX
    else if (i == 1) receivedData[1] = map(raw, 0, 255, -127, 127);  // LY
    else if (i == 2) receivedData[2] = map(raw, 0, 255, 0, 127);     // L2
    else if (i == 3) receivedData[3] = map(raw, 0, 255, 0, -127);    // R2

    i++;
  }

  // if (abs(receivedData[0]) < buffer) receivedData[0] = 0;
  // if (abs(receivedData[1]) < buffer) receivedData[1] = 0;
  // if (abs(receivedData[2]) < buffer) receivedData[2] = 0;
  // if (abs(receivedData[3]) < buffer) receivedData[3] = 0;
}

void printPS() {
  Serial.print("LX : ");
  Serial.print(receivedData[0]);
  Serial.print("   ||   LY : ");
  Serial.print(receivedData[1]);
  Serial.print("   ||   L2 : ");
  Serial.print(receivedData[2]);
  Serial.print("   ||   R2 : ");
  Serial.println(receivedData[3]);
}

void printEq() {
  Serial.print("wLF : ");
  Serial.print(wLF);
  Serial.print("   ||   wRF : ");
  Serial.print(wRF);
  Serial.print("   ||   wLR : ");
  Serial.print(wLR);
  Serial.print("   ||   wRR : ");
  Serial.println(wRR);
}
