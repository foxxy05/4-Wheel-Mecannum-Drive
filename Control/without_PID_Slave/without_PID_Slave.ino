// (SLAVE) PS4-Equation
#include <PS4Controller.h>
#include <Wire.h>
#define slaveAddr 10
#define arraySize 4
#define buffer 10
uint8_t lx = 0, ly = 0, l2 = 0, r2 = 0;
uint8_t data[arraySize] = { 0 };  //Lx  Ly  L2  R2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  PS4.begin();
  Serial.println("PS4 Ready!");
  PS4.attach(receivePS);

  Wire.begin(slaveAddr);
  Wire.onRequest(onRequestEvent);
  Serial.println("I2C Slave Ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!PS4.isConnected()){
    data[0] = 128;
    data[1] = 128;
    data[2] = 0;
    data[3] = 0;
  }
  
  printPS();
  delay(50);
}

void onRequestEvent(){
  Wire.write((uint8_t*)data, sizeof(data));
}

void receivePS() {
  if (PS4.isConnected()) {
    lx = PS4.LStickX();
    ly = PS4.LStickY();
    l2 = PS4.L2Value();
    r2 = PS4.R2Value();

    if (abs(lx) < buffer) lx = 0;
    if (abs(ly) < buffer) ly = 0;
    if (l2 < (buffer/2)) l2 = 0;
    if (r2 < (buffer/2)) r2 = 0;    

    data[0] = map(lx, -127, 127, 0, 255);
    data[1] = map(ly, -127, 127, 0, 255);
    data[2] = l2;
    data[3] = r2;
  }
}

void printPS() {
  Serial.print("LX : ");
  Serial.print(data[0]);
  Serial.print("   ||    LY : ");
  Serial.print(data[1]);
  Serial.print("   ||   L2 : ");
  Serial.print(data[2]);
  Serial.print("   ||   R2 : ");
  Serial.println(data[3]);
  // Serial.print("   ||   Button : ");
  // Serial.println(data[4]);
}
