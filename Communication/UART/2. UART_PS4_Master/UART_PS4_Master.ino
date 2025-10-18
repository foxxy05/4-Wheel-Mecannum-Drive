#include <PS4Controller.h>
int8_t lx = 0;
int8_t ly = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // UART2 on pins 16=RX, 17=TX

  PS4.begin();  // Replace with your ESP32 MAC address
  Serial.println("Waiting for PS4 controller...");
}

void loop() {
  if (PS4.isConnected()) {
    // down = PS4.Down();  // Range: -128 to 127
    // Serial2.println(down);     // Send as text with newline
    // up = PS4.Up();  // Range: -128 to 127
    // Serial2.println(up);     // Send as text with newline
    lx = PS4.LStickX();
    Serial2.write(lx);6
    ly = PS4.LStickY();
    Serial2.write(ly);

    delay(50);  // ~20 updates per second
  }
}
