int8_t lx = 0;
int8_t ly = 0;

void setup() {
  Serial.begin(115200);  // USB Serial Monitor
  Serial1.begin(9600);   // UART from ESP32 (pins 19=RX1, 18=TX1)
  Serial.println("Ready to receive joystick data...");
}

void loop() {
  if (Serial1.available() >= 2) {
    lx = Serial1.read();  // reads integer until newline
    Serial.print("Received LStickX: "); Serial.print(lx);

    
    ly = Serial1.read();  // reads integer until newline
    Serial.print("     ||    Received LStickY: ");Serial.println(ly);

    delay(10);
  }
}
