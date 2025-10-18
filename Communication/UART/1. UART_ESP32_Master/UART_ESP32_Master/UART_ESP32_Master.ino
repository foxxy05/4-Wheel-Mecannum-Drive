// ESP32 UART Sender using Serial2

void setup() {
  // Initialize Serial2 on pins 16 (RX2), 17 (TX2)
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  delay(1000);
}

void loop() {
  for (int i = 0; i <= 9; i++) {
    Serial2.write(i);   // send raw integer (binary)
    delay(1000);
  }
}
