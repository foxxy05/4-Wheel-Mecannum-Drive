// Arduino Mega UART Receiver using Serial1
int incomingValue;

void setup() {
  Serial.begin(115200);    // for Serial Monitor
  Serial1.begin(9600);   // for ESP32 comms (pins 19=RX1, 18=TX1)
  Serial.println("Ready to receive digits from ESP32 on Serial1...");
}

void loop() {
  if (Serial1.available() > 0) {
    incomingValue = Serial1.read(); // read raw byte
    Serial.print("Received digit: ");
    Serial.println(incomingValue);  // display in Serial Monitor
  }
}
