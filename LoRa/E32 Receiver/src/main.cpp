#include <Arduino.h>

#define RXD1 16  // ESP32 RX
#define TXD1 17  // ESP32 TX

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);  // LoRa UART
  Serial.println("Receiver Ready");
}

void loop() {
  if (Serial1.available()) {
    String msg = Serial1.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(msg);
  }
}
