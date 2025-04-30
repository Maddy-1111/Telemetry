#include <Arduino.h>

#define RXD1 16  // ESP32 RX
#define TXD1 17  // ESP32 TX

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1); // LoRa UART
  Serial.println("Transmitter Ready");

  delay(1000);
}

void loop() {
  Serial1.println("Hello from ESP32 Transmitter!");
  Serial.println("Message sent.");
  delay(2000);  // Send every 2 seconds
}

