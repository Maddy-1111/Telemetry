#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"
#include "utils.h"


HardwareSerial E32Serial(2);

void setup() {
  Serial.begin(USB_BAUD);
  E32Serial.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);

  Serial.println("Entering config mode... :)");
  enterSleepMode();

//   apply915TransparentConfig();

  Serial.println("Reading config...");
  readAndPrintConfig();

  Serial.println("Returning to normal mode...");
  enterNormalMode();
}


void loop() {
  if (E32Serial.available()) {
    String received = E32Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(received);
  }
}