#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"
#include "utils.h"


HardwareSerial E32Serial(2);

void setup() {
	Serial.begin(USB_BAUD);
	Serial.setTimeout(TIMEOUT); 
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
	uint8_t data_len[2] = {0};
	uint8_t data_crc[2] = {0};
  uint8_t data_type[1] = {0};
	uint8_t data_buf[MAX_DATA_LEN] = {0};
	uint8_t data_term[sizeof(terminator)] = {0};

	while (E32Serial.available() < 2) delay(1);
	E32Serial.readBytes(data_len, 2);
	uint16_t length = (data_len[1] << 8) | data_len[0];
// Serial.println("Checkpoint A");
// Serial.println(length);
	if (length == 0 || length > 512) {
		flushSerial(E32Serial);
		return;
	}
// Serial.println("Checkpoint B");
	while (E32Serial.available() < 2) delay(1);
	E32Serial.readBytes(data_crc, 2);

  while (E32Serial.available() < 1) delay(1);
	E32Serial.readBytes(data_type, 1);

	while (E32Serial.available() < length) delay(1);
	E32Serial.readBytes(data_buf, length);

	while (E32Serial.available() < sizeof(terminator)) delay(1);
	E32Serial.readBytes(data_term, sizeof(terminator));

	if (checkTerminator(data_term)) {
		flushSerial(E32Serial);
/////////// Sending packet via USB ////////////

		Serial.write(data_len, 2);
		Serial.write(data_crc, 2);
    	Serial.write(data_type, 1);
		Serial.write(data_buf, length);
		Serial.write(data_term, sizeof(data_term));

	}
}