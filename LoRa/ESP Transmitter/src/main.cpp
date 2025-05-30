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
	pinMode(AUX_PIN, INPUT);

	Serial.println("Entering config mode... :)");
	enterSleepMode();

	//   apply915TransparentConfig();

	Serial.println("Reading config...");
	readAndPrintConfig();

	Serial.println("Returning to normal mode...");
	enterNormalMode();
}


void loop() {
	uint8_t data_head[sizeof(header)] = {0};
	uint8_t data_len[2] = {0};
	uint8_t data_crc[2] = {0};
	uint8_t data_type[1] = {0};
	uint8_t data_buf[MAX_DATA_LEN] = {0};

	while (Serial.available() < sizeof(header)) delay(2);
	Serial.readBytes(data_head, sizeof(header));
	if (!checkHeader(data_head)) {
		flushSerial(Serial);
		return;
	}

	while (Serial.available() < 2) delay(2);
	Serial.readBytes(data_len, 2);
	uint16_t length = (data_len[1] << 8) | data_len[0];
	if (length == 0 || length > MAX_DATA_LEN) {
		flushSerial(Serial);
		return;
	}

	while (Serial.available() < 2) delay(2);
	Serial.readBytes(data_crc, 2);

	while (Serial.available() < 1) delay(2);
	Serial.readBytes(data_type, 1);

	while (Serial.available() < length) delay(2);
	Serial.readBytes(data_buf, length);

/////////// Sending packet via LoRa: ////////////

	E32Serial.write(data_head, sizeof(data_head));
	E32Serial.write(data_len, 2);
	E32Serial.write(data_crc, 2);
	E32Serial.write(data_type, 1);		
	E32Serial.write(data_buf, length);

	// uint16_t total_len = 2 + 2 + 1 + length + sizeof(data_term);
	// uint8_t packet[total_len];
	// uint16_t pos = 0;
	// memcpy(packet + pos, data_len, 2);
	// pos += 2;
	// memcpy(packet + pos, data_crc, 2);
	// pos += 2;
	// memcpy(packet + pos, data_type, 1);
	// pos += 1;
	// memcpy(packet + pos, data_buf, length);
	// pos += length;
	// memcpy(packet + pos, data_term, sizeof(data_term));

	// E32Serial.write(packet, total_len);

Serial.write("LoRa data Sent!!!\n");

	// while (digitalRead(AUX_PIN) == LOW) delay(2);
	delay(2000);
}