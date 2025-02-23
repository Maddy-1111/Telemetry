#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// Define LoRa module pins for ESP32
#define LORA_RST  27
#define LORA_DIO0 2
#define LORA_SS 15

// Define SPI channel for ESP32
#define HSPI_CS LORA_SS
#define HSPI_CLK  14
#define HSPI_MISO 12
#define HSPI_MOSI 13

#define LORA_FREQ 433E6
#define BAUD_RATE 115200


void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Initializing LoRa Receiver...");
    SPI.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("LoRa initialization failed!");
        while (1);
    }

    LoRa.setSpreadingFactor(7);  // SF (default)
    LoRa.setSignalBandwidth(125E3);  // BW (default)
    LoRa.setCodingRate4(6);  // CR (default)

    Serial.println("LoRa Receiver Ready");
}

void loop() {

    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        String receivedData = "";
        while (LoRa.available()) {
            receivedData += (char)LoRa.read();
        }

        // Split the received string at ',' and extract first & last float
        int firstComma = receivedData.indexOf(',');
        int lastComma = receivedData.lastIndexOf(',');

        if (firstComma != -1 && lastComma != -1 && firstComma != lastComma) {
            String firstValue = receivedData.substring(0, firstComma);
            String lastValue = receivedData.substring(lastComma + 1);

            Serial.printf("First: %s, Last: %s\n", firstValue.c_str(), lastValue.c_str());
        }
    }


    if (Serial.available() > 0) {
        // Read the incoming data as a string
        String input = Serial.readString();
        
        // Trim any extra spaces or newlines
        input.trim();
        
        // Check if the received string is "reset"
        if (input == "reset") {
            Serial.println("Initiating Software Reset...");
            delay(1000);  // Optional delay to see the message before reset
            ESP.restart();  // Triggers the software reset
        }
    }
}



int crc_verify(char *data, int len, int expected_crc) {
    int crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (unsigned char)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    // Return 0 if CRC matches, non-zero if it doesn't match
    return (crc == expected_crc) ? 0 : 1;
}