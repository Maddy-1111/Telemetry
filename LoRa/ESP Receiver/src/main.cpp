#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <vector>
#include <cstring>  // Required for memcpy


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

#define CHUNK_SIZE 60
#define DATA_SIZE 600

std::vector<float> receivedData;

int generateCRC(const std::vector<float>& data, int expected_crc);
void fillArray(std::vector<float>& arr, size_t size);
std::vector<float> decodeLoRaPacket();
void writeSerial(std::vector<float>& dataArray);


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

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(6);

    Serial.println("LoRa Receiver Ready");

	fillArray(receivedData, DATA_SIZE);
}

void loop() {

    int packetSize = LoRa.parsePacket();
    while(packetSize) {
        
		std::vector<float> packetData = decodeLoRaPacket();
		Serial.printf("First 3: %.10f, %.10f, %.10f ... Last 2: %.2f, %.2f\n", 
			packetData[0], packetData[1], packetData[2], 
			packetData[packetData.size() - 2], packetData[packetData.size() - 1]);

		int packetSize = LoRa.parsePacket();
    }

	// writeSerial(receivedData);

	// Serial.printf("First 3: %.10f, %.10f, %.10f ... Last 2: %.2f, %.2f\n", 
	// 	receivedData[0], receivedData[1], receivedData[2], 
	// 	receivedData[receivedData.size() - 2], receivedData[receivedData.size() - 1]);

    if (Serial.available() > 0) {
        String input = Serial.readString();
        input.trim();

        if (input == "reset") {
            Serial.println("Initiating Software Reset...");
            delay(1000);  // Optional delay to see the message before reset
            ESP.restart();
        }
    }
}



int generateCRC(const std::vector<float>& data) {
	int crc = 0xFFFF;
	for (size_t i = 0; i < data.size(); i++) {
		uint8_t byteArray[sizeof(float)];
		memcpy(byteArray, &data[i], sizeof(float));
		for (int j = 0; j < sizeof(float); j++) {
		crc ^= byteArray[j];
		for (int k = 0; k < 8; k++) {
			if (crc & 0x01) {
			crc = (crc >> 1) ^ 0xA001;
			} else {
			crc = crc >> 1;
			}
		}
		}
	}
	return crc;
}


void fillArray(std::vector<float>& arr, size_t size) {
    arr.resize(size); // Resize to the required size
    for (size_t i = 0; i < arr.size(); ++i) {
        arr[i] = static_cast<float>(i + 1) + 1e-4f;
    }
}


void decodeLoRaPacket() {
	std::vector<uint8_t> receivedBytes(packetSize);
        
        for (int i = 0; i < packetSize; i++) {
            receivedBytes[i] = LoRa.read();
        }

        // Ensure we have at least 4 bytes for the checksum
        if (receivedBytes.size() < sizeof(int)) {
            Serial.println("Error: Packet too small!");
            return;
        }

        int receivedCRC;
        std::memcpy(&receivedCRC, &receivedBytes[receivedBytes.size() - sizeof(int)], sizeof(int));

        size_t numFloats = (receivedBytes.size() - sizeof(int)) / sizeof(float);
        std::vector<float> receivedData(numFloats);

        for (size_t i = 0; i < numFloats; i++) {
            std::memcpy(&receivedData[i], &receivedBytes[i * sizeof(float)], sizeof(float));
        }

        // Print first 3 and last 2 elements
        Serial.printf("Received %d floats. CRC: %d\n", (int)receivedData.size(), receivedCRC);
        if(generateCRC(receivedData) == receivedCRC) {
            Serial.printf("VALID\n");
        } else {
            Serial.printf("INVALID\n");
        }
}
