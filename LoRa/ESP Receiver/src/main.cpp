#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <vector>
#include <tuple>
#include <cstring>  // Required for memcpy
#include <cmath>


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
std::tuple<uint8_t, std::vector<float>> decodeLoRaPacket(int packetSize);
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
	std::vector<float> dataBuffer;
	uint8_t expectedIndex = 0;

	while (expectedIndex < std::floor(DATA_SIZE/CHUNK_SIZE)) {
        int packetSize = LoRa.parsePacket();
        if (packetSize > 0) {
            // auto [index, packetData] = decodeLoRaPacket(packetSize);
			std::tuple<uint8_t, std::vector<float>> result = decodeLoRaPacket(packetSize);
			uint8_t index = std::get<0>(result);
			std::vector<float> packetData = std::get<1>(result);

            if (index != expectedIndex) {
                Serial.printf("Out-of-order packet received: %d (expected %d).\n", index, expectedIndex);
                return;  // Restart loop() to start over
            }

            dataBuffer.insert(dataBuffer.end(), packetData.begin(), packetData.end());
			Serial.printf("received packet: %d\n", index);

            expectedIndex++;
        }
    }


	receivedData = dataBuffer;
	Serial.printf("First 3: %.10f, %.10f, %.10f ... Last 2: %.2f, %.2f\n", 
		receivedData[0], receivedData[1], receivedData[2], 
		receivedData[receivedData.size() - 2], receivedData[receivedData.size() - 1]);


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


std::tuple<uint8_t, std::vector<float>> decodeLoRaPacket(int packetSize) {
	// std::vector<uint8_t> receivedBytes(packetSize);
        
	// for (int i = 0; i < packetSize; i++) {
	// 	receivedBytes[i] = LoRa.read();
	// }

	// // // Ensure we have at least 4 bytes for the checksum
	// // if (receivedBytes.size() < sizeof(int)) {
	// //     Serial.println("Error: Packet too small!");
	// //     return;
	// // }

	// int receivedCRC;
	// std::memcpy(&receivedCRC, &receivedBytes[receivedBytes.size() - sizeof(int)], sizeof(int));

	// uint8_t index;
	// std::memcpy(&index, &receivedBytes, sizeof(uint8_t));

	// size_t numFloats = (receivedBytes.size() - sizeof(int) - sizeof(uint8_t)) / sizeof(float);
	// std::vector<float> packetData(numFloats);

	// for (size_t i = 0; i < numFloats; i++) {
	// 	std::memcpy(&packetData[i], &receivedBytes[(i * sizeof(float)) + sizeof(uint8_t)], sizeof(float));
	// }

	// // Print first 3 and last 2 elements
	// // Serial.printf("Received %d floats. CRC: %d\n", (int)packetData.size(), receivedCRC);
	// // if(generateCRC(packetData) == receivedCRC) {
	// // 	Serial.printf("VALID\n");
	// // } else {
	// // 	Serial.printf("INVALID\n");
	// // }
	// return {index, packetData};



	if (packetSize <= sizeof(uint8_t) + sizeof(int)) {
        Serial.println("Error: Packet too small!");
        return {255, {}};  // Return an invalid index
    }

    std::vector<uint8_t> receivedBytes(packetSize);
        
    for (int i = 0; i < packetSize; i++) {
        receivedBytes[i] = LoRa.read();
    }

    // Extract packet index
    uint8_t index;
    std::memcpy(&index, receivedBytes.data(), sizeof(uint8_t));  // ✅ Corrected

    // Extract CRC
    int receivedCRC;
    std::memcpy(&receivedCRC, receivedBytes.data() + (packetSize - sizeof(int)), sizeof(int));

    // Extract float data
    size_t numFloats = (packetSize - sizeof(uint8_t) - sizeof(int)) / sizeof(float);
    std::vector<float> packetData(numFloats);

    for (size_t i = 0; i < numFloats; i++) {
        std::memcpy(&packetData[i], receivedBytes.data() + sizeof(uint8_t) + (i * sizeof(float)), sizeof(float));
    }

    return {index, packetData};
}
