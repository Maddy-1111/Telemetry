#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <vector>
#include <cstring>  // Required for memcpy

// LoRa module pins for ESP32
#define LORA_RST  27
#define LORA_DIO0 2
#define LORA_SS   15

// SPI channel for ESP32
#define HSPI_CS   LORA_SS
#define HSPI_CLK  14
#define HSPI_MISO 12
#define HSPI_MOSI 13

#define LORA_FREQ 433E6
#define BAUD_RATE 115200

// Global vector to hold received data (assumed to be filled elsewhere)
std::vector<float> receivedData;
#define CHUNK_SIZE 60
#define HEADER_SIZE 4 			// for the crc

std::vector<std::vector<float>> chunkVector(const std::vector<float>& vec, int chunkSize);
bool readSerial(std::vector<float>& dataArray);
int generateCRC(const std::vector<float>& data);
void sendLoRaPacket(const std::vector<float>& chunk, int crc);
void fillArray(std::vector<float>& arr, size_t size);


void setup() {
	Serial.begin(115200);
	while (!Serial);
	Serial.println("Initializing LoRa Transmitter...");

	SPI.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
	LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
	
	if (!LoRa.begin(LORA_FREQ)) {
		Serial.println("LoRa initialization failed!");
		while (1);
	}

	LoRa.setSpreadingFactor(7);
	LoRa.setSignalBandwidth(125E3);
	LoRa.setCodingRate4(6);

	Serial.println("LoRa Transmitter Ready");

	fillArray(receivedData, 600);
}

void loop() {

	if(readSerial(receivedData)) {
		Serial.printf("First 3: %.10f, %.10f, %.10f ... Last 2: %.2f, %.2f\n", 
			receivedData[0], receivedData[1], receivedData[2], 
			receivedData[receivedData.size() - 2], receivedData[receivedData.size() - 1]);
	}

	std::vector<float> dataCopy = receivedData;
	auto chunks = chunkVector(dataCopy, CHUNK_SIZE);

	// For each chunk, calculate its CRC and send via LoRa
	// for (auto &chunk : chunks) {
	// 	int crc = generateCRC(chunk);
	// 	sendLoRaPacket(chunk, crc);

	// 	if(chunk.size() > 2) {
	// 		Serial.printf("First 3: %.10f, %.10f, %.10f ... Last 2: %.2f, %.2f\n", 
	// 					chunk[0], chunk[1], chunk[2], 
	// 					chunk[chunk.size() - 2], chunk[chunk.size() - 1]);
	// 	}
	// 	else { Serial.printf("Size of chunk < 3"); }

	// 	delay(100);
			
	// }

	// Serial.printf("loop\n");

	delay(100);  // Delay before next loop iteration
}



// Helper function: Chunk a vector into sub-vectors of given size
std::vector<std::vector<float>> chunkVector(const std::vector<float>& vec, int chunkSize) {
	std::vector<std::vector<float>> chunks;
	for (size_t i = 0; i < vec.size(); i += chunkSize) {
		auto start = vec.begin() + i;
		auto end = (i + chunkSize < vec.size()) ? start + chunkSize : vec.end();
		chunks.push_back(std::vector<float>(start, end));
	}
	return chunks;
}

// Helper function: Calculate CRC from a vector of floats by processing each float's byte representation
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


bool readSerial(std::vector<float>& dataArray) {
	std::vector<float> dataBuffer;  // Temporary buffer to store received packets
	bool new_data = false;
	while (Serial.available()) {
		uint8_t length;				// number of floats being received
		Serial.readBytes(&length, 1);

		if (length > 0 && length <= CHUNK_SIZE) {
            float packet[length];
            Serial.readBytes((char*)packet, length * sizeof(float));  // Read float bytes
			
			dataBuffer.insert(dataBuffer.end(), packet, packet + length);
			new_data = true;
			Serial.println("ack");
			delay(50);		///////////////////// KEEP THIS IN MIND, 20ms works for my laptop ///////////////////
        }
        else {
			Serial.println("invalid length");
			return false;	// invalid data length
		}
	}
	
	if (new_data) {
		dataArray = dataBuffer;
	}
    return new_data;  // No new data received
}


void fillArray(std::vector<float>& arr, size_t size) {
    arr.resize(size); // Resize to the required size
    for (size_t i = 0; i < arr.size(); ++i) {
        arr[i] = static_cast<float>(i + 1) + 1e-4f;
    }
}


// Sends the data as bytes, provided the vector chunk (float) and crc (int)
void sendLoRaPacket(const std::vector<float>& chunk, int crc) {
    if (chunk.empty()) return; // Avoid sending empty packets

    std::vector<uint8_t> packetData(chunk.size() * sizeof(float) + sizeof(int));

    std::memcpy(packetData.data(), chunk.data(), chunk.size() * sizeof(float));
    std::memcpy(packetData.data() + (chunk.size() * sizeof(float)), &crc, sizeof(int));

    LoRa.beginPacket();
    LoRa.write(packetData.data(), packetData.size());
    Serial.printf("Packet size: %d \n", packetData.size());
    LoRa.endPacket();
}