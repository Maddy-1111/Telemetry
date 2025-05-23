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

#define DATA_SIZE 248


std::vector<float> Data;

bool readSerial(std::vector<float>& dataArray);
int generateCRC(const std::vector<float>& data);
void sendLoRaPacket(const std::vector<float>& chunk, int crc);
void fillArray(std::vector<float>& arr, size_t size);



void setup() {
	Serial.begin(BAUD_RATE);
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

	fillArray(Data, DATA_SIZE);
}



void loop() {

	if(readSerial(Data)) {
		Serial.printf("First 3: %.10f, %.10f, %.10f ... Last 2: %.2f, %.2f\n", 
			Data[0], Data[1], Data[2], 
			Data[Data.size() - 2], Data[Data.size() - 1]);

			int crc = generateCRC(Data);
			sendLoRaPacket(Data, crc);
// TODO: will probably fail without the prints in the sendLoRaPacket
// can fix it by just waiting till it receives ack to send next packet or smthng
			Serial.println("Sent Lora Packet");
		}
//////// NOTE: delay at end of loop must be more than timeout on uplink ///////
	// delay(750);			//TODO: worked till 600ms (depends on LoRa settings) ////////// was this line, replaced it with the below delay ////////////////////
		delay(50);
}




// TODO: make the crc take in a bytes array and generate a byte out of it (will be able to validate chunk index too)
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
	std::vector<float> dataBuffer;
	bool data_valid = false;
	while (Serial.available()) {
		uint8_t length;				// number of floats being received
		Serial.readBytes(&length, 1);

		if (length > 0 && length <= DATA_SIZE) {
            float packet[length];						//////////////////TODO: rewrite this with using dataBuffer directly or only using packet /////////////
            Serial.readBytes((char*)packet, length * sizeof(float));  // Read float bytes
			
			dataBuffer.insert(dataBuffer.end(), packet, packet + length);
			data_valid = true;
			Serial.println("ack");
			delay(50);		// TODO: KEEP THIS IN MIND, 20ms works for my laptop 
        }
        else {
			Serial.println("invalid length!");
			return false;	// invalid data length
		}
	}
	
	if (data_valid) {
		dataArray = dataBuffer;
	}
    return data_valid;  // No new data received
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

	std::memcpy(packetData.data(), chunk.data(), chunk.size() * sizeof(float));  // Copy chunk data
	std::memcpy(packetData.data() + (chunk.size() * sizeof(float)), &crc, sizeof(int));  // Copy CRC at the end
	
    LoRa.beginPacket();
    LoRa.write(packetData.data(), packetData.size());
    Serial.printf("sent packet, size: %d \n", packetData.size());
    LoRa.endPacket();
}