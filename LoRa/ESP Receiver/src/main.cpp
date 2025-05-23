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

#define DATA_SIZE 248

std::vector<float> Data;

int generateCRC(const std::vector<float>& data, int expected_crc);
void fillArray(std::vector<float>& arr, size_t size);                       //TODO: remove this function
std::vector<float> decodeLoRaPacket(int packetSize);
void writeSerial(float Rssi, float Snr, std::vector<float>& dataArray);


void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial);

    // Serial.println("Initializing LoRa Receiver...");
    SPI.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_FREQ)) {
        // Serial.println("LoRa initialization failed!");
        while (1);
    }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(6);
    
    // Serial.println("LoRa Receiver Ready");

	fillArray(Data, DATA_SIZE);
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if(packetSize){
        std::vector<float> packetData = decodeLoRaPacket(packetSize);
// Serial.printf("%d \n", packetSize);

        Data = packetData;
        float packetRssi = (float)LoRa.packetRssi();
        float packetSnr = LoRa.packetSnr();

// Serial.printf("First 3: %.10f, %.10f, %.10f ... Last 2: %.2f, %.2f\n", 
// Data[0], Data[1], Data[2], 
// Data[Data.size() - 2], Data[Data.size() - 1]);

        writeSerial(packetRssi, packetSnr, Data);

    }

    // if (Serial.available() > 0) {
    //     String input = Serial.readString();
    //     input.trim();

    //     if (input == "reset") {
    //         Serial.println("Initiating Software Reset...");
    //         delay(1000);  // Optional delay to see the message before reset
    //         ESP.restart();
    //     }
    // }
    delay(50);
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


std::vector<float> decodeLoRaPacket(int packetSize) {

	if (packetSize <= sizeof(uint8_t) + sizeof(int)) {
        Serial.println("Error: Packet too small!");
        return {};
    }

    std::vector<uint8_t> receivedBytes(packetSize);
    for (int i = 0; i < packetSize; i++) {
        receivedBytes[i] = LoRa.read();
    }

    int receivedCRC;
    std::memcpy(&receivedCRC, receivedBytes.data() + (packetSize - sizeof(int)), sizeof(int));
    size_t numFloats = (packetSize - sizeof(int)) / sizeof(float);
    std::vector<float> packetData(numFloats);
    for (size_t i = 0; i < numFloats; i++) {
        std::memcpy(&packetData[i], receivedBytes.data() + (i * sizeof(float)), sizeof(float));
    }

    return packetData;
}


void writeSerial(float Rssi, float Snr, std::vector<float>& dataArray) {
    int totalFloats = dataArray.size();
    uint8_t buffer[sizeof(float) + sizeof(float) + totalFloats * sizeof(float)];

    // memcpy(buffer, &totalFloats, sizeof(int));
    memcpy(buffer, &Rssi, sizeof(float));
    memcpy(buffer + sizeof(float), &Snr, sizeof(float));
    memcpy(buffer + sizeof(float) + sizeof(float), dataArray.data(), totalFloats * sizeof(float));

    Serial.print("%%");
    Serial.write(buffer, sizeof(float) + sizeof(float) + totalFloats * sizeof(float));
}