#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <vector>

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

std::vector<std::vector<float>> chunkVector(const std::vector<float>& vec, int chunkSize);
bool checkSerial(std::vector<float>& dataArray);
int crc_generate(const std::vector<float>& data);


void setup() {
  Serial.begin(115200);
  while (!Serial);
  // Serial.println("Initializing LoRa Transmitter...");

  SPI.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(LORA_FREQ)) {
    // Serial.println("LoRa initialization failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  // Serial.println("LoRa Transmitter Ready");
}

void loop() {

  if (checkSerial(receivedData)) {
    // Process the data here
    Serial.printf("First 3: %.2f, %.2f, %.2f ... Last 2: %.2f, %.2f\n", 
                  receivedData[0], receivedData[1], receivedData[2], 
                  receivedData[receivedData.size() - 2], receivedData[receivedData.size() - 1]);

    std::vector<float> dataCopy = receivedData;
    int chunkSize = 50;
    auto chunks = chunkVector(dataCopy, chunkSize);

    // For each chunk, calculate its CRC and send via LoRa
    for (auto &chunk : chunks) {
      int crc = crc_generate(chunk);
      String packetData = "";
      
      // Convert chunk data to a CSV string
      for (size_t i = 0; i < chunk.size(); i++) {
        packetData += String(chunk[i], 2); // 2 decimal places
        if (i < chunk.size() - 1) {
          packetData += ",";
        }
      }
      // Append the CRC value in hexadecimal
      packetData += ",CRC:";
      packetData += String(crc, HEX);

      // Send the packet via LoRa
      LoRa.beginPacket();
      LoRa.print(packetData);
      LoRa.endPacket();
      delay(100);  // Small delay between packets
    }
  }  

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
int crc_generate(const std::vector<float>& data) {
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


bool checkSerial(std::vector<float>& dataArray) {
    if (Serial.available()) {  
        String receivedString = Serial.readStringUntil('\n');  // Read CSV data until newline
        int index = 0;

        dataArray.clear();  // Clear the previous data

        char* token = strtok(receivedString.begin(), ",");  // Tokenize using comma
        while (token != nullptr) {
            dataArray.push_back(atof(token));  // Convert token to float and store
            token = strtok(nullptr, ",");  // Get next token
        }

        return true;  // Data successfully received and stored

    }
    return false;  // No new data received
}
