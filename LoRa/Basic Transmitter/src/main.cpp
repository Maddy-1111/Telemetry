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


void getCommRate();


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
    // LoRa.setSpreadingFactor(10);
    // LoRa.setSignalBandwidth(62.5E3);
    // LoRa.setCodingRate4(8);

Serial.println("LoRa Transmitter Ready");
}

int count = 0;
void loop() {
    
    getCommRate();
    
    Serial.println("Sending: " + String(count));

    LoRa.beginPacket();
    LoRa.print("Value: ");
    LoRa.print(count);
    LoRa.print(", Temp: ");
    LoRa.print(25.6);
    LoRa.print("--------------------------------------------------");
    LoRa.print("--------------------------------------------------");
    LoRa.print("--------------------------------------------------");
    LoRa.print("--------------------------------------------------");
    LoRa.endPacket();  // Sends the entire packet

    count += 1;
    if(count == 100){
        count = 0;
    }

    delay(100);  // Small delay for stability
}


void getCommRate() {
    unsigned long startTime = millis();

    // LoRa.setSpreadingFactor(10);
    // LoRa.setSignalBandwidth(62.5E3);
    // LoRa.setCodingRate4(8);

    for(int i=0; i<500; i++) {
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            Serial.print("Received: ");
            while (LoRa.available()) {
                Serial.print((char)LoRa.read());
            }
            Serial.println();
        }
    }

    // LoRa.setSpreadingFactor(7);
    // LoRa.setSignalBandwidth(125E3);
    // LoRa.setCodingRate4(6);

    unsigned long endTime = millis();
    Serial.println("Execution Time: " + String(endTime - startTime) + " ms");
}
