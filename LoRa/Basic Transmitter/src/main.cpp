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

Serial.println("Initializing LoRa Transmitter...");
    SPI.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_FREQ)) {
Serial.println("LoRa initialization failed!");
        while (1);
    }

    LoRa.setSpreadingFactor(7);  // SF (default)
    LoRa.setSignalBandwidth(125E3);  // BW (default)
    LoRa.setCodingRate4(6);  // CR (default)

Serial.println("LoRa Transmitter Ready");
}

void loop() {

    for (int i = 0; i < 100; i++) {
Serial.println("Sending: " + String(i));

        LoRa.beginPacket();
        LoRa.print("Value: ");
        LoRa.print(i);
        LoRa.print(", Temp: ");
        LoRa.print(25.6);
        LoRa.endPacket();  // Sends the entire packet

        delay(100);  // Small delay for stability
    }
}
