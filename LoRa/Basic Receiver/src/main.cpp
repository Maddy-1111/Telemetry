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


void processCommand(String input);
void setCommRate(int sf, int bw, int cr);

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
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        Serial.print("Received: ");
        while (LoRa.available()) {
            char buf = (char)LoRa.read(); 
            if (buf == '-') continue;
            Serial.print(buf);
        }
        Serial.print(" | RSSI: ");
        Serial.print(LoRa.packetRssi());  // Signal strength
        Serial.print(" dBm, SNR: ");
        Serial.println(LoRa.packetSnr()); // Signal-to-noise ratio
    }

    if (Serial.available() > 0) {
        String input = Serial.readString();
        processCommand(input);
    }
}


void processCommand(String input) {
    input.trim();  // Remove any trailing spaces or newlines

    if (input == "reset") {
        Serial.println("Initiating Software Reset...");
        delay(1000);
        ESP.restart();
    } 
    else if (input.startsWith("set ")) {
        input = input.substring(4);  // Remove "set " from the string

        int val1, val2, val3;
        int numExtracted = sscanf(input.c_str(), "%d %d %d", &val1, &val2, &val3);
        
        if (numExtracted == 3) {  // Ensure we got exactly 3 values
            Serial.printf("Received values: %d, %d, %d\n", val1, val2, val3);
            setCommRate(val1, val2, val3);
        }
    }
}

//// can change int to something else later ////
void setCommRate(int sf, int bw, int cr) {
    // LoRa.setSpreadingFactor(10);
    // LoRa.setSignalBandwidth(62.5E3);
    // LoRa.setCodingRate4(8);

    LoRa.beginPacket();
    // LoRa.write((uint8_t*)&sf, sizeof(sf));
    // LoRa.write((uint8_t*)&bw, sizeof(bw));
    // LoRa.write((uint8_t*)&cr, sizeof(cr));
    LoRa.print("This is a test");
    LoRa.endPacket();

    // LoRa.setSpreadingFactor(7);
    // LoRa.setSignalBandwidth(125E3);
    // LoRa.setCodingRate4(6);
    // LoRa.setSpreadingFactor(sf);
    // LoRa.setSignalBandwidth(bw);
    // LoRa.setCodingRate4(cr);
    // Serial.printf("Updated to SF: %d, BW: %.1f kHz, CR: 4/%d\n", sf, bw / 1E3, cr);
}