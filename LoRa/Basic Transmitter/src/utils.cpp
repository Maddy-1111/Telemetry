#include <Arduino.h>
#include "config.h"


void enterSleepMode()
{
    digitalWrite(PIN_M0, HIGH);
    digitalWrite(PIN_M1, HIGH);
    delay(200);
}

void enterNormalMode()
{
    digitalWrite(PIN_M0, LOW);
    digitalWrite(PIN_M1, LOW);
    delay(200);
}

void apply915TransparentConfig()
{
    byte config[] = {0xC0, 0x00, 0x00, 0x1A, 0x35, 0x44};
    E32Serial.write(config, 6);
    Serial.println("✅ Config sent: 915 MHz, transparent, 9600 baud, FEC ON");
    delay(500);
}

void readAndPrintConfig()
{
    byte cmd[] = {0xC1, 0xC1, 0xC1};
    E32Serial.write(cmd, 3);
    delay(100);

    if (E32Serial.available() >= 6)
    {
        byte response[6];
        for (int i = 0; i < 6; ++i)
            response[i] = E32Serial.read();

        Serial.println("📡 Module Settings (Raw Hex):");
        for (int i = 0; i < 6; ++i)
        {
            Serial.print("0x");
            if (response[i] < 0x10)
                Serial.print("0");
            Serial.print(response[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        // Decode values
        byte ADDH = response[1];
        byte ADDL = response[2];
        byte SPED = response[3];
        byte CHAN = response[4];
        byte OPTION = response[5];

        Serial.println("🔍 Decoded Parameters:");
        Serial.printf("Address: 0x%02X%02X\n", ADDH, ADDL);

        const char *parityModes[4] = {"8N1", "8O1", "8E1", "8N1"};
        Serial.print("UART Parity: ");
        Serial.println(parityModes[(SPED >> 6) & 0b11]);

        const int baudRates[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
        Serial.print("UART Baudrate: ");
        Serial.println(baudRates[(SPED >> 3) & 0b111]);

        const float airRates[8] = {0.3, 1.2, 2.4, 4.8, 9.6, 19.2, 19.2, 19.2};
        Serial.print("Air data rate: ");
        Serial.print(airRates[SPED & 0b111]);
        Serial.println(" kbps");

        Serial.print("Channel: ");
        Serial.print(CHAN);
        Serial.print(" → Freq = ");
        Serial.print(862 + CHAN);
        Serial.println(" MHz");

        Serial.print("Fixed transmission: ");
        Serial.println((OPTION & 0b10000000) ? "Enabled" : "Disabled");

        const int txPowers[4] = {30, 27, 24, 21};
        Serial.print("TX Power: ");
        Serial.print(txPowers[OPTION & 0b11]);
        Serial.println(" dBm");

        Serial.print("FEC: ");
        Serial.println((OPTION & 0b00001000) ? "Enabled" : "Disabled");
    }
    else
    {
        Serial.println("❌ No response or not enough bytes received.");
    }
}