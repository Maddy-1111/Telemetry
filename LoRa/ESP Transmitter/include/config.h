// config.h
#pragma once

#include <HardwareSerial.h>

#define USB_BAUD 115200
#define UART_BAUD 9600
#define RX_PIN 16
#define TX_PIN 17
#define AUX_PIN 18
#define TIMEOUT 1000                        // (ms) for UART
#define MAX_DATA_LEN 400                 // in bytes
#define HEADER {0xDE, 0xAD, 0xBE, 0xEF}

const uint8_t header[4] = HEADER;
const int PIN_M0 = 4;
const int PIN_M1 = 5;

extern HardwareSerial E32Serial;
