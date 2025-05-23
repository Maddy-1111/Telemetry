// config.h
#pragma once

#include <HardwareSerial.h>

#define USB_BAUD 115200
#define UART_BAUD 9600
#define RX_PIN 16
#define TX_PIN 17

extern HardwareSerial E32Serial;

const int PIN_M0 = 4;
const int PIN_M1 = 5;