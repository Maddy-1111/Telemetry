// utils.h
#ifndef UTILS_H
#define UTILS_H

void flushSerial(HardwareSerial &serialPort);
bool checkHeader(const uint8_t *buf);
void enterSleepMode();
void enterNormalMode();
void apply915TransparentConfig();
void readAndPrintConfig();

#endif