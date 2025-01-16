#ifndef GNSS_MODULE_H
#define GNSS_MODULE_H

#include <HardwareSerial.h>

// Deklaration von gpsSerial
extern HardwareSerial gpsSerial;

void enableALPMode();
void disableALPMode();
const char* enableALPCommand;
const char* disableALPCommand;

#endif // GNSS_MODULE_H
