#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <SD.h>

void debugPrint(const String &message);
void debugPrintln(const String &message);
void debugPrint(const char* message);
void debugPrintln(const char* message);
void debugPrint(bool value);
void debugPrintln(bool value);

#endif // DEBUG_H
