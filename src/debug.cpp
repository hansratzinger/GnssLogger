#include "debug.h"

void debugPrint(const String &message) {
  Serial.print(message);
  File file = SD.open("/debug.txt", FILE_APPEND);
  if (file) {
    file.print(message);
    file.close();
  }
}

void debugPrintln(const String &message) {
  Serial.println(message);
  File file = SD.open("/debug.txt", FILE_APPEND);
  if (file) {
    file.println(message);
    file.close();
  }
}

void debugPrint(const char* message) {
  Serial.print(message);
  File file = SD.open("/debug.txt", FILE_APPEND);
  if (file) {
    file.print(message);
    file.close();
  }
}

void debugPrintln(const char* message) {
  Serial.println(message);
  File file = SD.open("/debug.txt", FILE_APPEND);
  if (file) {
    file.println(message);
    file.close();
  }
}

void debugPrint(bool value) {
  Serial.print(value);
  File file = SD.open("/debug.txt", FILE_APPEND);
  if (file) {
    file.print(value);
    file.close();
  }
}

void debugPrintln(bool value) {
  Serial.println(value);
  File file = SD.open("/debug.txt", FILE_APPEND);
  if (file) {
    file.println(value);
    file.close();
  }
}