#ifndef SD_CARD_H
#define SD_CARD_H

#include "FS.h"
#include <Arduino.h>
include <TinyGPS++.h>

extern const bool TEST; // Deklaration der Konstante TEST

void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
void writeDebug(const String &message);
void initializeSDCard();

// Wrapper-Funktion für Serial.print
void debugPrint(const String &message);
void debugPrintln(const String &message);

String generateFileName(TinyGPSPlus &gps);

extern const char firstline[];

#endif // SD_CARD_H
