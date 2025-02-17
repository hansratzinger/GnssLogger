#pragma once// filepath: /c:/esp32/GnssLogger/src/SD_card.h
#include <pins.h>
#include <SD.h>
#include <FS.h>
#include <TinyGPS++.h>

extern const bool TEST; // Deklaration der Konstante TEST

void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);

void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
void writeDebug(const String &message);
bool initializeSDCard();
bool appendFile(fs::FS &fs, const char * path, const char * message);
bool writeFile(fs::FS &fs, const char * path, const char * message);  // Diese Zeile hinzufügen
bool listDirectory(fs::FS &fs, const char * dirname, uint8_t levels);
String generateFileName(TinyGPSPlus& gps);

bool initializeSDCard();

// Wrapper-Funktion für Serial.print
void debugPrint(const String &message);
void debugPrintln(const String &message);

char getDirectionOfLat(double latitude);
char getDirectionOfLng(double longitude);
