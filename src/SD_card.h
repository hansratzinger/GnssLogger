// filepath: /c:/esp32/GnssLogger/src/SD_card.h
#ifndef SD_CARD_H
#define SD_CARD_H

#include <FS.h>
#include <Arduino.h>
// Define the SPI pins for the SD card
// #define SD_CS_PIN 5 // GPIO5 für CS (Chip Select) ESP32-WROOM-32D
// GPIO5 für CS (Chip Select) SIM7000G ESP32-WROVER-32D
#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS_PIN           13

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
bool initializeSDCard();

// Wrapper-Funktion für Serial.print
void debugPrint(const String &message);
void debugPrintln(const String &message);

char getDirectionOfLat(double latitude);
char getDirectionOfLng(double longitude);

#endif // SD_CARD_H
//--------------------------------------------------------------------------
