#ifndef SD_CARD_H
#define SD_CARD_H
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "SD_card.h"
#include <TinyGPS++.h>
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
String generateFileName(TinyGPSPlus &gps);
String getDirectionLat(double latitude);
String getDirectionLng(double longitude);

#endif // SD_CARD_H  
