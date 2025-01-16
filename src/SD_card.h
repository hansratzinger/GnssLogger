#ifndef SD_CARD_H
#define SD_CARD_H

#include "FS.h"
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
String convertToDMM(double decimalDegrees);
void writeCreationAndModificationDate(fs::FS &fs, const char *path, TinyGPSPlus &gps);
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double calculateDifference(double firstData, double secondData);
bool isWithinRange(double lat1, double lon1, double lat2, double lon2, double radius);
void saveToRTC(const String &gpstimeLast, const String &dateLast, const String &latLast, const String &lonLast, bool isMissionMode);
void loadFromRTC(String &gpstimeLast, String &dateLast, String &latLast, String &lonLast, bool &isMissionMode);

#endif // SD_CARD_H
