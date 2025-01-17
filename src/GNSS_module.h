#ifndef GNSS_MODULE_H
#define GNSS_MODULE_H

#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Deklaration von gpsSerial
extern HardwareSerial gpsSerial;

String generateFileName(TinyGPSPlus &gps);
String getDirectionLat(double latitude);
String getDirectionLng(double longitude);
String convertToDMM(double decimalDegrees);
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double calculateDifference(double firstData, double secondData);
bool isWithinRange(double lat1, double lon1, double lat2, double lon2, double radius);
void writeCreationAndModificationDate(fs::FS &fs, const char *path, TinyGPSPlus &gps);
void saveToRTC(const String &gpstimeLast, const String &dateLast, const String &latLast, const String &lonLast, bool isMissionMode, bool isWakedUp);
void loadFromRTC(String &gpstimeLast, String &dateLast, String &latLast, String &lonLast, bool &isMissionMode);
void enableALPMode();
void disableALPMode();

#endif // GNSS_MODULE_H
