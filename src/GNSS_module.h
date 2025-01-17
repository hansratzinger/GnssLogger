#ifndef GNSS_MODULE_H
#define GNSS_MODULE_H

#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h> // Einbinden der WiFi-Bibliothek
#include <esp_wifi.h>

// Deklaration von gpsSerial
extern HardwareSerial gpsSerial;

const bool TEST = true; // Hinzufügen der TEST-Konstanten
String generateFileName(TinyGPSPlus &gps);
String getDirectionLat(double latitude);
String getDirectionLng(double longitude);
String convertToDMM(double decimalDegrees);

// Deklaration der RTC-Variablen
extern RTC_DATA_ATTR bool isMissionModeRTC;
extern RTC_DATA_ATTR bool isWakedUpRTC;

double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double calculateDifference(double firstData, double secondData);
bool isWithinRange(double lat1, double lon1, double lat2, double lon2, double radius);
void writeCreationAndModificationDate(fs::FS &fs, const char *path, TinyGPSPlus &gps);
void saveToRTC(const String &gpstimeLast, const String &dateLast, const String &latLast, const String &lonLast, bool isMissionMode, bool isWakedUp);
void loadFromRTC(String &gpstimeLast, String &dateLast, String &latLast, String &lonLast, bool &isMissionMode);
void enableALPMode();
void disableALPMode();
void enableModemSleep();

// Wrapper-Funktion für Serial.print
void debugPrint(const String &message);
void debugPrintln(const String &message);

#endif // GNSS_MODULE_H
