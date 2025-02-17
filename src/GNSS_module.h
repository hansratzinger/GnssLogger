#define GNSS_MODULE_H
#pragma once
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h> // Einbinden der WiFi-Bibliothek
#include <esp_wifi.h>
#include <FS.h>

// Deklaration von gpsSerial
extern HardwareSerial gpsSerial;
extern const unsigned long deepSleepTime; // Second 
extern const unsigned long lightSleepTime; // Second

extern RTC_DATA_ATTR bool isMissionModeRTC;
extern RTC_DATA_ATTR bool isWakedUpRTC;

String generateFileName(TinyGPSPlus &gps);
String getDirectionLat(double latitude);
String getDirectionLng(double longitude);
String convertToDMM(double decimalDegrees);

// Deklaration der RTC-Variablen
extern RTC_DATA_ATTR bool isMissionModeRTC;
extern RTC_DATA_ATTR bool isWakedUpRTC;

double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double calculateDifference(double firstData, double secondData);
void writeCreationAndModificationDate(fs::FS &fs, const char *path, TinyGPSPlus &gps);
void saveToRTC(const String &gpstimeLast, const String &dateLast, const String &latLast, const String &lonLast, bool isMissionMode);
void loadFromRTC(String &gpstimeLast, String &dateLast, String &latLast, String &lonLast, bool &isMissionMode);


// Wrapper-Funktion für Serial.print
void debugPrint(const String &message);
void debugPrintln(const String &message);

// Funktion zur Berechnung der Zeitdifferenz zwischen gpstime und gpstimeLast
unsigned long getTimeDifference(const String &gpstime, const String &gpstimeLast);
