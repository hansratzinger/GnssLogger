#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPS++.h>
#include <EEPROM.h>
#include "GNSS_module.h"

// Definition der RTC-Variablen
RTC_DATA_ATTR bool isMissionModeRTC;
RTC_DATA_ATTR bool isWakedUpRTC;
RTC_DATA_ATTR String gpstimeLastRTC;
RTC_DATA_ATTR String dateLastRTC;
RTC_DATA_ATTR String latLastRTC;
RTC_DATA_ATTR String lonLastRTC;

// Funktion zur Aktivierung des ALP-Modus
void enableALPMode() {
  // Befehl zur Aktivierung des ALP-Modus
  const char* enableALPCommand = "$PMTK225,8*23\r\n";
  gpsSerial.print(enableALPCommand);
  Serial.println("ALP-Modus aktiviert");
}

// Funktion zur Deaktivierung des ALP-Modus
void disableALPMode() {
  // Befehl zur Deaktivierung des ALP-Modus
  const char* disableALPCommand = "$PMTK225,0*2B\r\n";
  gpsSerial.print(disableALPCommand);
  Serial.println("ALP-Modus deaktiviert");
}

void saveToRTC(const String &gpstimeLast, const String &dateLast, const String &latLast, const String &lonLast, bool isMissionMode, bool isWakedUp) {
  gpstimeLastRTC = gpstimeLast;
  dateLastRTC = dateLast;
  latLastRTC = latLast;
  lonLastRTC = lonLast;
  isMissionModeRTC = isMissionMode;
}

void loadFromRTC(String &gpstimeLast, String &dateLast, String &latLast, String &lonLast, bool &isMissionMode) {
  gpstimeLast = gpstimeLastRTC;
  dateLast = dateLastRTC;
  latLast = latLastRTC;
  lonLast = lonLastRTC;
  isMissionMode = isMissionModeRTC;
}
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000; // Radius der Erde in Metern
  double latRad1 = radians(lat1);
  double latRad2 = radians(lat2);
  double deltaLat = radians(lat2 - lat1);
  double deltaLon = radians(lon2 - lon1);

  double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
             cos(latRad1) * cos(latRad2) *
             sin(deltaLon / 2) * sin(deltaLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c; // Entfernung in Metern
}

double calculateDifference(double firstData,  double secoundData) {
  return firstData - secoundData;
}

bool isWithinRange(double lat1, double lon1, double lat2, double lon2, double radius) {
  return calculateDistance(lat1, lon1, lat2, lon2) <= radius;
}

// Funktion zur Aktivierung des Modem-Sleep-Modus
void enableModemSleep() {
  WiFi.mode(WIFI_OFF); // Deaktivieren des WiFi-Moduls
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // Aktivieren des Modem-Sleep-Modus
  Serial.println("Modem-Sleep-Modus aktiviert");
  }