#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPS++.h>
#include <EEPROM.h>
#include "GNSS_module.h"
#include <WiFi.h>
#include <esp_wifi.h>

const bool TEST = true; // Definition der Konstante TEST

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
