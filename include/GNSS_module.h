#ifndef GNSS_MODULE_H
#define GNSS_MODULE_H

#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <FS.h>

// GPS Serial Interface
extern HardwareSerial gpsSerial;

/**
 * @brief Generiert einen Dateinamen basierend auf GPS-Daten
 * @param gps TinyGPS++ Objekt mit aktuellen GPS-Daten
 * @return String mit generiertem Dateinamen
 */
String generateFileName(TinyGPSPlus &gps);

/**
 * @brief Ermittelt die Richtung (N/S) für Breitengrad
 * @param latitude Breitengrad als Dezimalzahl
 * @return String mit "N" oder "S"
 */
String getDirectionLat(double latitude);

/**
 * @brief Ermittelt die Richtung (E/W) für Längengrad
 * @param longitude Längengrad als Dezimalzahl
 * @return String mit "E" oder "W"
 */
String getDirectionLng(double longitude);

/**
 * @brief Konvertiert Dezimalgrad in DMM Format
 * @param decimalDegrees Grad in Dezimalschreibweise
 * @return String im DMM Format
 */
String convertToDMM(double decimalDegrees);

/**
 * @brief Berechnet Distanz zwischen zwei Koordinaten
 * @return Distanz in Metern
 */
double calculateDistance(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief Berechnet Differenz zwischen zwei Werten
 * @return Absolute Differenz
 */
double calculateDifference(double firstData, double secondData);

/**
 * @brief Schreibt Erstellungs- und Änderungsdatum in Datei
 */
void writeCreationAndModificationDate(fs::FS &fs, const char *path, TinyGPSPlus &gps);

/**
 * @brief Debug Ausgabe Funktionen
 */
void debugPrint(const String &message);
void debugPrintln(const String &message);

/**
 * @brief Berechnet die Zeitdifferenz zwischen zwei GPS-Zeitstempeln
 * @param gpstime Aktueller Zeitstempel
 * @param gpstimeLast Letzter Zeitstempel
 * @return Zeitdifferenz in Sekunden
 */
unsigned long getTimeDifference(const char* gpstime, const char* gpstimeLast);

#endif // GNSS_MODULE_H