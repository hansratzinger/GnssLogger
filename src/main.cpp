/*********
 *   Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete instructions at https://RandomNerdTutorials.com/esp32-neo-6m-gps-module-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include <TinyGPS++.h>
#include "SD_card.h"
#include <EEPROM.h>
#include <esp_sleep.h>
#include <SD.h> // Einbinden der SD-Bibliothek
#include <SPI.h> // Einbinden der SPI-Bibliothek
#include <deque>
#include <WiFi.h> // Einbinden der WiFi-Bibliothek
#include "GNSS_module.h" // Einbinden der GNSS-Modul-Header-Datei

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 115200

unsigned long start = millis();
String gpstime, date, lat, lon, speed, altitude ,hdop, satellites, logging, firstline;
String gpstimeLast, dateLast, latLast, lonLast, speedLast, altitudeLast ,hdopLast, satellitesLast, loggingLast, firstlineLast;  
double distanceLast, latDifference, lonDifference;
bool isMissionMode = true;
bool isWakedUp = false;
unsigned long lastSwitchTime = start;
const unsigned long switchInterval =  300000; // 5 Minuten in Millisekunden
const double circleAroundPosition = 5.0; // Radius in Metern
const unsigned long sleepingTime = 2000; // 2 Sekunden in Millisekunden

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2); // Initialisierung von gpsSerial

// Deque zum Speichern der letzten 5 Positionen
std::deque<std::pair<double, double>> lastPositions;

// Funktion zur Aktivierung des Light-Sleep-Modus
void enableLightSleep() {
  debugPrintln("Light-Sleep-Modus aktiviert");
  delay(100); // Warte 100 Millisekunden
  esp_sleep_enable_timer_wakeup(sleepingTime * 1000); // 2 Sekunden in Mikrosekunden
  esp_light_sleep_start();
}

// Funktion zur Aktivierung des Deep-Sleep-Modus
void enableDeepSleep() {
  debugPrintln("Deep-Sleep-Modus aktiviert");
  delay(100); // Warte 100 Millisekunden
  esp_sleep_enable_timer_wakeup(sleepingTime * 1000); // 4 Sekunden in Mikrosekunden
  esp_deep_sleep_start();
}

void setup() {
  // Serial Monitor
  Serial.begin(115200);
  
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  debugPrintln("Serial 2 started at 9600 baud rate");

  if (!SD.begin()) {
    debugPrintln("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    debugPrintln("No SD card attached");
    return;
  }

  debugPrint("SD Card Type: ");
  if (cardType == CARD_MMC) {
    debugPrintln("MMC");
  } else if (cardType == CARD_SD) {
    debugPrintln("SDSC");
  } else if (cardType == CARD_SDHC) {
    debugPrintln("SDHC");
  } else if (cardType == CARD_UNKNOWN) {
    debugPrintln("UNKNOWN CARD");
  } else if (cardType == CARD_NONE) {
    debugPrintln("No SD card attached");
    return;
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  debugPrintln("SD Card Size: " + String(cardSize) + "MB");

  listDir(SD, "/", 0);

  // Load data from RTC memory
  loadFromRTC(gpstimeLast, dateLast, latLast, lonLast, isMissionMode);

  // Aktivieren des Modem-Sleep-Modus
  enableModemSleep();
}

void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isUpdated()) {
    lat = String(gps.location.lat(), 6);      
    lon = String(gps.location.lng(), 6);

    // Bestimme die Himmelsrichtung
    String directionLat = getDirectionLat(gps.location.lat());
    String directionLng = getDirectionLng(gps.location.lng());

    char timeBuffer[10];
    sprintf(timeBuffer, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    gpstime = String(timeBuffer);
    date = String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day());
    hdop = String(gps.hdop.hdop());
    satellites = String(gps.satellites.value());
    speed = String(gps.speed.knots());
    altitude = String(gps.altitude.meters());
    firstline = "Date;UTC;Lat;N/S;Lon;E/W;knots;Alt/m;HDOP;Satellites;Fix-distance/m;LatDiff;LonDiff\n";
    logging = date + ";" + gpstime + ";" + lat + ";" + directionLat + ";" + lon + ";" +  directionLng + ";" + speed + ";" + altitude + ";" + hdop + ";" + satellites;

    // Berechne die Entfernung zum letzten Punkt
    if (latLast != "" && lonLast != "") {
      distanceLast = calculateDistance(lat.toDouble(), lon.toDouble(), latLast.toDouble(), lonLast.toDouble());
      logging += ";" + String(distanceLast);
      latDifference = calculateDifference(lat.toDouble(), latLast.toDouble());
      lonDifference = calculateDifference(lon.toDouble(), lonLast.toDouble());  
      logging += ";" + String(latDifference, 6) + ";" + String(lonDifference, 6);
    }
    logging += "\n";

    // Ersetze Punkte durch Kommas in den Zahlen
    logging.replace('.', ',');

    // Wechsel zwischen Station- und Mission-Modus
    if (isMissionMode) {
      // Schreibe nur im Mission-Modus auf die SD-Karte
      if (date != "2000/0/0") {
        // SD card    
        // Generiere den Dateinamen basierend auf dem aktuellen Datum
        String fileName = generateFileName(gps);

        // Überprüfe, ob die Datei bereits existiert
        if (!SD.exists(fileName.c_str())) {
          // Datei existiert nicht, erstelle die Datei und schreibe die erste Zeile
          writeFile(SD, fileName.c_str(), firstline.c_str());
        }
     
        // Schreibe die Daten in die Datei
        appendFile(SD, fileName.c_str(), logging.c_str());

        // Serial monitor          
        debugPrint("Date: ");
        debugPrintln(date);
        debugPrint("Time: ");
        debugPrintln(gpstime);
        debugPrint("LAT: ");
        debugPrint(lat);
        debugPrintln(" " + directionLat);
        debugPrint("LON: "); 
        debugPrint(lon);
        debugPrintln(" " + directionLng);
        debugPrint("SPEED (knots) = "); 
        debugPrintln(speed); 
        debugPrint("Alt = "); 
        debugPrintln(altitude); 
        debugPrint("HDOP = "); 
        debugPrintln(hdop); 
        debugPrint("Satellites = "); 
        debugPrintln(satellites); 
        debugPrint("Distance (m) = ");
        debugPrintln(String(distanceLast)); // Konvertieren von double zu String
        debugPrintln("----------------------------");

        // Save the last values
        gpstimeLast = gpstime;
        dateLast = date;
        latLast = lat;
        lonLast = lon;
      }

      if (millis() - lastSwitchTime >= switchInterval) {
        bool withinRange = false;
        for (const auto& pos : lastPositions) {
          if (isWithinRange(lat.toDouble(), lon.toDouble(), pos.first, pos.second, circleAroundPosition)) {
            withinRange = true;
            break;
          }
        }
        if (withinRange) {
          isMissionModeRTC = false;
          lastSwitchTime = millis();
          debugPrintln("Switched to Station Mode");
        }
      }

      // Aktivieren des Light-Sleep-Modus im Mission-Modus
      enableLightSleep();
    } else {
      bool withinRange = false;
      for (const auto& pos : lastPositions) {
        if (isWithinRange(lat.toDouble(), lon.toDouble(), pos.first, pos.second, circleAroundPosition)) {
          withinRange = true;
          break;
        }
      }
      if (!withinRange) {
        isMissionModeRTC = true;
        lastSwitchTime = millis();
        debugPrintln("Switched to Mission Mode");
        enableALPMode(); // ALP-Modus aktivieren
      }

      // Aktivieren des Deep-Sleep-Modus im Station-Modus
      enableDeepSleep();
    }

    // Füge die aktuelle Position zur Liste der letzten 5 Positionen hinzu
    lastPositions.push_back({lat.toDouble(), lon.toDouble()});
    if (lastPositions.size() > 5) {
      lastPositions.pop_front();
    }

    // Speichern der Daten im RTC-Speicher
    saveToRTC(gpstimeLast, dateLast, latLast, lonLast, isMissionModeRTC, isWakedUpRTC=false);
  }
}

// Funktion zur Überprüfung, ob eine Position innerhalb eines bestimmten Radius liegt
bool isWithinRange(double lat1, double lon1, double lat2, double lon2, double radius) {
  double distance = calculateDistance(lat1, lon1, lat2, lon2);
  return distance <= radius;
}
