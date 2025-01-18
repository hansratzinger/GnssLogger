// main.cpp
// HR 2025-01-18 13:28 NK


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


String gpstime, date, lat, directionLat,lon, directionLon,speed, altitude ,hdop, satellites, logging, firstline;
String gpstimeLast, dateLast, latLast, lonLast, speedLast, altitudeLast ,hdopLast, satellitesLast, loggingLast, firstlineLast;  
double distanceLast, latDifference, lonDifference;
bool isMissionMode = true;
bool isWakedUp = false;
const bool TEST = true; // Definition der Debug-Konstante TEST

const unsigned long deepSleepTime =  5; // Second 
const unsigned long lightSleepTime = 2; // Second
const double circleAroundPosition = 1.0; // Radius in meter
const double hdopTreshold = 0.7; // only positons with hdop < 0.7 are valid
const double timeToLastPositionTreshold = 15; // only positons with time difference < 15 seconds are valid


// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2); // Initialisierung von gpsSerial

// Deque zum Speichern der letzten 5 Positionen
std::deque<std::pair<double, double>> stationPositions;

void setup() {
  // Serial Monitor
  Serial.begin(115200);
  
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  debugPrintln("Serial 2 started at" + String(GPS_BAUD) + " baud rate");

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
}

void loop() {
  // Read data from the GPS module
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if ((gps.location.isUpdated()) && (gps.hdop.hdop() < hdopTreshold)) {
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
    logging = date + ";" + gpstime + ";" + lat + ";" + directionLat + ";" + lon + ";" +  directionLon + ";" + speed + ";" + altitude + ";" + hdop + ";" + satellites;

    // Berechne die Zeitdifferenz zwischen gpstime und gpstimeLast
    unsigned long timeDifference = getTimeDifference(gpstime, gpstimeLast);
    debugPrintln("Time difference: " + String(timeDifference) + " seconds");

    if (timeToLastPositionTreshold <= timeDifference) {
      // Füge die aktuelle Position zur Liste 5 Station-Positionen hinzu
      // neue Station-Positionen werden am Anfang der Liste hinzugefügt
      stationPositions.push_back(std::make_pair(lat.toDouble(), lon.toDouble()));
      if (stationPositions.size() < 5) {
        stationPositions.pop_front();
      }

    } else {
        debugPrintln("Time difference to last position is too high");        
        isMissionMode = true;
        latLast = "";
        lonLast = "";
     }

    // Berechne die Entfernung zum letzten Punkt
    if (latLast != "" && lonLast != "") {
      distanceLast = calculateDistance(lat.toDouble(), lon.toDouble(), latLast.toDouble(), lonLast.toDouble());
      logging += ";" + String(distanceLast);
      latDifference = calculateDifference(lat.toDouble(), latLast.toDouble());
      lonDifference = calculateDifference(lon.toDouble(), lonLast.toDouble());  
      logging += ";" + String(latDifference, 6) + ";" + String(lonDifference, 6);
    } else {
      logging += ";;";
      distanceLast, latDifference, lonDifference = 0;
    }
    logging += "\n";

    // Ersetze Punkte durch Kommas in den Zahlen
    logging.replace('.', ',');
//--------------------------------------------------------------------------------------------------------------------------------

    bool withinRange = false;
    for (const auto& pos : stationPositions) {
      if (isWithinRange(lat.toDouble(), lon.toDouble(), pos.first, pos.second, circleAroundPosition)) {
        withinRange = true;
        break;
    
    }
    if (withinRange) {
      isMissionMode = false;
      debugPrintln("Switched to Station Mode");
      enableDeepSleep(); // Deep-Sleep-Modus aktivieren
    } else {
      isMissionMode = true;
      debugPrintln("Switched to Mission Mode");
      enableALPMode(); // ALP-Modus aktivieren
      enableLightSleep(); // Aktivieren des Light-Sleep-Modus im Mission-Modus
    }
  }

       // Füge die aktuelle Position zur Liste der letzten 5 Positionen hinzu  
    } else {
      bool withinRange = false;
      for (const auto& pos : stationPositions) {
        if (isWithinRange(lat.toDouble(), lon.toDouble(), pos.first, pos.second, circleAroundPosition)) {
          withinRange = true;
          break;
        }
      }
      if (!withinRange) {
        isMissionMode = true;
        lastSwitchTime = millis();
        debugPrintln("Switched to Mission Mode");
        enableALPMode(); // ALP-Modus aktivieren
              }
    }



//--------------------------------------------------------
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
        debugPrintln(" " + directionLon);
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
    }
  }
