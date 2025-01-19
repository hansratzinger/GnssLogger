#include <esp_sleep.h>
#include <SD.h> // Einbinden der SD-Bibliothek
#include <SPI.h> // Einbinden der SPI-Bibliothek
#include <deque>
#include <WiFi.h> // Einbinden der WiFi-Bibliothek
#include "GNSS_module.h" // Einbinden der GNSS-Modul-Header-Datei
#include "SD_card.h" // Einbinden der SD-Karten-Header-Datei
#include "Sleep.h" // Einbinden der Sleep-Header-Datei
#include <esp_wifi.h>
#include <Morse_LED.h> // Einbinden der Morse-LED-Header-Datei

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 115200

// Define the GPIO pins for the LEDs
const int RED_LED_PIN = 25; // station mode  // Definition des GPIO-Pins für die rote LED
const int GREEN_LED_PIN = 26; // mission mode // Definition des GPIO-Pins für die grüne LED

unsigned long start = millis();
String gpstime, date, lat, directionLat, lon, directionLng, speed, altitude, hdop, satellites, logging;
String gpstimeLast, dateLast, latLast, lonLast, speedLast, altitudeLast, hdopLast, satellitesLast, loggingLast, firstlineLast;  
double distanceLast, latDifference, lonDifference;
bool isMissionMode = true;
bool isWakedUp = false;
bool isWakedUpFromLightSleep = false;
bool isWakedUpFromDeepSleep = false;
bool isWakedUpFromDeepSleepRTC = false;
 
RTC_DATA_ATTR bool isMissionModeRTC = true;
RTC_DATA_ATTR std::deque<std::pair<double, double>> stationPositionsRTC;
RTC_DATA_ATTR bool isWakedUpRTC; 

const bool TEST = true; // Definition der Konstante TEST

unsigned long lastSwitchTime = start;
const unsigned long switchInterval = 300000; // 5 Minuten in Millisekunden
const double circleAroundPosition = 1.0; // Radius in Metern
const unsigned long sleepingTimeLightSleep = 2; // 2 Sekunden
const unsigned long sleepingTimeDeepSleep = 5; // 5 Sekunden
const double hdopTreshold = 0.7; // HDOP-Schwellenwert
const unsigned long timeToLastPositionTreshold = 15; // Zeitdifferenz-Schwellenwert in Sekunden
const unsigned long delayTime = 500; // LED blink delay time

const String firstline = "Date;UTC;Lat;N/S;Lon;E/W;knots;Alt/m;HDOP;Satellites;Fix-distance/m;LatDiff;LonDiff\n";

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2); // Initialisierung von gpsSerial

// Deque zum Speichern der letzten 5 Positionen
std::deque<std::pair<double, double>> stationPositions;

// Funktion zur Berechnung der Zeitdifferenz zwischen gpstime und gpstimeLast
unsigned long getTimeDifference(const String &gpstime, const String &gpstimeLast) {
  int hour1, minute1, second1;
  int hour2, minute2, second2;

  sscanf(gpstime.c_str(), "%d:%d:%d", &hour1, &minute1, &second1);
  sscanf(gpstimeLast.c_str(), "%d:%d:%d", &hour2, &minute2, &second2);

  unsigned long time1 = hour1 * 3600 + minute1 * 60 + second1;
  unsigned long time2 = hour2 * 3600 + minute2 * 60 + second2;

  if (time1 >= time2) {
    return time1 - time2;
  } else {
    return time2 - time1;
  }
}

// Funktion zur Überprüfung, ob eine Position innerhalb eines bestimmten Radius liegt
bool isWithinRange(double lat1, double lon1, double lat2, double lon2, double radius) {
  double distance = calculateDistance(lat1, lon1, lat2, lon2);
  return distance <= radius;
}

void setup() {
  // Serial Monitor
  Serial.begin(115200);
  
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  debugPrintln("Serial 2 started at " + String(GPS_BAUD) + " baud rate");

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
  loadStationPositionsFromRTC(stationPositions);

  // Initialisiere die LED-Pins
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  // Aktivieren des Modem-Sleep-Modus
  enableModemSleep();
}

void loop() {
  // Read data from the GPS module
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if ((gps.location.isUpdated()) && (gps.hdop.hdop() < hdopTreshold) && (gps.date.year()) != 2000 && (gps.date.month()) != 0 && (gps.date.day()) != 0  && (gps.time.hour()) != 0 && (gps.time.minute()) != 0 && (gps.time.second()) != 0 ) {
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

    // Berechne die Zeitdifferenz zwischen gpstime und gpstimeLast
    unsigned long timeDifference = getTimeDifference(gpstime, gpstimeLast);
    debugPrintln("Time difference: " + String(timeDifference) + " seconds");

    if (timeDifference > timeToLastPositionTreshold) {
      // Füge die aktuelle Position zur Liste 5 Station-Positionen hinzu
      // neue Station-Positionen werden am Anfang der Liste hinzugefügt
      stationPositions.clear();
      stationPositions.push_back(std::make_pair(lat.toDouble(), lon.toDouble()));
      while (stationPositions.size() < 5) {
        // Warte auf die nächste gültige Position
        while (gpsSerial.available() > 0) {
          gps.encode(gpsSerial.read());
        }
        if ((gps.location.isUpdated()) && (gps.hdop.hdop() < hdopTreshold)) {
          double newLat = gps.location.lat();
          double newLon = gps.location.lng();
          if (isWithinRange(newLat, newLon, stationPositions.back().first, stationPositions.back().second, circleAroundPosition)) {
            stationPositions.push_back(std::make_pair(newLat, newLon));
          }
        }
      }
      if (stationPositions.size() == 5) {
        isMissionMode = false;
        debugPrintln("Switched to Station Mode");

        // Schreibe die Station-Positionen auf die SD-Karte
        for (const auto& pos : stationPositions) {
          String stationLogging = date + ";" + gpstime + ";" + String(pos.first, 6) + ";" + directionLat + ";" + String(pos.second, 6) + ";" + directionLng + ";" + speed + ";" + altitude + ";" + hdop + ";" + satellites + ";station-mode\n";
          String fileName = generateFileName(gps);
          if (!SD.exists(fileName.c_str())) {
            // Datei existiert nicht, erstelle die Datei und schreibe die erste Zeile
            writeFile(SD, fileName.c_str(), firstline.c_str());
          }
          appendFile(SD, fileName.c_str(), stationLogging.c_str());
        }
      }
    }

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
        for (const auto& pos : stationPositions) {
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
      enableLightSleep(sleepingTimeLightSleep);

      // Schalte die LEDs entsprechend dem Modus
      if (TEST) {
        blinkMorseCode("G", GREEN_LED_PIN, 1); // Grüne LED blinkt im Mission-Modus
      }
    } else {
      // Überprüfen, ob die aktuelle Position außerhalb des doppelten Radius der stationPositions liegt
      bool outsideDoubleRadius = true;
      for (const auto& pos : stationPositions) {
        if (isWithinRange(lat.toDouble(), lon.toDouble(), pos.first, pos.second, 2 * circleAroundPosition)) {
          outsideDoubleRadius = false;
          break;
        }
      }
      if (outsideDoubleRadius) {
        isMissionMode = true;
        stationPositions.clear();
        debugPrintln("Switched to Mission Mode due to position outside double radius");
      }

      // Aktivieren des Deep-Sleep-Modus im Station-Modus
      if (stationPositions.size() >= 5) {
        saveStationPositionsToRTC(stationPositions);
        for (const auto& pos : stationPositions) {
          String stationLogging = date + ";" + gpstime + ";" + String(pos.first, 6) + ";" + directionLat + ";" + String(pos.second, 6) + ";" + directionLng + ";" + speed + ";" + altitude + ";" + hdop + ";" + satellites + ";station-mode\n";
          String fileName = generateFileName(gps);
          if (!SD.exists(fileName.c_str())) {
            // Datei existiert nicht, erstelle die Datei und schreibe die erste Zeile
            writeFile(SD, fileName.c_str(), firstline.c_str());
          }
          appendFile(SD, fileName.c_str(), stationLogging.c_str());
        }
        enableDeepSleep(sleepingTimeDeepSleep);
      }

      // Schalte die LEDs entsprechend dem Modus
      if (TEST) {
        blinkMorseCode("R", RED_LED_PIN, 1); // Rote LED blinkt im Station-Modus
      }
    }

    // Füge die aktuelle Position zur Liste der letzten 5 Positionen hinzu
    stationPositions.push_back({lat.toDouble(), lon.toDouble()});
    if (stationPositions.size() > 5) {
      stationPositions.pop_front();
    }

    // Speichern der Daten im RTC-Speicher
    saveToRTC(gpstimeLast, dateLast, latLast, lonLast, isMissionModeRTC, isWakedUpRTC=false);
  }
}
