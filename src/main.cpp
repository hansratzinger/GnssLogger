#include <esp_sleep.h>
#include <deque>
#include <WiFi.h>
#include "GNSS_module.h"
#include "SD_card.h"
#include "Sleep.h"
#include <Morse_LED.h>
#include <SD.h>
#include <SPI.h>

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 115200

// Define the GPIO pins for the LEDs
const int RED_LED_PIN = 25; // station mode
const int GREEN_LED_PIN = 26; // mission mode

unsigned long lastSwitchTime, timeDifference = 0;
double positionDifference = 0.0;
String gpstime, date, lat, directionLat, lon, directionLng, speed, altitude, hdop, satellites, logging;
String gpstimeLast, dateLast, latLast, lonLast, speedLast, altitudeLast, hdopLast, satellitesLast, loggingLast, firstlineLast;  
double distanceLast, latDifference, lonDifference;
bool isMissionMode = true;
bool isWakedUp = false;
bool isWakedUpFromLightSleep = false;
bool isWakedUpFromDeepSleep = false;

RTC_DATA_ATTR std::deque<std::pair<double, double>> stationPositionsRTC;
RTC_DATA_ATTR bool isWakedUpRTC, isMissionModeRTC, isWakedUpFromDeepSleepRTC; 
// RTC_DATA_ATTR String latLastRTC, lonLastRTC, gpstimeLastRTC, dateLastRTC;
// RTC_DATA_ATTR unsigned long timeDifferenceRTC;

// Struktur für RTC-Speicher
struct RtcData {
  String gpstimeLast;
  String dateLast;
  String latLast;
  String lonLast;
  bool isMissionMode;
  unsigned long timeDifference;
};

// RTC-Speicher-Variable für die Struktur
RTC_DATA_ATTR RtcData rtcData;

const bool TEST = true; // Definition der Konstante TEST

const unsigned long switchInterval = 5000; // 5 Sekunden
const double circleAroundPosition = 15.0; // Radius in Metern
const unsigned long sleepingTimeLightSleep = 2; // 2 Sekunden
const unsigned long sleepingTimeDeepSleep = 5; // 5 Sekunden
const double hdopTreshold = 1; // HDOP-Schwellenwert
const double positionDifferenceTreshold = 5; // in meter
const unsigned long timeToLastPositionTreshold = 20; // Zeitdifferenz-Schwellenwert in Sekunden
const unsigned long delayTime = 500; // LED blink delay time

const String firstline = "Date;UTC;Lat;N/S;Lon;E/W;knots;Alt/m;HDOP;Satellites;Fix-distance/m;LatDiff;LonDiff;Distance/m\n";

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

void processAndStorePosition() {
  // Berechnung der Distanz zwischen der aktuellen und der letzten Position
  if (latLast != "" && lonLast != "") {
    distanceLast = calculateDistance(lat.toDouble(), lon.toDouble(), latLast.toDouble(), lonLast.toDouble());
    logging += ";" + String(distanceLast);
    latDifference = calculateDifference(lat.toDouble(), latLast.toDouble());
    lonDifference = calculateDifference(lon.toDouble(), lonLast.toDouble());
    logging += ";" + String(latDifference, 6) + ";" + String(lonDifference, 6);
  }


  // Berechne die Entfernung zum letzten Punkt
  if (latLast != "" && lonLast != "") {
    distanceLast = calculateDistance(lat.toDouble(), lon.toDouble(), latLast.toDouble(), lonLast.toDouble());
    logging += ";" + String(distanceLast);
    latDifference = calculateDifference(lat.toDouble(), latLast.toDouble());
    lonDifference = calculateDifference(lon.toDouble(), lonLast.toDouble());
    logging += ";" + String(latDifference, 6) + ";" + String(lonDifference, 6);
  }

  // Berechne die Differenz der Position zur vorhergehenden Position in Metern
  if (latLast != "" && lonLast != "") {
    positionDifference = calculateDistance(lat.toDouble(), lon.toDouble(), latLast.toDouble(), lonLast.toDouble());
  }
  logging += ";" + String(positionDifference, 6) + "\n";

  // Ersetze Punkte durch Kommas in den Zahlen
  logging.replace('.', ',');

  debugPrintln("new logging: " + logging);  
   
  // Berechne die Zeitdifferenz zwischen gpstime und gpstimeLast
  if (gpstimeLast != "") {
    timeDifference = getTimeDifference(gpstime, gpstimeLast);
    debugPrintln("Time difference: " + String(timeDifference) + " seconds");
    debugPrintln("Position difference: " + String(positionDifference) + " meters");
  }

  // Generiere den Dateinamen basierend auf dem aktuellen Datum
  String fileName = generateFileName(gps);

  // Überprüfe, ob die Datei bereits existiert
  if (!SD.exists(fileName.c_str())) {
    // Datei existiert nicht, erstelle die Datei und schreibe die erste Zeile
    writeFile(SD, fileName.c_str(), firstline.c_str());
  }

  // Schreibe die Daten in die Datei
  appendFile(SD, fileName.c_str(), logging.c_str());


}

void setup() {
  // Serial Monitor
  Serial.begin(115200);

  WiFi.mode(WIFI_OFF);  // Deaktivieren des WiFi-Moduls
  btStop(); 
  
  // Deaktivieren des Bluetooth-Moduls
  // Überprüfen des Wakeup-Reasons
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: 
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1: 
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER: 
      Serial.println("Wakeup caused by timer");
      isWakedUpFromDeepSleep = true;
      isWakedUpRTC = true; // Setze die RTC-Variable auf true
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: 
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP: 
      Serial.println("Wakeup caused by ULP program");
      break;
    default: 
      Serial.println("Wakeup was not caused by deep sleep");
      break;
  }
delay(1000);

  if (!isWakedUpFromDeepSleep) {
    isWakedUpRTC = false; // Setze die RTC-Variable auf false
    delay(10000); // Warte 10 Sekunden
    Serial.println("Booting up for the first time");
  } else {
    isWakedUpRTC = true; // Setze die RTC-Variable auf true
    debugPrintln("Waking up from deep sleep caused by timer");
    debugPrintln("isWakedUpRTC: " + String(isWakedUpRTC));
    debugPrintln("isMissionModeRTC: " + String(rtcData.isMissionMode));  
    debugPrintln("timeDifferenceRTC: " + rtcData.timeDifference );
    debugPrintln("LatLast: " + rtcData.latLast + ", LonLast: " + rtcData.lonLast );
    debugPrintln("Loaded data from RTC memory");
  }



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

  // Load data from RTC memory only if waking up from deep sleep
  if (isWakedUpFromDeepSleep) {
    gpstimeLast = rtcData.gpstimeLast;
    dateLast = rtcData.dateLast;
    latLast = rtcData.latLast;
    lonLast = rtcData.lonLast;
    isMissionMode = rtcData.isMissionMode;
    timeDifference = rtcData.timeDifference;
    loadStationPositionsFromRTC(stationPositions);
  }

  // Aktivieren des Modem-Sleep-Modus
  // enableModemSleep();

  // Initialisiere die LED-Pins
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
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

    // Aufrufen der Funktion zur Verarbeitung und Speicherung der Positionsdaten
    processAndStorePosition();

    // Berechne die Zeitdifferenz zwischen gpstime und gpstimeLast
    if (gpstimeLast != "") {
        timeDifference = getTimeDifference(gpstime, gpstimeLast);
        debugPrintln("Time difference: " + String(timeDifference) + " seconds");
    } 
    if ((timeDifference > timeToLastPositionTreshold) || (gpstimeLast == "")) { // Überprüfe, ob die letzte Position lang zurückliegt -> die Zeitdifferenz größer als der Schwellenwert ist
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
            debugPrintln("Added position to stationPositions: " + String(newLat, 6) + ", " + String(newLon, 6));
          } else {
            debugPrintln("Position out of range: " + String(newLat, 6) + ", " + String(newLon, 6));
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
        // Schalte die LEDs entsprechend dem Modus
        if (TEST) {
          blinkMorseCode("G", GREEN_LED_PIN, 1); // Grüne LED blinkt im Mission-Modus
        }
    
        // Aufrufen der Funktion zur Verarbeitung und Speicherung der Positionsdaten
        processAndStorePosition();
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
            appendFile(SD, fileName.c_str(), stationLogging.c_str());
        }
        
        // Save the last values
        gpstimeLast = gpstime;
        dateLast = date;
        latLast = lat;
        lonLast = lon;
        
        debugPrintln("Switched to Deep Sleep Mode");
        debugPrintln("gpstimeLast: " + gpstimeLast + ", dateLast: " + dateLast + ", latLast: " + latLast + ", lonLast: " + lonLast + ", isMissionMode: " + String(isMissionMode));
        delay(delayTime); // Wartezeit für die LED-Anzeige

        // Speichern der Daten im RTC-Speicher
        rtcData.gpstimeLast = gpstimeLast;
        rtcData.dateLast = dateLast;
        rtcData.latLast = latLast;
        rtcData.lonLast = lonLast;
        rtcData.isMissionMode = isMissionMode;
        rtcData.timeDifference = timeDifference;
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
    
    // Save the last values
    gpstimeLast = gpstime;
    dateLast = date;
    latLast = lat;
    lonLast = lon;

    // Speichern der Daten im RTC-Speicher
    rtcData.gpstimeLast = gpstimeLast;
    rtcData.dateLast = dateLast;
    rtcData.latLast = latLast;
    rtcData.lonLast = lonLast;
    rtcData.isMissionMode = isMissionMode;
    rtcData.timeDifference = timeDifference;
  }
}