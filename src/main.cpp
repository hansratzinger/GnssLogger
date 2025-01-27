// ----------------------------------------------------------------------------------------------
// GNSSLogger  
// This project uses the ESP32 Dev Module board and a GNSS module to log GPS data to an SD card.
// The GNSS module is connected to the ESP32 via UART.
// The ESP32 is put into deep sleep mode when the GNSS module is not in use.
// The ESP32 wakes up from deep sleep mode every 5 seconds to check the GNSS module for new data.
// If the GNSS module has new data, the ESP32 processes the data and logs it to an SD card.
// The ESP32 switches between mission mode and station mode based on the distance between the current position and the last position.
// In station mode, the ESP32 logs the current position to the SD card and goes into deep sleep mode.
// In mission mode, the ESP32 checks if the current position is within a certain radius of the station positions.
// If the current position is within the radius, the ESP32 switches to station mode.
// If the current position is outside the radius, the ESP32 logs the current position to the SD card and goes into deep sleep mode.
// The ESP32 uses the RTC memory to store the last position, the last time, and the mode (mission or station).
// -  the SD card to log the GPS data in CSV format.
// -  the built-in LED pins to indicate the mode (mission or station) with red and green LEDs.
// -  the Morse_LED library to blink the LEDs in Morse code.
// -  the GNSS_module library to process the GPS data.
// -  the SD_card library to interact with the SD card.
// -  the Sleep library to manage the sleep modes.
// -  the TinyGPS++ library to parse the GPS data.
// -  the WiFi library to turn off WiFi and Bluetooth.
// -  the SPI library to communicate with the SD card.
// -  the FS library to interact with the file system.
// -  the Arduino.h library for general Arduino functions.
// -  the HardwareSerial library to communicate with the GNSS module.
// -  the esp_sleep library to manage the sleep modes.
// -  the deque library to store the station positions.
// -  the stdio.h library for standard input and output functions.
// -  the string.h library for string manipulation functions.
// -  the time.h library for time-related functions.
// Hans Ratzinger 2025-01-26
// https://github.com/hansratzinger/GnssLogger
// release 1.0.1
// ----------------------------------------------------------------------------------------------

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

const String BRANCH="release 1.0.1"; // Branch name

// Deklaration von Variablen

unsigned long lastSwitchTime = 0, timeDifference = 0;
double positionDifference = 0.0;
char gpstime[10] = "", date[11] = "", lat[15] = "", directionLat[2] = "", lon[15] = "", directionLng[2] = "", speed[10] = "", altitude[10] = "", hdop[10] = "", satellites[10] = "", logging[100] = "";
char gpstimeLast[10] = "", dateLast[11] = "", latLast[15] = "", lonLast[15] = "", speedLast[10] = "", altitudeLast[10] = "", hdopLast[10] = "", satellitesLast[10] = "", loggingLast[100] = "", firstlineLast[100] = "";  
double distanceLast = 0.0, latDifference = 0.0, lonDifference = 0.0;
bool isMissionMode = true;
bool isWakedUpFromLightSleep = false;
bool isWakedUpFromDeepSleep = false;

double newLat = 0.0, newLon = 0.0, lastLat = 0.0, lastLon = 0.0;

RTC_DATA_ATTR std::deque<std::pair<double, double>> stationPositionsRTC;
// Struktur für RTC-Speicher
struct RtcData {
  char gpstimeLast[10];
  char dateLast[11];
  char latLast[15];
  char lonLast[15];
  bool isMissionMode;
  unsigned long timeDifference;
};

// RTC-Speicher-Variable für die Struktur
RTC_DATA_ATTR RtcData rtcData;

const bool TEST = true; // Definition der Konstante TEST

const unsigned long switchInterval = 500; // 0,5 Sekunden
double circleAroundPosition = 0; // Radius in Metern
const double circleAroundPositionInMissionMode = 0.8; // Radius-Schwellenwert in Metern
const double circleAroundPositionInStationMode = 3; // Radius-Schwellenwert in Metern
const unsigned long sleepingTimeLightSleep = 1; // 2 Sekunden
const unsigned long sleepingTimeDeepSleep = 5; // 5 Sekunden
const double hdopTreshold = 1; // HDOP-Schwellenwert

const unsigned long timeToLastPositionTreshold = 60; // Zeitdifferenz-Schwellenwert in Sekunden
const unsigned long delayTime = 500; // LED blink delay time
const unsigned long switchTime = 500; // Zeitdifferenz-Schwellenwert in Sekunden
const char firstline[] = "Date;UTC;Lat;N/S;Lon;E/W;knots;Alt/m;HDOP;Satellites;LatDiff;LonDiff;Distance/m;Mission\n";

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2); // Initialisierung von gpsSerial

// Deque zum Speichern der 10 Positionen des Station-Modus-Speichers
std::deque<std::pair<double, double>> stationPositions;

// Funktion zur Berechnung der Zeitdifferenz zwischen gpstime und gpstimeLast
unsigned long getTimeDifference(const char *gpstime, const char *gpstimeLast) {
  int hour1, minute1, second1;
  int hour2, minute2, second2;

  sscanf(gpstime, "%d:%d:%d", &hour1, &minute1, &second1);
  sscanf(gpstimeLast, "%d:%d:%d", &hour2, &minute2, &second2);

  unsigned long time1 = hour1 * 3600 + minute1 * 60 + second1;
  unsigned long time2 = hour2 * 3600 + minute2 * 60 + second2;

  if (time1 >= time2) {
    return time1 - time2;
  } else {
    return time2 - time1;
  }
}

double setCircleAroundPosition() {
  if (isMissionMode) {
    return circleAroundPositionInMissionMode;
  } else {
    return circleAroundPositionInStationMode;
  }
}

// Funktion zur Überprüfung, ob eine Position innerhalb eines bestimmten Radius liegt
bool isWithinRange(double lat1, double lon1, double lat2, double lon2, double radius) {
  double distance = calculateDistance(lat1, lon1, lat2, lon2);
  debugPrint("Distance: " + String(distance) + " meters");
  debugPrint(" Radius: " + String(radius) + " meters");
  debugPrint("Lat: " + String(lat1, 6));
  debugPrint("Lon: " + String(lat1, 6));
  debugPrint("LatLast: " + String(lat2, 6));
  debugPrintln("LonLast: " + String(lon2, 6));
  
  if (distance <= radius) {
    return true;
  } else {
    return false;
  }
} 

void writeToCSV(const String& data) {
  // Generiere den Dateinamen basierend auf dem aktuellen Datum
  String fileName = generateFileName(gps);
  // Überprüfe, ob die Datei bereits existiert
  if (!SD.exists(fileName.c_str())) {
    // Datei existiert nicht, erstelle die Datei und schreibe die erste Zeile
    writeFile(SD, fileName.c_str(), firstline);
  }
  // Schreibe die Daten in die Datei
  appendFile(SD, fileName.c_str(), logging);
}

void processPosition() {   
  snprintf(lat, sizeof(lat), "%.6f", gps.location.lat());
  snprintf(lon, sizeof(lon), "%.6f", gps.location.lng());

  // Bestimme die Himmelsrichtung
  // filepath: /c:/esp32/GnssLogger/src/main.cpp
  char directionLatChar = getDirectionOfLat(gps.location.lat());
  char directionLngChar = getDirectionOfLng(gps.location.lng());
  snprintf(directionLat, sizeof(directionLat), "%c", directionLatChar);
  snprintf(directionLng, sizeof(directionLng), "%c", directionLngChar);

  snprintf(gpstime, sizeof(gpstime), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  snprintf(date, sizeof(date), "%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
  snprintf(hdop, sizeof(hdop), "%.1f", gps.hdop.hdop());
  snprintf(satellites, sizeof(satellites), "%d", gps.satellites.value());
  snprintf(speed, sizeof(speed), "%.1f", gps.speed.knots());
  snprintf(altitude, sizeof(altitude), "%.1f", gps.altitude.meters());
  snprintf(logging, sizeof(logging), "%s;%s;%s;%s;%s;%s;%s;%s;%s;%s", date, gpstime, lat, directionLat, lon, directionLng, speed, altitude, hdop, satellites);

  // Berechnung der Distanz zwischen der aktuellen und der letzten Position
  distanceLast = calculateDistance(atof(lat), atof(lon), atof(latLast), atof(lonLast));

  // Weitere Verarbeitung und Speicherung der Positionsdaten
  // snprintf(logging, sizeof(logging), "%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%.6f", date, gpstime, lat, directionLat, lon, directionLng, speed, altitude, hdop, satellites, distanceLast);

  latDifference = calculateDifference(atof(lat), atof(latLast));
  lonDifference = calculateDifference(atof(lon), atof(lonLast));
  snprintf(logging + strlen(logging), sizeof(logging) - strlen(logging), ";%.6f;%.6f", latDifference, lonDifference);

  positionDifference = calculateDistance(atof(lat), atof(lon), atof(latLast), atof(lonLast));
  snprintf(logging + strlen(logging), sizeof(logging) - strlen(logging), ";%.6f", positionDifference);
  
  if (isMissionMode) {
  strcat(logging, ";mission\n");
  } else {
  strcat(logging, ";station\n");
  }

  // Ersetzen von '.' durch ',' in logging um Zahlen in die CSV-Datei zu schreiben
  for (int i = 0; i < strlen(logging); i++) {
    if (logging[i] == '.') {
      logging[i] = ',';
    }
  }

  // Debug-Ausgabe
  debugPrint("new logging: ");
  debugPrintln(logging);

  // Speichern der Daten in der Datei
  // String fileName = generateFileName(gps);
  // appendFile(SD, fileName.c_str(), logging);
  writeToCSV(logging);

  // Leeren des logging-Arrays
  memset(logging, 0, sizeof(logging)); 
}

void setup() {
  // Serial Monitor
  Serial.begin(115200);

  // WiFi und Bluetooth ausschalten
  WiFi.mode(WIFI_OFF);
  btStop();

  // Reduzieren der Clock-Rate auf 80 MHz
  setCpuFrequencyMhz(80);
  debugPrintln("CPU frequency set to 80 MHz");

  // Initialisiere die SD-Karte
  initializeSDCard();

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
      debugPrintln("Wakeup caused by timer");
      isWakedUpFromDeepSleep = true;
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


  // Load data from RTC memory only if waking up from deep sleep
  if (isWakedUpFromDeepSleep) {
    strcpy(gpstimeLast, rtcData.gpstimeLast);
    strcpy(dateLast, rtcData.dateLast);
    strcpy(latLast, rtcData.latLast);
    strcpy(lonLast, rtcData.lonLast);
    isMissionMode = rtcData.isMissionMode;
    timeDifference = rtcData.timeDifference;
    loadStationPositionsFromRTC(stationPositions);
  }

  // Debug-Ausgabe der geladenen Werte
  if (isWakedUpFromDeepSleep && !isMissionMode) {
    debugPrintln("isWakedUpFromDeepSleep -> Station Mode");
  } else if (isWakedUpFromDeepSleep && isMissionMode) {
    debugPrintln("WakedUpFromDeepSleep -> Mission Mode");
  }
  debugPrint("LatLast: ");
  debugPrint(rtcData.latLast);
  debugPrint(", LonLast: ");
  debugPrintln(rtcData.lonLast);

  // Initialisiere die LED-Pins
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
}

void loop() {
  // Read data from the GPS module
  while (gpsSerial.available() > 0) {
    static unsigned long lastPositionTime = 0;
    gps.encode(gpsSerial.read());
  }
  unsigned long currentTime = millis();
  unsigned long lastPositionTime = 0;
  if (currentTime - lastPositionTime >= switchTime) { // Wartezeit von mindestens 0,25 Sekunde
    lastPositionTime = currentTime;
    if ((gps.location.isUpdated()) && (gps.hdop.hdop() < hdopTreshold) && (gps.date.year()) != 2000 && (gps.date.month()) != 0 && (gps.date.day()) != 0  && (gps.time.hour()) != 0 && (gps.time.minute()) != 0 && (gps.time.second()) != 0 ) {
    // Überprüfung ob die Position aktualisiert wurde und der HDOP-Wert unter dem Schwellenwert liegt
    // Aufrufen der Funktion zur Verarbeitung und Speicherung der Positionsdaten
    processPosition();

    // Berechne die Zeitdifferenz zwischen gpstime und gpstimeLast
    if (strlen(gpstimeLast) > 0) {
      timeDifference = getTimeDifference(gpstime, gpstimeLast);
      debugPrint("Time difference: " + String(timeDifference) + " seconds");
      debugPrintln("timeToLastPositionTreshold: " + String(timeToLastPositionTreshold) + " seconds");
    }
    if ((positionDifference > setCircleAroundPosition()) || (timeDifference > timeToLastPositionTreshold) || (strlen(gpstimeLast) == 0)) { 
      // Überprüfe, ob die letzte Position lang zurückliegt, zB weil das GPS-Modul neu gestartet wurde 
      // und die Zeitdifferenz größer als der Schwellenwert ist
      // Wenn true wird der Mission-Modus aktiviert und der Postionsspeicher geleert
      // neue Station-Positionen werden am Anfang der Liste hinzugefügt
      debugPrintln("Clear stationPositions due to position difference or time difference");
      isMissionMode = true;
      stationPositions.clear();
      stationPositions.push_back(std::make_pair(atof(lat), atof(lon)));
      
      // Ermitteln von 10 Positionen für den Station-Mode
      debugPrintln("Building SMM stationPositions");
      while (stationPositions.size() < 10) {
        // Warte auf die nächste gültige Position
 
        while (gpsSerial.available() > 0) {
          gps.encode(gpsSerial.read());
        }
        if (gps.location.isUpdated() && gps.hdop.hdop() < hdopTreshold && gps.date.year() != 2000 && gps.date.month() != 0 && gps.date.day() != 0 && gps.time.hour() != 0 && gps.time.minute() != 0 && gps.time.second() != 0) {
          lastLat = gps.location.lat();          
          lastLon = gps.location.lng();
          newLat = gps.location.lat();
          newLon = gps.location.lng();
          processPosition();
          // if (isWithinRange(newLat, newLon, stationPositions.back().first, stationPositions.back().second, circleAroundPosition)) {
          if (isWithinRange(atof(lat), atof(lon), atof(latLast), atof(lonLast),setCircleAroundPosition())) {
            stationPositions.push_back(std::make_pair(newLat, newLon));
            debugPrintln("Added position to stationPositions: " + String(newLat, 6) + ", " + String(newLon, 6));
          } else {
            debugPrintln("Position out of range: " + String(newLat, 6) + ", " + String(newLon, 6));
            stationPositions.clear();
            stationPositions.push_back(std::make_pair(atof(lat), atof(lon)));
          }
        }
      }
      
      if (stationPositions.size() == 10) {
        isMissionMode = false;
        debugPrintln("Switched to Station Mode");
        // Schreibe nur die erste Position aus stationPositions auf die SD-Karte
        if (!stationPositions.empty()) {
          const auto& pos = stationPositions.front();
          snprintf(logging, sizeof(logging), "%s;%s;%.6f;%s;%.6f;%s;%s;%s;%s;%s;station-mode\n", date, gpstime, pos.first, directionLat, pos.second, directionLng, speed, altitude, hdop, satellites);
        }
      }
    }

    // Wechsel zwischen Station- und Mission-Modus
    if (isMissionMode) {
        // if (strcmp(date, "2000/00/00") != 0) {
        // Schalte die LEDs entsprechend dem Modus
        if (TEST) {
          blinkMorseCode("G", GREEN_LED_PIN, 1,TEST); // Grüne LED blinkt im Mission-Modus
        }
      // }
      if (millis() - lastSwitchTime >= switchInterval) {
        bool withinRange = false;
        for (const auto& pos : stationPositions) {
          if (isWithinRange(atof(lat), atof(lon), atof(latLast), atof(lonLast),setCircleAroundPosition())) {
            // Überprüfen, ob die aktuelle Position innerhalb des Radius der stationPositions liegt
            withinRange = true;
            // break;
          }
        }
        if (withinRange) {
          isMissionMode = false;
          lastSwitchTime = millis();
          debugPrintln("Switched to Station Mode");
        }
      }
      // Aktivieren des Light-Sleep-Modus im Mission-Modus
      enableLightSleep(sleepingTimeLightSleep);

    } else {  // Station-Modus
      // Überprüfen, ob die aktuelle Position außerhalb des Radius der stationPositions liegt
      bool outsideDoubleRadius = true;
      for (const auto& pos : stationPositions) {
        debugPrintln("lat: " + String(atof(lat), 6) + ", lon: " + String(atof(lon), 6));
        debugPrintln("Checking position first/second: " + String(pos.first, 6) + ", " + String(pos.second, 6));
        debugPrintln("circleAroundPosition : " + String(setCircleAroundPosition()));
        if (!isWithinRange(atof(lat), atof(lon), atof(latLast), atof(lonLast),setCircleAroundPosition())) {
          isMissionMode = true;
          stationPositions.clear();
          debugPrintln("Switched to Mission Mode due to position outside double radius");
        }
      }
      if (!isMissionMode) {      // Aktivieren des Deep-Sleep-Modus im Station-Modus
        if (stationPositions.size() >= 5) {
        saveStationPositionsToRTC(stationPositions);
        }
        // Save the last values
        strcpy(gpstimeLast, gpstime);
        strcpy(dateLast, date);
        strcpy(latLast, lat);
        strcpy(lonLast, lon);
        
        debugPrintln("Switched to Deep Sleep Mode");
        debugPrint("gpstimeLast: ");
        debugPrint(gpstimeLast);
        debugPrint(", dateLast: ");
        debugPrint(dateLast);
        debugPrint(", latLast: ");
        debugPrint(latLast);
        debugPrint(", lonLast: ");
        debugPrint(lonLast);
        debugPrint(", isMissionMode: ");
        debugPrintln(isMissionMode ? "true" : "false");
        
        // Speichern der Daten im RTC-Speicher
        strcpy(rtcData.gpstimeLast, gpstimeLast);
        strcpy(rtcData.dateLast, dateLast);
        strcpy(rtcData.latLast, latLast);
        strcpy(rtcData.lonLast, lonLast);
        rtcData.isMissionMode = isMissionMode;
        rtcData.timeDifference = timeDifference;
        enableDeepSleep(sleepingTimeDeepSleep);
      } else {
        // Aktivieren des Light-Sleep-Modus im Mission-Modus
        enableLightSleep(sleepingTimeLightSleep);
      }
     }
    }
  }
}