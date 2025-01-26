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

const String BRANCH="sunday"; // Branch name

// Deklaration von Variablen

unsigned long lastSwitchTime = 0, timeDifference = 0;
double positionDifference = 0.0;
char gpstime[10] = "", date[11] = "", lat[15] = "", directionLat[2] = "", lon[15] = "", directionLng[2] = "", speed[10] = "", altitude[10] = "", hdop[10] = "", satellites[10] = "", logging[100] = "";
char gpstimeLast[10] = "", dateLast[11] = "", latLast[15] = "", lonLast[15] = "", speedLast[10] = "", altitudeLast[10] = "", hdopLast[10] = "", satellitesLast[10] = "", loggingLast[100] = "", firstlineLast[100] = "";  
double distanceLast = 0.0, latDifference = 0.0, lonDifference = 0.0;
bool isMissionMode = true;
bool isWakedUpFromLightSleep = false;
bool isWakedUpFromDeepSleep = false;

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

const unsigned long switchInterval = 5000; // 5 Sekunden
const double circleAroundPosition = 15.0; // Radius in Metern
const unsigned long sleepingTimeLightSleep = 2; // 2 Sekunden
const unsigned long sleepingTimeDeepSleep = 5; // 5 Sekunden
const double hdopTreshold = 1; // HDOP-Schwellenwert

const unsigned long timeToLastPositionTreshold = 20; // Zeitdifferenz-Schwellenwert in Sekunden
const unsigned long delayTime = 500; // LED blink delay time
const unsigned long switchTime = 1000; // Zeitdifferenz-Schwellenwert in Sekunden
const char firstline[] = "Date;UTC;Lat;N/S;Lon;E/W;knots;Alt/m;HDOP;Satellites;Fix-distance/m;LatDiff;LonDiff;Distance/m\n";

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

// Funktion zur Überprüfung, ob eine Position innerhalb eines bestimmten Radius liegt
bool isWithinRange(double lat1, double lon1, double lat2, double lon2, double radius) {
  double distance = calculateDistance(lat1, lon1, lat2, lon2);
  debugPrint("Distance: " + String(distance) + " meters");
  debugPrint(" Radius: " + String(radius) + " meters");
  debugPrint("Lat: " + String(lat1));
  debugPrint("Lon: " + String(lat1));
  debugPrint("LatLast: " + String(lat2));
  debugPrintln("LonLast: " + String(lon2));
  
  return distance <= radius;
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
  snprintf(logging + strlen(logging), sizeof(logging) - strlen(logging), ";%.6f\n", positionDifference);

  // Ersetzen von '.' durch ',' in logging um Zahlen in die CSV-Datei zu schreiben
  for (int i = 0; i < strlen(logging); i++) {
    if (logging[i] == '.') {
      logging[i] = ',';
    }
  }

  // Debug-Ausgabe
  Serial.print("new logging: ");
  Serial.println(logging);

  // Speichern der Daten in der Datei
  String fileName = generateFileName(gps);
  appendFile(SD, fileName.c_str(), logging);
  // Leeren des logging-Arrays
  memset(logging, 0, sizeof(logging)); 

}

void writeToCSV(const String& data) {
  String fileName = generateFileName(gps);
  File file = SD.open(fileName.c_str(), FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }

  // Schreibe die erste Zeile, falls die Datei neu erstellt wird
  if (file.size() == 0) {
    file.println(firstline);
  }

  // Schreibe die übergebenen Daten in die Datei
  file.println(data);
  file.close();
}

void setup() {
  // Serial Monitor
  Serial.begin(115200);

  // WiFi und Bluetooth ausschalten
  WiFi.mode(WIFI_OFF);
  btStop();

  // Reduzieren der Clock-Rate auf 80 MHz
  setCpuFrequencyMhz(80);
  Serial.println("CPU frequency set to 80 MHz");

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
      Serial.println("Wakeup caused by timer");
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
  Serial.print("LatLast: ");
  Serial.print(rtcData.latLast);
  Serial.print(", LonLast: ");
  Serial.println(rtcData.lonLast);

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
    if ((timeDifference > timeToLastPositionTreshold) || (strlen(gpstimeLast) == 0)) { 
      // Überprüfe, ob die letzte Position lang zurückliegt, zB weil das GPS-Modul neu gestartet wurde 
      // und die Zeitdifferenz größer als der Schwellenwert ist
      // Wenn true wird der Mission-Modus aktiviert und der Postionsspeicher geleert
      // neue Station-Positionen werden am Anfang der Liste hinzugefügt
      debugPrintln("Clear stationPositions due to time difference");
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
          double newLat = gps.location.lat();
          double newLon = gps.location.lng();
          if (isWithinRange(newLat, newLon, stationPositions.back().first, stationPositions.back().second, circleAroundPosition)) {
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
      // Schreibe nur im Mission-Modus auf die SD-Karte
      if (strcmp(date, "2000/00/00") != 0) {
        // Schalte die LEDs entsprechend dem Modus
        if (TEST) {
          blinkMorseCode("G", GREEN_LED_PIN, 1); // Grüne LED blinkt im Mission-Modus
        }
      }
      if (millis() - lastSwitchTime >= switchInterval) {
        bool withinRange = false;
        for (const auto& pos : stationPositions) {
          if (isWithinRange(atof(lat), atof(lon), pos.first, pos.second, circleAroundPosition)) {
            withinRange = true;
            break;
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
      // Überprüfen, ob die aktuelle Position außerhalb des doppelten Radius der stationPositions liegt
      bool outsideDoubleRadius = true;
      for (const auto& pos : stationPositions) {
        debugPrintln("lat: " + String(atof(lat), 6) + ", lon: " + String(atof(lon), 6));
        debugPrintln("Checking position first/second: " + String(pos.first, 6) + ", " + String(pos.second, 6));
        debugPrintln("circleAroundPosition * 2: " + String(2 * circleAroundPosition));
        if (isWithinRange(atof(lat), atof(lon), pos.first, pos.second, 2 * circleAroundPosition)) {
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
        // for (const auto& pos : stationPositions) {
        //   snprintf(logging, sizeof(logging), "%s;%s;%.6f;%s;%.6f;%s;%s;%s;%s;%s;station-mode\n", date, gpstime, pos.first, directionLat, pos.second, directionLng, speed, altitude, hdop, satellites);
        //   processPosition();
        // }
        
        // Save the last values
        strcpy(gpstimeLast, gpstime);
        strcpy(dateLast, date);
        strcpy(latLast, lat);
        strcpy(lonLast, lon);
        
        debugPrintln("Switched to Deep Sleep Mode");
        Serial.print("gpstimeLast: ");
        Serial.print(gpstimeLast);
        Serial.print(", dateLast: ");
        Serial.print(dateLast);
        Serial.print(", latLast: ");
        Serial.print(latLast);
        Serial.print(", lonLast: ");
        Serial.print(lonLast);
        Serial.print(", isMissionMode: ");
        Serial.println(isMissionMode);
        delay(delayTime); // Wartezeit für die LED-Anzeige

        // Speichern der Daten im RTC-Speicher
        strcpy(rtcData.gpstimeLast, gpstimeLast);
        strcpy(rtcData.dateLast, dateLast);
        strcpy(rtcData.latLast, latLast);
        strcpy(rtcData.lonLast, lonLast);
        rtcData.isMissionMode = isMissionMode;
        rtcData.timeDifference = timeDifference;
        enableDeepSleep(sleepingTimeDeepSleep);
      }

      // Schalte die LEDs entsprechend dem Modus
      if (TEST) {
        blinkMorseCode("R", RED_LED_PIN, 1); // Rote LED blinkt im Station-Modus
      }
    }

    // // Füge die aktuelle Position zur Liste der letzten 5 Positionen hinzu
    // stationPositions.push_back({atof(lat), atof(lon)});
    // if (stationPositions.size() > 5) {
    //   stationPositions.pop_front();
    // }

    // Speichern der Daten im RTC-Speicher
    strcpy(rtcData.gpstimeLast, gpstimeLast);
    strcpy(rtcData.dateLast, dateLast);
    strcpy(rtcData.latLast, latLast);
    strcpy(rtcData.lonLast, lonLast);
    rtcData.isMissionMode = isMissionMode;
    rtcData.timeDifference = timeDifference;
    }
  } // End of loop min 1 sec
}