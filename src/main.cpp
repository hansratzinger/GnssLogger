// ----------------------------------------------------------------------------------------------
// GNSSLogger  
// This project uses the ESP32 Dev Module board and a GNSS module to log GPS data to an SD card.
// The GNSS module is connected to the ESP32 via UART.
// // The ESP32 is put into deep sleep mode when the GNSS module is not in use.
// // The ESP32 wakes up from deep sleep mode every 5 seconds to check the GNSS module for new data.
// If the GNSS module has new data, the ESP32 processes the data and logs it to an SD card.
// // The ESP32 switches between mission mode and station mode based on the distance between the current position and the last position.
// // In station mode, the ESP32 logs the current position to the SD card and goes into deep sleep mode.
// // In mission mode, the ESP32 checks if the current position is within a certain radius of the station positions.
// // If the current position is within the radius, the ESP32 switches to station mode.
// // If the current position is outside the radius, the ESP32 logs the current position to the SD card and goes into deep sleep mode.
// // The ESP32 uses the RTC memory to store the last position, the last time, and the mode (mission or station).
// -  the SD card to log the GPS data in CSV format.
// -  the built-in LED pins to indicate the mode (mission or station) with red and green LEDs.
// -  the Morse_LED library to blink the LEDs in Morse code.
// -  the GNSS_module library to process the GPS data.
// -  the SD_card library to interact with the SD card.
// // -  the Sleep library to manage the sleep modes.
// -  the TinyGPS++ library to parse the GPS data.
// -  the WiFi library to turn off WiFi and Bluetooth.
// -  the SPI library to communicate with the SD card.
// -  the FS library to interact with the file system.
// -  the Arduino.h library for general Arduino functions.
// -  the HardwareSerial library to communicate with the GNSS module.
// // -  the esp_sleep library to manage the sleep modes.
// // -  the deque library to store the station positions.
// -  the stdio.h library for standard input and output functions.
// -  the string.h library for string manipulation functions.
// -  the time.h library for time-related functions.
// -  the Wire library to communicate with the MPU6050.
// -  the MPU6050 library to read the accelerometer and gyroscope values.
// -  the I2Cdev library to communicate with the MPU6050.
// -  the Adafruit_MPU6050 library to read the accelerometer and gyroscope values.
// -  the pins.h file to define the GPIO pins.
// -  the my_Helpers.h file to define helper functions.
// -  the freertos/FreeRTOS.h library to use FreeRTOS functions.
// -  the freertos/task.h library to create tasks.
// Hans Ratzinger 2025-02-10
// Release 2.1.0
// https://github.com/hansratzinger/GnssLogger
// ----------------------------------------------------------------------------------------------
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_task_wdt.h"
#include <WiFi.h>
#include "GNSS_module.h"
#include "SD_card.h"
#include <my_Helpers.h>
#include <SD.h>
#include <SPI.h>
#include <Arduino.h>
#include "pins.h"
#include <I2Cdev.h>
#include <MPU6050.h>


// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 115200
#define SERIALMONITOR_BAUD 115200

// Define the GPIO pins for the LEDs
const int RED_LED_PIN = 25; // station mode
const int GREEN_LED_PIN = 26; // mission mode

const String BRANCH="release-2.1-GSM-FreeRTOS"; // Branch name
const String RELEASE="2.1.0"; // Branch name

// Deklaration von Variablen

unsigned long lastSwitchTime = 0, timeDifference = 0;
double positionDifference = 0.0;
char gpstime[10] = "", date[11] = "", lat[15] = "", directionLat[2] = "", lon[15] = "", directionLng[2] = "", speed[10] = "", altitude[10] = "", hdop[10] = "", satellites[10] = "";
char gpstimeLast[10] = "", dateLast[11] = "", latLast[15] = "", lonLast[15] = "", speedLast[10] = "", altitudeLast[10] = "", hdopLast[10] = "", satellitesLast[10] = "", loggingLast[100] = "", firstlineLast[100] = "";  
char logging[130] = "";
char gpsData[130] = "";
double distanceLast = 0.0, latDifference = 0.0, lonDifference = 0.0;


bool isSDcardReady = false;

const bool TEST = true; // Definition der Konstante TEST

unsigned long lastPositionTime = 0;
unsigned long currentTime = 0;
// const unsigned long timeToLastPositionTreshold = 30; // Zeitdifferenz-Schwellenwert in Sekunden
const int mydelayTime = 250; // LED blink mydelay time
// const int switchTime = 50; // Zeitdifferenz zwischen Positionen in msec
const char firstline[] = "Date;UTC;Lat;N/S;Lon;E/W;knots;Alt/m;HDOP;Satellites;LatDiff;LonDiff;Distance/m;acc/x;acc/y;acc/z;gyro/x;gyro/y;gyro/z;temp/C\n";

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2); // Initialisierung von gpsSerial

// Funktion zur Berechnung der Zeitdifferenz zwischen gpstime und gpstimeLast
unsigned long getTimeDifference(const char *gpstime, const char *gpstimeLast) {
  int hour1 = 0, minute1 = 0, second1 = 0;
  int hour2 = 0, minute2 = 0, second2 = 0;
 
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

  latDifference = calculateDifference(atof(lat), atof(latLast));
  lonDifference = calculateDifference(atof(lon), atof(lonLast));
  snprintf(logging + strlen(logging), sizeof(logging) - strlen(logging), ";%.6f;%.6f", latDifference, lonDifference);

  positionDifference = calculateDistance(atof(lat), atof(lon), atof(latLast), atof(lonLast));
  snprintf(logging + strlen(logging), sizeof(logging) - strlen(logging), ";%.6f", positionDifference);
  
  // Read MPU6050 values and append to logging variable
  readMPU6050();  // Read accelerometer values
  snprintf(logging + strlen(logging), sizeof(logging) - strlen(logging), ";%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f", accelX, accelY, accelZ,gyroX, gyroY, gyroZ, temp);

  // Ersetzen von '.' durch ',' in logging um Zahlen in die CSV-Datei zu schreiben
  for (int i = 0; i < strlen(logging); i++) {
    if (logging[i] == '.') {
      logging[i] = ',';
    }
  }

  strcat(logging, "\n"); // Füge einen Zeilenumbruch hinzu

  // Debug-Ausgabe
  Serial.print("new logging: ");
  Serial.println(logging);

  writeToCSV(logging);

  // Leeren des logging-Arrays
  memset(logging, 0, sizeof(logging)); 
}


void navigation( void * parameter ) {
  char receivedChar;
  for (;;) {
    // Read data from the GPS module
    while (gpsSerial.available() > 0) {
      // blinkMorseCode("e", GREEN_LED_PIN, 1,TEST); // Green LED for searching GPS data
      receivedChar = gpsSerial.read();
      Serial.print(receivedChar); // Ausgabe der empfangenen Daten auf dem seriellen Monitor

      // gps.encode(gpsSerial.read());
      gps.encode(receivedChar);
    }

    if ((gps.location.isUpdated()) && (gps.date.year()) != 2000 && (gps.date.month()) != 0 && (gps.date.day()) != 0  && (gps.time.hour()) != 0 && (gps.time.minute()) != 0 && (gps.time.second()) != 0 ) {
      if (TEST) {
        Serial.println("");
        Serial.print("Latitude: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(" Longitude: ");
        Serial.print(gps.location.lng(), 6);
        Serial.print(" HDOP: ");
        Serial.println(gps.hdop.hdop());
      }
      // Aufrufen der Funktion zur Verarbeitung und Speicherung der Positionsdaten
      ledON(GREEN_LED_PIN,TEST); // position processing
      // Read MPU6050 values and append to logging variable
      processPosition();
             
      // Save the last values
      strcpy(gpstimeLast, gpstime);
      strcpy(dateLast, date);
      strcpy(latLast, lat);
      strcpy(lonLast, lon);
      ledOFF(GREEN_LED_PIN,TEST); // position processed finished
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void communication( void * parameter ) {
  for (;;) {
    // Debug-Ausgabe
    ledON(RED_LED_PIN,TEST); // Communication task running
    Serial.println("Communication task running...");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ledOFF(RED_LED_PIN,TEST); // Communication task finished
    vTaskDelay(100 / portTICK_PERIOD_MS);(100,1);
  }
}

void setup() {

  // Setzen Sie die Watchdog-Timeout-Zeit auf 10 Sekunden
  esp_task_wdt_init(10, true);
  
    // Initialisiere die LED-Pins
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  ledON(RED_LED_PIN,TEST); // Rote LED starting setup

  // Serial Monitor
  Serial.begin(SERIALMONITOR_BAUD);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2); // Serielle Schnittstelle für GNSS-Modul

  Serial.println("GNSS serial connection started...");

  // WiFi und Bluetooth ausschalten
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.println("WiFi and Bluetooth turned off");

  // Initialisiere die SD-Karte
  if (!initializeSDCard()) {
    isSDcardReady = false;
    Serial.println("SD card initialization failed");
  } else {
    isSDcardReady = true;
    Serial.println("SD card initialized");
  }

  // Initialize the MPU6050
  setupMPU6050();
  Serial.println("MPU6050 initialized");

  if (isSDcardReady) {
    listDir(SD, "/", 0);
    ledOFF(RED_LED_PIN,TEST); // Rote LED finished setup
  } else {
    Serial.println("SD card not ready, System will reboot in 5 seconds");
    blinkMorseCode("SOS", RED_LED_PIN, 1,TEST); // Rote LED blinkt 3 mal
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    esp_restart();
  }

  TaskHandle_t taskNavigationHandle = NULL;
  TaskHandle_t taskCommunicationHandle = NULL;

  // Create the tasks
  xTaskCreatePinnedToCore(
    navigation, // Function to implement the task
    "navigation", // Name of the task
    20000, // Stack size in words
    NULL, // Task input parameter
    1, // Priority of the task
    &taskNavigationHandle, // Task handle
    1); // Core where the task should run

  xTaskCreatePinnedToCore(
    communication, // Function to implement the task
    "communication", // Name of the task
    20000, // Stack size in words
    NULL, // Task input parameter
    1, // Priority of the task
    &taskCommunicationHandle, // Task handle
    0); // Core where the task should run

  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Nichts zu tun
}
