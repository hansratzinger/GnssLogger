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
#include "esp_log.h"

// Define the RX and TX pins for Serial 2
#define RXD2 3 // RX Pin
#define TXD2 1 // TX Pin

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
  char* lat = (char*)malloc(15 * sizeof(char));
  char* lon = (char*)malloc(15 * sizeof(char));
  char* directionLat = (char*)malloc(2 * sizeof(char));
  char* directionLng = (char*)malloc(2 * sizeof(char));
  char* gpstime = (char*)malloc(10 * sizeof(char));
  char* date = (char*)malloc(11 * sizeof(char));
  char* hdop = (char*)malloc(10 * sizeof(char));
  char* satellites = (char*)malloc(10 * sizeof(char));
  char* speed = (char*)malloc(10 * sizeof(char));
  char* altitude = (char*)malloc(10 * sizeof(char));
  char* logging = (char*)malloc(130 * sizeof(char));

  // Überprüfe, ob die Speicherzuweisung erfolgreich war
  if (lat == NULL || lon == NULL || directionLat == NULL || directionLng == NULL || gpstime == NULL || date == NULL || hdop == NULL || satellites == NULL || speed == NULL || altitude == NULL || logging == NULL) {
    Serial.println("Speicherzuweisung fehlgeschlagen");
    return;
  }

  snprintf(lat, 15, "%.6f", gps.location.lat());
  snprintf(lon, 15, "%.6f", gps.location.lng());

  // Bestimme die Himmelsrichtung
  char directionLatChar = getDirectionOfLat(gps.location.lat());
  char directionLngChar = getDirectionOfLng(gps.location.lng());
  snprintf(directionLat, 2, "%c", directionLatChar);
  snprintf(directionLng, 2, "%c", directionLngChar);

  snprintf(gpstime, 10, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  snprintf(date, 11, "%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
  snprintf(hdop, 10, "%.1f", gps.hdop.hdop());
  snprintf(satellites, 10, "%d", gps.satellites.value());
  snprintf(speed, 10, "%.1f", gps.speed.knots());
  snprintf(altitude, 10, "%.1f", gps.altitude.meters());
  snprintf(logging, 130, "%s;%s;%s;%s;%s;%s;%s;%s;%s;%s", date, gpstime, lat, directionLat, lon, directionLng, speed, altitude, hdop, satellites);

  // Berechnung der Distanz zwischen der aktuellen und der letzten Position
  distanceLast = calculateDistance(atof(lat), atof(lon), atof(latLast), atof(lonLast));

  latDifference = calculateDifference(atof(lat), atof(latLast));
  lonDifference = calculateDifference(atof(lon), atof(lonLast));
  snprintf(logging + strlen(logging), 130 - strlen(logging), ";%.6f;%.6f", latDifference, lonDifference);

  positionDifference = calculateDistance(atof(lat), atof(lon), atof(latLast), atof(lonLast));
  snprintf(logging + strlen(logging), 130 - strlen(logging), ";%.6f", positionDifference);
  
  // Read MPU6050 values and append to logging variable
  // readMPU6050();  // Read accelerometer values from MPU6050
  snprintf(logging + strlen(logging), 130 - strlen(logging), ";%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f", accelX, accelY, accelZ,gyroX, gyroY, gyroZ, temp);

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
  memset(logging, 0, 130); 

  // Speicher freigeben
  free(lat);
  free(lon);
  free(directionLat);
  free(directionLng);
  free(gpstime);
  free(date);
  free(hdop);
  free(satellites);
  free(speed);
  free(altitude);
  free(logging);
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
      strncpy(gpstimeLast, gpstime, sizeof(gpstimeLast) - 1);
      gpstimeLast[sizeof(gpstimeLast) - 1] = '\0'; // Sicherstellen, dass die Zielzeichenkette nullterminiert ist
      strncpy(dateLast, date, sizeof(dateLast) - 1);
      dateLast[sizeof(dateLast) - 1] = '\0'; // Sicherstellen, dass die Zielzeichenkette nullterminiert ist
      strncpy(latLast, lat, sizeof(latLast) - 1);
      latLast[sizeof(latLast) - 1] = '\0'; // Sicherstellen, dass die Zielzeichenkette nullterminiert ist
      strncpy(lonLast, lon, sizeof(lonLast) - 1);
      lonLast[sizeof(lonLast) - 1] = '\0'; // Sicherstellen, dass die Zielzeichenkette nullterminiert ist      
    }

    ledOFF(GREEN_LED_PIN,TEST); // position processed finished
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void communication( void * parameter ) {
  for (;;) {
    // Debug-Ausgabe
    ledON(RED_LED_PIN,TEST); // Communication task running
    if (TEST) {
      Serial.println("Communication task running...");
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ledOFF(RED_LED_PIN,TEST); // Communication task finished
    vTaskDelay(100 / portTICK_PERIOD_MS);(100,1);
  }
}

void setup() {
  // TRACE();
  // Setzen Sie die Watchdog-Timeout-Zeit auf 10 Sekunden
  ESP_ERROR_CHECK(esp_task_wdt_init(10, true));
  
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
  // BREAK(); // Breakpoint
  // DUMP(RELEASE); // Dump all variables
  // WiFi und Bluetooth ausschalten
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.println("WiFi and Bluetooth turned off");

  if (!initializeSDCard()) {  // Initialisiere die SD-Karte und überprüfe, ob sie bereit ist    
    Serial.println("SD card initialization failed");
    Serial.println("SD card not ready, System will reboot in 5 seconds");
    blinkMorseCode("SOS", RED_LED_PIN, 1,TEST); // Rote LED blinkt 3 mal
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    esp_restart();
  } else {  
    listDir(SD, "/", 0);
    ledOFF(RED_LED_PIN,TEST); // Rote LED finished setup
    Serial.println("SD card initialized");
    // Überprüfen Sie, ob die Datei debug.txt vorhanden ist
    if (!SD.exists("/debug.txt")) {
      // Datei existiert nicht, erstelle die Datei
      File file = SD.open("/debug.txt", FILE_WRITE);
      if (file) {
        file.println("Debug file created"); 
        file.close();
      } else {
        Serial.println("Failed to create debug file");
      } 
    }
  }
  
  // // Initialize the MPU6050
  // setupMPU6050();
  // Serial.println("MPU6050 initialized");
  
  TaskHandle_t taskNavigationHandle = NULL;
  TaskHandle_t taskCommunicationHandle = NULL;

  // Create the tasks
  xTaskCreatePinnedToCore(
    navigation, // Function to implement the task
    "navigation", // Name of the task
    25000, // Stack size in words
    NULL, // Task input parameter
    1, // Priority of the task
    &taskNavigationHandle, // Task handle
    1); // Core where the task should run

  xTaskCreatePinnedToCore(
    communication, // Function to implement the task
    "communication", // Name of the task
    25000, // Stack size in words
    NULL, // Task input parameter
    0, // Priority of the task
    &taskCommunicationHandle, // Task handle
    1); // Core where the task should run

  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Nichts zu tun
}