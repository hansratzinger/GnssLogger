// #include<ArduinoTrace.h>  
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

// LilyGO T-SIM7000G Pins
#define SD_MISO     2
#define SD_MOSI    15
#define SD_SCLK    14
#define SD_CS      13
#define LED_PIN    12
#define DTR_PIN     4
#define MODEM_RX   26
#define MODEM_TX   27
#define MODEM_PW   25
#define GPS_RX     17  // LC76G TX
#define GPS_TX     16  // LC76G RX
#define GPS_BAUD 115200
#define SERIALMONITOR_BAUD 115200
static const uint32_t WDT_TIMEOUT_MS = 30000;  // 30 Sekunden Timeout
static const uint32_t TASK_DELAY_MS = 100;     // Task Delay

SemaphoreHandle_t watchdogMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;
xTaskHandle navigationTask = NULL;
xTaskHandle communicationTask = NULL;

// Define the GPIO pins for the LEDs
const int RED_LED_PIN = 12; // LILLYGO T-SIM7000G -> ESP32 WROVER
// const int GREEN_LED_PIN = 26; // mission mode

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

const char firstline[] = "Date;UTC;Lat;N/S;Lon;E/W;knots;Alt/m;HDOP;Satellites;LatDiff;LonDiff;Distance/m;acc/x;acc/y;acc/z;gyro/x;gyro/y;gyro/z;temp/C\n";

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(1); // Initialisierung von gpsSerial

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

bool listDirectory(fs::FS &fs, const char * dirname, uint8_t levels) {
    if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
        Serial.printf("Listing directory: %s\n", dirname);
        xSemaphoreGive(serialMutex);
    }

    File root = fs.open(dirname);
    if(!root) {
        return false;
    }
    if(!root.isDirectory()) {
        return false;
    }

    File file = root.openNextFile();
    while(file) {
        if(file && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            if(file.isDirectory()) {
                Serial.print("  DIR : ");
                Serial.println(file.name());
            } else {
                Serial.print("  FILE: ");
                Serial.print(file.name());
                Serial.print("  SIZE: ");
                Serial.println(file.size());
            }
            xSemaphoreGive(serialMutex);
        }
        file = root.openNextFile();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

void processPosition() {
    // Statische Puffer statt dynamischer Speicherzuweisung
    static char lat[15];
    static char lon[15];
    static char directionLat[2];
    static char directionLng[2];
    static char gpstime[10];
    static char date[11];
    static char hdop[10];
    static char satellites[10];
    static char speed[10];
    static char altitude[10];
    static char logging[130];

    // GPS-Daten formatieren
    snprintf(lat, sizeof(lat), "%.6f", gps.location.lat());
    snprintf(lon, sizeof(lon), "%.6f", gps.location.lng());

    // Himmelsrichtung bestimmen
    char directionLatChar = getDirectionOfLat(gps.location.lat());
    char directionLngChar = getDirectionOfLng(gps.location.lng());
    snprintf(directionLat, sizeof(directionLat), "%c", directionLatChar);
    snprintf(directionLng, sizeof(directionLng), "%c", directionLngChar);

    // Zeit und Datum
    snprintf(gpstime, sizeof(gpstime), "%02d:%02d:%02d", 
        gps.time.hour(), gps.time.minute(), gps.time.second());
    snprintf(date, sizeof(date), "%04d-%02d-%02d", 
        gps.date.year(), gps.date.month(), gps.date.day());

    // GPS-Daten
    snprintf(hdop, sizeof(hdop), "%.1f", gps.hdop.hdop());
    snprintf(satellites, sizeof(satellites), "%d", gps.satellites.value());
    snprintf(speed, sizeof(speed), "%.1f", gps.speed.knots());
    snprintf(altitude, sizeof(altitude), "%.1f", gps.altitude.meters());

    // Logging-String zusammenbauen
    snprintf(logging, sizeof(logging), "%s;%s;%s;%s;%s;%s;%s;%s;%s;%s", 
        date, gpstime, lat, directionLat, lon, directionLng, 
        speed, altitude, hdop, satellites);

    // Distanzberechnungen
    double currentLat = atof(lat);
    double currentLon = atof(lon);
    double lastLat = atof(latLast);
    double lastLon = atof(lonLast);

    latDifference = calculateDifference(currentLat, lastLat);
    lonDifference = calculateDifference(currentLon, lastLon);
    positionDifference = calculateDistance(currentLat, currentLon, lastLat, lastLon);

    // Differenzen anhängen
    snprintf(logging + strlen(logging), sizeof(logging) - strlen(logging), 
        ";%.6f;%.6f;%.6f", latDifference, lonDifference, positionDifference);

    // MPU6050-Daten anhängen
    snprintf(logging + strlen(logging), sizeof(logging) - strlen(logging), 
        ";%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f", 
        accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp);

    // Dezimalpunkte durch Kommas ersetzen
    for (size_t i = 0; i < strlen(logging); i++) {
        if (logging[i] == '.') {
            logging[i] = ',';
        }
    }

    strcat(logging, "\n");

    // Debug-Ausgabe
    if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
        Serial.print("new logging: ");
        Serial.println(logging);
        xSemaphoreGive(serialMutex);
    }

    writeToCSV(logging);
}

void navigation(void * parameter) {
    // Watchdog Setup
    if(xSemaphoreTake(watchdogMutex, pdMS_TO_TICKS(100))) {
        if(esp_task_wdt_add(NULL) != ESP_OK) {
            Serial.println("Navigation: WDT Add failed");
        }
        xSemaphoreGive(watchdogMutex);
    }
    
    // GPS Setup
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
        Serial.println("Navigation started");
        xSemaphoreGive(serialMutex);
    }
    
    for(;;) {
        // Wenn keine GPS-Daten verfügbar sind, kurzes Delay und Watchdog Reset
        if (!gpsSerial.available()) {
            if(xSemaphoreTake(watchdogMutex, pdMS_TO_TICKS(100))) {
                esp_task_wdt_reset();
                xSemaphoreGive(watchdogMutex);
            }
            vTaskDelay(pdMS_TO_TICKS(100));  // Kürzeres Delay statt portMAX_DELAY
            continue;
        }

        // GPS-Daten verarbeiten wenn verfügbar
        while (gpsSerial.available() > 0) {
            if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
                Serial.print("."); // waiting for GPS data
                xSemaphoreGive(serialMutex);
            }
            gps.encode(gpsSerial.read());
            
            // Watchdog während der Datenverarbeitung zurücksetzen
            if(xSemaphoreTake(watchdogMutex, pdMS_TO_TICKS(100))) {
                esp_task_wdt_reset();
                xSemaphoreGive(watchdogMutex);
            }
        }
        
        if (gps.location.isUpdated()) {
            digitalWrite(RED_LED_PIN, HIGH);
            if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
                Serial.println("GPS data received");
                xSemaphoreGive(serialMutex);
            }
            processPosition();  // Verarbeite die GPS-Daten
            digitalWrite(RED_LED_PIN, LOW);
        }
        
        // Watchdog zurücksetzen mit Mutex
        if(xSemaphoreTake(watchdogMutex, pdMS_TO_TICKS(100))) {
            esp_task_wdt_reset();
            xSemaphoreGive(watchdogMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void communication(void * parameter) {
    // Task Setup - nur Task zum Watchdog hinzufügen
    if(esp_task_wdt_add(NULL) != ESP_OK) {
        if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            Serial.println("Communication: WDT Add failed");
            xSemaphoreGive(serialMutex);
        }
    }
    
    // Debug-Ausgabe mit Mutex
    if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
        Serial.println("Communication task started");
        xSemaphoreGive(serialMutex);
    }
    
    for(;;) {
        // LED Blinken
        digitalWrite(RED_LED_PIN, HIGH);
        
        // Debug-Ausgabe mit Mutex
        if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            Serial.println("Communication running");
            xSemaphoreGive(serialMutex);
        }
        
        digitalWrite(RED_LED_PIN, LOW);
        
        // Watchdog zurücksetzen
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup() {

    // SPI für SD-Karte initialisieren
    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    
    printf("Branch: %s\n", BRANCH.c_str());
    printf("Release: %s\n", RELEASE.c_str());
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Basis-Initialisierung
    Serial.begin(SERIALMONITOR_BAUD);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("Starting setup...");

    // // LED-Pins initialisieren
    // pinMode(RED_LED_PIN, OUTPUT);
    // // pinMode(GREEN_LED_PIN, OUTPUT);
    // digitalWrite(RED_LED_PIN, LOW);
    // // digitalWrite(GREEN_LED_PIN, LOW);

    // WiFi und Bluetooth ausschalten
    WiFi.mode(WIFI_OFF);
    btStop();
    Serial.println("WiFi and Bluetooth turned off");

    // setupMPU6050(); // MPU6050 initialisieren

    // Mutexe erstellen
    watchdogMutex = xSemaphoreCreateMutex();
    serialMutex = xSemaphoreCreateMutex();
    
    if (watchdogMutex == NULL || serialMutex == NULL) {
        Serial.println("Error creating mutexes");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    }

    // SD-Karte initialisieren mit Wiederholungsversuchen
    int retries = 3;
    bool sdInitialized = false;

    while (retries > 0 && !sdInitialized) {
        if (SD.begin(SD_CS, SPI)) {  // SPI explizit übergeben
            uint8_t cardType = SD.cardType();
            if (cardType != CARD_NONE) {
                sdInitialized = true;
                Serial.println("Card Mount Success");
                Serial.printf("SD Card Type: %d\n", cardType);
                Serial.println("SD card initialized");
                sdInitialized = true;
                
                // Verzeichnislisting durchführen
                if (!listDirectory(SD, "/", 0)) {
                    Serial.println("Directory listing failed");
                    sdInitialized = false;
                }
            }
        }
        
        if (!sdInitialized) {
            retries--;
            if (retries > 0) {
                Serial.printf("SD card initialization failed, retrying... (%d attempts left)\n", retries);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }

    if (!sdInitialized) {
        Serial.println("SD card initialization failed permanently");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    }

    // Hardware-Serial für GPS initialisieren
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.println("GPS Serial initialized");


    // Tasks erstellen
    Serial.println("Creating tasks...");
    
    // Navigation Task erstellen
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        navigation,
        "navigation",
        8192,
        NULL,
        2,
        &navigationTask,
        1
    );

    if (xReturned != pdPASS) {
        Serial.println("Navigation task creation failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    }
    Serial.println("Navigation task created");

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Communication Task erstellen
    xReturned = xTaskCreatePinnedToCore(
        communication,
        "communication",
        8192,
        NULL,
        1,
        (TaskHandle_t*)&communicationTask,
        0
    );

    if (xReturned != pdPASS) {
        Serial.println("Communication task creation failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    }
    Serial.println("Communication task created");
    Serial.println("Setup complete");
}

void loop() {
  // Nichts zu tun
}