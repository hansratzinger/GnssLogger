#include <Arduino.h>
#include <pins.h>
#define GPS_BAUD 115200
#define SERIALMONITOR_BAUD 115200

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
#include <I2Cdev.h>
// #include <MPU6050.h>
#include "esp_log.h"
#include <FS.h>


static const uint32_t WDT_TIMEOUT_MS = 30000;  // 30 Sekunden Timeout
static const uint32_t TASK_DELAY_MS = 100;     // Task Delay

SemaphoreHandle_t watchdogMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;
xTaskHandle navigationTask = NULL;
xTaskHandle communicationTask = NULL;

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

// Alte Header-Zeile mit acc,gyro,temp
// const char firstline[] = "Date;UTC;Lat;N/S;Lon;E/W;knots;Alt/m;HDOP;Satellites;LatDiff;LonDiff;Distance/m;acc/x;acc/y;acc/z;gyro/x;gyro/y;gyro/z;temp/C\n";
const char firstline[] = "Date;UTC;Lat;N/S;Lon;E/W;knots;Alt/m;HDOP;Satellites;LatDiff;LonDiff;Distance/m\n";

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(1); // Initialisierung von gpsSerial

// Simulierte GPS-Daten für Testzwecke
// void simulateGPS(TinyGPSPlus& gps) {
//     static uint32_t timestamp = 0;
//     static double lat = 48.484272;  // Startposition
//     static double lon = 16.471992;
//     static double alt = 145.3;
//     static float speed = 0.8;
//     static int satellites = 10;
//     static float hdop = 1.2;
    
//     // Zeitstempel aktualisieren
//     timestamp += 1000; // 1 Sekunde weiterzählen
    
//     // Position leicht ändern
//     lat += 0.000010;  // ca. 1m nach Norden
//     lon += 0.000015;  // ca. 1m nach Osten
    
//     // TinyGPS+ Daten mittels encode() updaten
//     char nmea[100];
//     int year = 2024;
//     int month = 2;
//     int day = 17;
//     int hour = 14;
//     int minute = 30;
//     int second = timestamp / 1000 % 60;

//     // GPRMC Satz generieren
//     snprintf(nmea, sizeof(nmea), 
//         "$GPRMC,%02d%02d%02d.000,A,%02.6f,N,%03.6f,E,%.1f,%.1f,170224,,*",
//         hour, minute, second, lat, lon, speed, hdop);
    
//     // Checksumme berechnen und anhängen
//     byte checksum = 0;
//     for(int i = 1; i < strlen(nmea) - 1; i++) {
//         checksum ^= nmea[i];
//     }
//     char final_nmea[100];
//     snprintf(final_nmea, sizeof(final_nmea), "%s%02X\r\n", nmea, checksum);
    
//     // NMEA Satz an TinyGPS+ übergeben
//     for(int i = 0; i < strlen(final_nmea); i++) {
//         gps.encode(final_nmea[i]);
//     }
// }


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

bool writeToCSV(const String& data) {
    if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
        Serial.println("Attempting to write to CSV...");
        Serial.println("Data to write: " + data);
        xSemaphoreGive(serialMutex);
    }

    // Generiere den Dateinamen
    String fileName = generateFileName(gps);
    bool isNewFile = !SD.exists(fileName.c_str());
    
    if(isNewFile) {
        if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            Serial.println("Creating new file with header");
            xSemaphoreGive(serialMutex);
        }
        
        // Neue Datei mit Header erstellen
        if(!writeFile(SD, fileName.c_str(), firstline)) {
            if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
                Serial.println("Failed to write header!");
                xSemaphoreGive(serialMutex);
            }
            return false;
        }
    }

    // Daten anhängen
    String dataWithNewline = data + "\n";  // Zeilenumbruch hinzufügen
    if(!appendFile(SD, fileName.c_str(), dataWithNewline.c_str())) {
        if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            Serial.println("Failed to append data!");
            xSemaphoreGive(serialMutex);
        }
        return false;
    }

    // Datei zur Überprüfung öffnen und Größe ausgeben
    File file = SD.open(fileName.c_str());
    if(file) {
        if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            Serial.printf("File size after write: %lu bytes\n", file.size());
            xSemaphoreGive(serialMutex);
        }
        file.close();
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

    if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
      Serial.println("Processing position data...");
      xSemaphoreGive(serialMutex);
    }
    
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

    // MPU6050-Block 
    /* 
    snprintf(logging + strlen(logging), sizeof(logging) - strlen(logging), 
        ";%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f", 
        accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp);
    */

    // Dezimalpunkte durch Kommas ersetzen
    for (size_t i = 0; i < strlen(logging); i++) {
        if (logging[i] == '.') {
            logging[i] = ',';
        }
    }

    // Nach dem Zusammenbau des logging-Strings
    if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
        Serial.println("Logging string built: " + String(logging));
        xSemaphoreGive(serialMutex);
    }

    if(strlen(logging) > 0) {
        if(!writeToCSV(String(logging))) {
            if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
                Serial.println("Failed to write to CSV file!");
                xSemaphoreGive(serialMutex);
            }
        }
    } else {
        if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            Serial.println("Error: Empty logging string!");
            xSemaphoreGive(serialMutex);
        }
    }
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




    
    // for(;;) {
    //     // GPS-Daten simulieren
    //     simulateGPS(gps);
        
    //     if (gps.location.isUpdated()) {
    //         digitalWrite(RED_LED_PIN, HIGH);
    //         if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
    //             Serial.println("Simulated GPS data received");
    //             xSemaphoreGive(serialMutex);
    //         }
    //         processPosition();  // Verarbeite die GPS-Daten
    //         digitalWrite(RED_LED_PIN, LOW);
    //     }
        
    //     // Watchdog zurücksetzen mit Mutex
    //     if(xSemaphoreTake(watchdogMutex, pdMS_TO_TICKS(100))) {
    //         esp_task_wdt_reset();
    //         xSemaphoreGive(watchdogMutex);
    //     }
        
    //     vTaskDelay(pdMS_TO_TICKS(1000)); // Längeres Delay für die Simulation
    // }
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