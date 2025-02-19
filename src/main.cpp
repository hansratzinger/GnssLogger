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
#include "esp_log.h"
#include <FS.h>
#include <esp_now.h> // ESP-NOW Bibliothek
#include <WiFi.h>    // WiFi Bibliothek

#define DEBUG_SD 1  // Debug-Ausgaben aktivieren
#define MUTEX_TIMEOUT_MS 1000  // 1 Sekunde Timeout für Mutex-Operationen

// Globale Variablen für Mutexe
SemaphoreHandle_t watchdogMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;
SemaphoreHandle_t ledMutex = NULL;
// Globaler Mutex für SD-Karten Zugriffe
SemaphoreHandle_t sdMutex = NULL;
static constexpr uint32_t WDT_TIMEOUT_MS = 30000;  // 30 Sekunden Watchdog
static SemaphoreHandle_t mutexes[4] = {NULL, NULL, NULL, NULL};
enum MutexIndex { WATCHDOG_MUTEX = 0, SERIAL_MUTEX = 1, LED_MUTEX = 2, SD_MUTEX = 3 };

// Task Handles
xTaskHandle navigationTask = NULL;

// Vorwärtsdeklarationen der Task-Funktionen
void navigation(void * parameter);
void writeBufferToSD();
void setLed(bool state, uint8_t ledPin, bool TEST) ;

const String BRANCH="esp-now"; // Branch name
const String RELEASE="2.2.0"; // Branch name

// Buffer für SD-Karten Schreibzugriffe
static const int BUFFER_SIZE = 4096;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

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

extern const char* CSV_HEADER;
const char* firstline = CSV_HEADER;  // Verwende CSV_HEADER als firstline

// Konstanten für Task-Konfiguration
static const uint32_t NAVIGATION_STACK_SIZE = 16384;

static const uint32_t TASK_DELAY_MS = 100;
static const size_t GPS_BUFFER_SIZE = 256;  // Vergrößert und aligned
alignas(4) static char gpsBuffer[GPS_BUFFER_SIZE];  // Aligned Buffer
static const size_t SMALL_BUFFER_SIZE = 15;
static const size_t TINY_BUFFER_SIZE = 10;

// GPS Status Struktur
struct GPSState {
    char gpstime[TINY_BUFFER_SIZE];
    char date[TINY_BUFFER_SIZE];
    char lat[SMALL_BUFFER_SIZE];
    char lon[SMALL_BUFFER_SIZE];
    char speed[TINY_BUFFER_SIZE];
    char altitude[TINY_BUFFER_SIZE];
    char hdop[TINY_BUFFER_SIZE];
    char satellites[TINY_BUFFER_SIZE];
    double lastLat;
    double lastLon;
    double distance;
} gpsState = {0};

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(1); // Initialisierung von gpsSerial

// Struktur für die Daten, die gesendet werden sollen
typedef struct struct_message {
    int rpm;
    double latitude;
    double longitude;
} struct_message;

struct_message myData;

// MAC-Adresse des Empfängers (GprsSender)
uint8_t broadcastAddress[] = {0x34, 0x98, 0x7A, 0x86, 0xF5, 0xCC}; // Hier die MAC Adresse des Empfänger ESP32 eintragen

// ESP-NOW Status
esp_now_peer_info_t peerInfo;

// Mutex Helper Funktionen
bool initializeMutexes() {
    for(int i = 0; i < 4; i++) {
        mutexes[i] = xSemaphoreCreateMutex();
        if(!mutexes[i]) {
            Serial.printf("Fehler bei Mutex %d Initialisierung\n", i);
            return false;
        }
    }
    return true;
}

bool takeMutex(MutexIndex index, TickType_t timeout = pdMS_TO_TICKS(100)) {
    if(mutexes[index] == NULL) return false;
    return xSemaphoreTake(mutexes[index], timeout) == pdTRUE;
}

void giveMutex(MutexIndex index) {
    if(mutexes[index] != NULL) {
        xSemaphoreGive(mutexes[index]);
    }
}

// LED Funktion vereinfacht
void setLed(bool state, uint8_t pin, bool TEST = true) {
    if (!TEST) return;

    if (takeMutex(LED_MUTEX, pdMS_TO_TICKS(100))) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, state);
        giveMutex(LED_MUTEX);
    }
}

bool initSDCard() {
    // SPI Bus explizit konfigurieren
    SPIClass spi = SPIClass(VSPI);
    spi.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

    // In setup() nach SPI.begin():
    Serial.println("SPI Pins:");
    Serial.printf("MISO: %d\n", SD_MISO);
    Serial.printf("MOSI: %d\n", SD_MOSI);
    Serial.printf("SCK:  %d\n", SD_SCLK);
    Serial.printf("CS:   %d\n", SD_CS);

    // SD-Karte mit mehreren Versuchen initialisieren
    int retries = 3;
    while (retries > 0) {
        if (SD.begin(SD_CS, spi)) {
            uint8_t cardType = SD.cardType();
            if (cardType != CARD_NONE) {
                Serial.println("SD Card Mount erfolgreich");
                Serial.printf("SD Card Typ: %d\n", cardType);
                return true;
            }
        }
        retries--;
        if (retries > 0) {
            Serial.printf("SD-Karten Initialisierung fehlgeschlagen, Versuch %d von 3...\n", 4-retries);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    return false;
}

bool initGPS() {
    Serial.println("Initializing GPS Serial...");
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

    // Erweiterte Initialisierungsversuche
    int gpsRetries = 5;
    while (gpsRetries > 0) {
        if (gpsSerial) {
            // Puffer leeren
            while(gpsSerial.available()) {
                gpsSerial.read();
            }

            // Warte auf erste gültige NMEA-Daten
            unsigned long startTime = millis();
            while(millis() - startTime < 2000) {  // 2 Sekunden Timeout
                if(gpsSerial.available()) {
                    char c = gpsSerial.read();
                    if(gps.encode(c) && gps.location.isValid()) {
                        Serial.println("GPS data stream verified");
                        return true;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        Serial.printf("Waiting for GPS Serial, attempt %d of 5...\n", 6-gpsRetries);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpsRetries--;
    }

    return false;
}

void bufferData(const char* data) {
    size_t len = strlen(data);
    if(bufferIndex + len >= BUFFER_SIZE) {
        // Buffer ist voll - erst schreiben
        writeBufferToSD();
    }

    // Sicheres Kopieren
    memcpy(buffer + bufferIndex, data, len);
    bufferIndex += len;
}

// Modifizierte writeBufferToSD Funktion:
void writeBufferToSD() {
    if (bufferIndex == 0) {
#if DEBUG_SD
        if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            Serial.println("DEBUG: Nichts zu schreiben (Buffer leer)");
            xSemaphoreGive(serialMutex);
        }
#endif
        return;
    }

    if(xSemaphoreTake(sdMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
#if DEBUG_SD
        if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            Serial.printf("DEBUG: Versuche %d Bytes zu schreiben\n", bufferIndex);
            xSemaphoreGive(serialMutex);
        }
#endif

        String fullPath = "/GPS/";

        if(!SD.exists("/GPS")) {
#if DEBUG_SD
            if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
                Serial.println("DEBUG: Erstelle GPS-Verzeichnis");
                xSemaphoreGive(serialMutex);
            }
#endif

            if(!SD.mkdir("/GPS")) {
#if DEBUG_SD
                if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
                    Serial.println("DEBUG: Fehler beim Erstellen des GPS-Verzeichnisses");
                    xSemaphoreGive(serialMutex);
                }
#endif
                xSemaphoreGive(sdMutex);
                return;
            }
        }

        fullPath += generateFileName(gps);

#if DEBUG_SD
        if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            Serial.printf("DEBUG: Öffne Datei: %s\n", fullPath.c_str());
            xSemaphoreGive(serialMutex);
        }
#endif

        File file = SD.open(fullPath.c_str(), FILE_APPEND);
        if(!file) {
#if DEBUG_SD
            if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
                Serial.println("DEBUG: Fehler beim Öffnen der Datei");
                xSemaphoreGive(serialMutex);
            }
#endif
            xSemaphoreGive(sdMutex);
            return;
        }

        size_t bytesWritten = file.write((uint8_t*)buffer, bufferIndex);

#if DEBUG_SD
        if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
            Serial.printf("DEBUG: Buffer Status - Geschrieben: %d von %d Bytes\n",
                        bytesWritten, bufferIndex);
            Serial.printf("DEBUG: Erste 32 Bytes: ");
            for(int i = 0; i < min(32, bufferIndex); i++) {
                Serial.printf("%c", buffer[i]);
            }
            Serial.println();
            xSemaphoreGive(serialMutex);
        }
#endif

        file.flush();
        file.close();
        xSemaphoreGive(sdMutex);

        // Buffer zurücksetzen
        bufferIndex = 0;
    }
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

    char tempBuffer[132];  // 130 (logging size) + 1 (\n) + 1 (\0)
    snprintf(tempBuffer, sizeof(tempBuffer), "%s\n", logging);
    bufferData(tempBuffer);

    // Aktuelle Position als letzte Position speichern
    snprintf(gpstimeLast, sizeof(gpstimeLast), "%s", gpstime);
    snprintf(dateLast, sizeof(dateLast), "%s", date);
    snprintf(latLast, sizeof(latLast), "%s", lat);
    snprintf(lonLast, sizeof(lonLast), "%s", lon);
    snprintf(speedLast, sizeof(speedLast), "%s", speed);
    snprintf(altitudeLast, sizeof(altitudeLast), "%s", altitude);
    snprintf(hdopLast, sizeof(hdopLast), "%s", hdop);
    snprintf(satellitesLast, sizeof(satellitesLast), "%s", satellites);
    snprintf(loggingLast, sizeof(loggingLast), "%s", logging);
}

void navigation(void * parameter) {
    esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
    esp_task_wdt_add(NULL);

    for(;;) {
        String rawData = "";

        // GPS-Daten lesen
        if(takeMutex(SERIAL_MUTEX, pdMS_TO_TICKS(100))) {
            while (gpsSerial.available() > 0 && rawData.length() < GPS_BUFFER_SIZE - 1) {
                char c = gpsSerial.read();
                rawData += c;
                gps.encode(c);
            }
            giveMutex(SERIAL_MUTEX);
        }

        // Wenn neue GPS-Position verfügbar
        if (gps.location.isUpdated()) {
            processPosition();  // GPS-Daten verarbeiten

            // Daten für den Versand vorbereiten (Beispielwerte, ersetzen Sie diese durch Ihre tatsächlichen Werte)
            int rpmValue = 1234; // Beispielwert
            double latitudeValue = gps.location.lat();
            double longitudeValue = gps.location.lng();

            // Daten per ESP-NOW senden
            sendDataViaESPNow(rpmValue, latitudeValue, longitudeValue);

            // LED-Feedback für neue Position
            setLed(true, GREEN_LED_PIN, TEST);
            vTaskDelay(pdMS_TO_TICKS(150));
            setLed(false, GREEN_LED_PIN, TEST);
        }

        // Watchdog zurücksetzen
        if(takeMutex(WATCHDOG_MUTEX, pdMS_TO_TICKS(100))) {
            esp_task_wdt_reset();
            giveMutex(WATCHDOG_MUTEX);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void testSDCard() {
    Serial.println("testSDCard: Starte SD-Karten Test");
    if(xSemaphoreTake(sdMutex, pdMS_TO_TICKS(15000))) {
        Serial.println("testSDCard: sdMutex genommen");
        String testFile = "/GPS/test.txt";
        Serial.printf("testSDCard: Öffne Datei %s zum Schreiben\n", testFile.c_str());
        File file = SD.open(testFile.c_str(), FILE_WRITE);
        if(file) {
            Serial.println("testSDCard: Datei geöffnet");
            Serial.println("testSDCard: Schreibe Daten in die Datei");
            file.println("SD Test " + String(millis()));
            Serial.println("testSDCard: Schliesse Datei");
            file.close();
            Serial.println("testSDCard: Datei erfolgreich geschrieben");

            // Lese Test
            Serial.printf("testSDCard: Öffne Datei %s zum Lesen\n", testFile.c_str());
            file = SD.open(testFile.c_str());
            if(file) {
                Serial.println("testSDCard: Datei zum Lesen geöffnet");
                Serial.println("testSDCard: Dateiinhalt:");
                while(file.available()) {
                    Serial.write(file.read());
                }
                Serial.println("testSDCard: Schliesse Datei");
                file.close();
            } else {
                Serial.println("testSDCard: Fehler beim Öffnen der Test-Datei zum Lesen");
            }
        } else {
            Serial.println("testSDCard: Fehler beim Schreiben der Test-Datei");
        }
        xSemaphoreGive(sdMutex);
        Serial.println("testSDCard: sdMutex freigegeben");
    } else {
        Serial.println("testSDCard: sdMutex konnte nicht genommen werden");
    }
    Serial.println("testSDCard: SD-Karten Test abgeschlossen");
}

void setup() {
    Serial.begin(SERIALMONITOR_BAUD);
    delay(1000);
    Serial.println("Starting setup...");

    // Mutexe initialisieren
    if (!initializeMutexes()) {
        Serial.println("FEHLER: Mutex-Initialisierung fehlgeschlagen");
        while(1) {
            digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
            delay(100);
        }
    }
    Serial.println("Mutexe erfolgreich erstellt");

    // SD-Karte initialisieren
    bool sdCardInitialized = initSDCard();
    if (!sdCardInitialized) {
        Serial.println("SD-Karten-Initialisierung fehlgeschlagen. Das Programm wird ohne SD-Karten-Funktionalität fortgesetzt.");
    } else {
        Serial.println("SD-Karte erfolgreich initialisiert");
        delay(100);
        // Testen der SD-Karte
        testSDCard();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // WLAN initialisieren
    WiFi.mode(WIFI_STA);

    // ESP-NOW initialisieren
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // ESP-NOW Rolle festlegen
    esp_now_register_send_cb(OnDataSent);

    // Peer-Informationen definieren
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Peer registrieren
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Tasks mit angepassten Prioritäten erstellen
    BaseType_t result = xTaskCreatePinnedToCore(
        navigation,
        "Navigation",
        16384,  // Stack-Größe erhöht
        NULL,
        2,  // Mittlere Priorität
        NULL,
        1
    );

    if (result != pdPASS) {
        Serial.println("Navigation Task Erstellung fehlgeschlagen");
        while(1) delay(1000);
    }

    // Watchdog konfigurieren
    esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, false);  // Watchdog nicht tödlich
}

void loop() {
  // Nichts zu tun
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void sendDataViaESPNow(int rpmValue, double latitudeValue, double longitudeValue) {
    // Daten für den Versand vorbereiten
    myData.rpm = rpmValue;
    myData.latitude = latitudeValue;
    myData.longitude = longitudeValue;

    // Daten per ESP-NOW senden
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    if (result != ESP_OK) {
        Serial.print("Error sending the data");
        Serial.println(esp_err_to_name(result));
    }
}