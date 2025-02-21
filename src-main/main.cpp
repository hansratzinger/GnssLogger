#include <Arduino.h>
#include <pins.h>
#define GPS_BAUD 115200
#define SERIALMONITOR_BAUD 115200

#include "GNSS_module.h"
#include "SD_MMC.h"
#include <SPI.h>
#include <FS.h>
#include <esp_now.h> // ESP-NOW Bibliothek
#include <WiFi.h>    // WiFi Bibliothek
#include <TinyGPSPlus.h>

#define DEBUG_SD 1  // Debug-Ausgaben aktivieren

// const String BRANCH = "esp-now"; // Branch name
// const String RELEASE = "2.2.0"; // Branch name

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

// Globale Variable für die SD-Karten-Datei
File gpsFile;
String currentFileName;

extern const char* CSV_HEADER;
const char* firstline = CSV_HEADER;  // Verwende CSV_HEADER als firstline

static const size_t SMALL_BUFFER_SIZE = 15;
static const size_t TINY_BUFFER_SIZE = 10;

char getDirectionOfLat(double lat);
char getDirectionOfLng(double lng);
bool initSDCard();

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

// LED Funktion vereinfacht
void setLed(bool state, uint8_t pin, bool TEST = true) {
    if (!TEST) return;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, state);
}

bool initGPS() {
    Serial.println("Initializing GPS Serial...");
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

    // Erweiterte Initialisierungsversuche
    int gpsRetries = 5;
    while (gpsRetries > 0) {
        if (gpsSerial) {
            // Puffer leeren
            while (gpsSerial.available()) {
                gpsSerial.read();
            }

            // Warte auf erste gültige NMEA-Daten
            unsigned long startTime = millis();
            while (millis() - startTime < 2000) {  // 2 Sekunden Timeout
                if (gpsSerial.available()) {
                    char c = gpsSerial.read();
                    if (gps.encode(c) && gps.location.isValid()) {
                        Serial.println("GPS data stream verified");
                        return true;
                    }
                }
                delay(10);
            }
        }

        Serial.printf("Waiting for GPS Serial, attempt %d of 5...\n", 6 - gpsRetries);
        delay(1000);
        gpsRetries--;
    }

    return false;
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
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));

    if (result != ESP_OK) {
        Serial.print("Error sending the data");
        Serial.println(esp_err_to_name(result));
    }
}

// Funktion zum Öffnen der SD-Karten-Datei
bool openGPSFile() {
    String fullPath = "/gps/";
    if (!SD_MMC.exists("/gps")) {
        if (!SD_MMC.mkdir("/gps")) {
            Serial.println("Fehler beim Erstellen des GPS-Verzeichnisses");
            return false;
        }
    }
    fullPath += generateFileName(gps);

    // Überprüfen, ob sich der Dateiname geändert hat
    if (fullPath != currentFileName) {
        // Wenn sich der Dateiname geändert hat, die alte Datei schließen
        if (gpsFile) {
            gpsFile.close();
        }
        currentFileName = fullPath;
    }

    // Wenn die Datei noch nicht geöffnet ist, öffnen Sie sie jetzt
    if (!gpsFile) {
        // SPI Transaktion starten
        SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
        gpsFile = SD_MMC.open(currentFileName.c_str(), FILE_APPEND);
        if (!gpsFile) {
            Serial.println("Fehler beim Öffnen der Datei");
            SPI.endTransaction(); // SPI Transaktion beenden
            return false;
        }
        SPI.endTransaction(); // SPI Transaktion beenden
    }
    return true;
}

// Funktion zum sicheren Konvertieren von String nach Double
double safeStrtod(const char* str, double defaultValue) {
    char* endPtr;
    double result = strtod(str, &endPtr);
    if (*endPtr != '\0') {
        Serial.println("Konvertierungsfehler: Ungültige Eingabe");
        return defaultValue;
    }
    return result;
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
    double currentLat = safeStrtod(lat, 0.0);
    double currentLon = safeStrtod(lon, 0.0);
    double lastLat = safeStrtod(latLast, 0.0);
    double lastLon = safeStrtod(lonLast, 0.0);
    

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

    // Auf SD-Karte schreiben
    Serial.println("Writing to SD card...");
    if (openGPSFile()) {
    
        // Interrupts deaktivieren
        noInterrupts();
    
        if (openGPSFile()) {
            gpsFile.println(logging);
        } else {
            Serial.println("Fehler beim Schreiben in die Datei");
        }
        
        // Interrupts aktivieren
        interrupts();
    }

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

    Serial.println("processPosition() finished");
}

void setup() {
    Serial.begin(SERIALMONITOR_BAUD);
    delay(1000);
    Serial.println("Starting setup...");

    // SD-Karte initialisieren
    Serial.println("Initializing SD card...");
    bool sdCardInitialized = initSDCard();
    if (!sdCardInitialized) {
        Serial.println("SD-Karten-Initialisierung fehlgeschlagen. Das Programm wird ohne SD-Karten-Funktionalität fortgesetzt.");
    } else {
        Serial.println("SD-Karte erfolgreich initialisiert");
        delay(100);
    }

    Serial.println("After SD card initialization...");

    // GPS initialisieren
    Serial.println("Initializing GPS...");
    bool gpsInitialized = initGPS();
    if (!gpsInitialized) {
        Serial.println("GPS-Initialisierung fehlgeschlagen. Das Programm wird ohne GPS-Funktionalität fortgesetzt.");
    } else {
        Serial.println("GPS erfolgreich initialisiert");
        delay(100);
    }

    // WLAN initialisieren
    Serial.println("Initializing WiFi...");
    WiFi.mode(WIFI_STA);

    // ESP-NOW initialisieren
    Serial.println("Initializing ESP-NOW...");
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // ESP-NOW Rolle festlegen
    Serial.println("Setting ESP-NOW role...");
    esp_now_register_send_cb(OnDataSent);

    // Peer-Informationen definieren
    Serial.println("Defining peer info...");
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Peer registrieren
    Serial.println("Adding peer...");
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    delay(5000);

    Serial.println("Setup finished!");
    delay(5000);
}

void loop() {
    currentTime = millis();
    static unsigned long lastPositionTime = 0;
    static const unsigned long switchTime = 250; // Wartezeit von mindestens 0,25 Sekunde

    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if ((TEST) && (gps.location.isValid())) {
        Serial.print(F("Latitude: "));
        Serial.println(gps.location.lat(), 6);
        Serial.print(F("Longitude: "));
        Serial.println(gps.location.lng(), 6);
        Serial.print(F("Altitude: "));
        Serial.println(gps.altitude.meters());
        Serial.print(F("Speed: "));
        Serial.println(gps.speed.kmph());
        Serial.print(F("Heading: "));
        Serial.println(gps.course.deg());
        Serial.print(F("Date: "));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.println(gps.date.year());
        Serial.print(F("Time: "));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
        Serial.println();
    }

    if (currentTime - lastPositionTime >= switchTime) { // Wartezeit von mindestens 0,25 Sekunde
        lastPositionTime = currentTime;
        if ((gps.location.isValid()) && (gps.hdop.hdop() < 3.0) && (gps.date.year()) != 2000 && (gps.date.month()) != 0 && (gps.date.day()) != 0 && (gps.time.hour()) != 0 && (gps.time.minute()) != 0 && (gps.time.second()) != 0) {
            // Überprüfung ob die Position aktualisiert wurde und der HDOP-Wert unter dem Schwellenwert liegt
            // Aufrufen der Funktion zur Verarbeitung und Speicherung der Positionsdaten
            processPosition();
            int rpmValue = 1200; // Beispielwert
            double latitudeValue = gps.location.lat();
            double longitudeValue = gps.location.lng();

            // Daten per ESP-NOW senden
            sendDataViaESPNow(rpmValue, latitudeValue, longitudeValue);
            setLed(true, GREEN_LED_PIN, TEST);
            delay(150);
            setLed(false, GREEN_LED_PIN, TEST);
        }
    }
}