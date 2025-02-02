#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17

TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // Initialisierung von gpsSerial
const float hdopTreshold = 1;
void setup() {
    Serial.begin(115200); // Serielle Schnittstelle für Debugging
    gpsSerial.begin(115200, SERIAL_8N1, RXD2, TXD2); // Serielle Schnittstelle für GNSS-Modul

    Serial.println("GPS Test gestartet...");
}

void loop() {
    // Überprüfen, ob Daten vom GNSS-Modul verfügbar sind
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        Serial.print(c); // Ausgabe der empfangenen Daten auf dem seriellen Monitor

        // GPS-Daten dekodieren
        gps.encode(c);
    }

    // Überprüfen, ob eine gültige Position empfangen wurde
    // if (gps.location.isUpdated()) {
    // if ((gps.location.isUpdated()) && (gps.hdop.hdop() < hdopTreshold) && (gps.date.year()) != 2000 && (gps.date.month()) != 0 && (gps.date.day()) != 0  && (gps.time.hour()) != 0 && (gps.time.minute()) != 0 && (gps.time.second()) != 0 ) {
    if ((gps.location.isUpdated())  && (gps.date.year()) != 2000 && (gps.date.month()) != 0 && (gps.date.day()) != 0  && (gps.time.hour()) != 0 && (gps.time.minute()) != 0 && (gps.time.second()) != 0 ) {
        Serial.print("Latitude: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(" Longitude: ");
        Serial.print(gps.location.lng(), 6);
        Serial.print(" HDOP: ");
        Serial.println(gps.hdop.hdop());
    }
}