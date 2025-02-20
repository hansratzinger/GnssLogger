#include <Arduino.h>
#include <HardwareSerial.h>
#include "DEV_Config.h"
#include "L76X.h"

unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 5000; // Überprüfe alle 5 Sekunden

GNRMC GPS1;
Coordinates B_GPS;
char buff_G[800] = {0};

// Funktion zum Senden eines Testbefehls und Überprüfen der Antwort
bool checkSerialConnection() {
  Serial2.print("$PMTK001*2E\r\n"); // Beispiel: Anfrage der Softwareversion
  delay(100);
  if (Serial2.available() > 0) {
    // Daten empfangen, Verbindung ist aktiv
    return true;
  } else {
    // Keine Daten empfangen, Verbindung ist möglicherweise unterbrochen
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  DEV_Set_Baudrate(115200);
  delay(5000);
  Serial.println("Start GNSS");

  // Warmstart durchführen (optional)
  char warmStartCommand[] = "$PMTK225,1*2A\r\n";
  L76X_Send_Command(warmStartCommand);
  delay(1000);
}

void loop() {
  // Periodische Überprüfung der seriellen Verbindung
  if (millis() - lastCheckTime > checkInterval) {
    if (!checkSerialConnection()) {
      Serial.println("Serielle Verbindung unterbrochen. Neustart...");
      DEV_Set_Baudrate(115200); // Serielle Schnittstelle neu initialisieren
      delay(1000);
      lastCheckTime = millis(); // Setze die letzte Checkzeit zurück
      // return; // Loop verlassen, um von vorne zu beginnen - ENTFERNT!
    }
    lastCheckTime = millis(); // Setze die letzte Checkzeit zurück
  }

  // Daten vom GNSS-Modul empfangen
  if (Serial2.available() > 0) {
    int bytesRead = Serial2.readBytesUntil('\n', buff_G, 799); // Bis zu 799 Zeichen oder bis zum Zeilenumbruch lesen
    buff_G[bytesRead] = '\0'; // String terminieren
    Serial.print(buff_G); // Ausgabe des empfangenen Strings
  }

  GPS1 = L76X_Gat_GNRMC();
  Serial.print("\r\n");
  Serial.print("Time:");
  Serial.print(GPS1.Time_H);
  Serial.print(":");
  Serial.print(GPS1.Time_M);
  Serial.print(":");
  Serial.print(GPS1.Time_S);
  Serial.print("\r\n");
  Serial.print("Lat:");
  Serial.print(GPS1.Lat, 7);
  Serial.print("\nLon:");
  Serial.print(GPS1.Lon, 7);
  Serial.print("\r\n");
  B_GPS = L76X_Baidu_Coordinates();
  Serial.print("\r\n");
  Serial.print("B_Lat:");
  Serial.print(B_GPS.Lat, 7);
  Serial.print("\nB_Lon:");
  Serial.print(B_GPS.Lon, 7);
  Serial.print("\r\n");
  delay(1000); // Kleine Verzögerung hinzufügen
}