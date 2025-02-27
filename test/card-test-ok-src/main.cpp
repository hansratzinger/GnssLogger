#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// Definiere den Chip Select (CS) Pin für die SD-Karte
const int chipSelect = 5; // Ändere dies, falls dein CS-Pin anders ist

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Warte, bis die serielle Verbindung bereit ist
  }

  Serial.println("Starte SD-Karten Test...");

  // SD-Karte initialisieren
  Serial.print("Initialisiere SD-Karte... ");
  if (!SD.begin(chipSelect)) {
    Serial.println("Initialisierung fehlgeschlagen!");
    while (1); // Bei Fehler in einer Endlosschleife anhalten
  }
  Serial.println("Initialisierung erfolgreich.");

  // Verzeichnis erstellen
  Serial.print("Erstelle Verzeichnis '/test2'... ");
  if (!SD.mkdir("/test2")) {
    Serial.println("Verzeichnis konnte nicht erstellt werden");
  } else {
    Serial.println("Verzeichnis erfolgreich erstellt.");
  }

  // Datei erstellen und beschreiben
  Serial.print("Erstelle und beschreibe Datei '/test2/test2.txt'... ");
  File file = SD.open("/test2/test2.txt", FILE_WRITE);
  if (file) {
    file.println("Hallo Welt! Dies ist ein Test2.");
    file.close();
    Serial.println("Datei erfolgreich beschrieben und geschlossen.");
  } else {
    Serial.println("Fehler beim Öffnen der Datei!");
  }

  Serial.println("SD-Karten Test abgeschlossen.");
}

void loop() {
  // Hier kann weiterer Code stehen, aber für diesen Test ist er nicht notwendig
}