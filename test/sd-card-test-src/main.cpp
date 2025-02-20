#include <Arduino.h>
#include <SD_MMC.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup...");

  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD Card Mount Failed");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return;
  }

  Serial.print("SD_MMC Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  Serial.println("Setup abgeschlossen!");
}

void loop() {
  Serial.println("Starting SD card test...");

  // Datei erstellen und beschreiben
  File file = SD_MMC.open("/sdcard/test2.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Error opening /sdcard/test2.txt");
    while (1); // Endlosschleife bei Fehler
  }

  Serial.println("Writing to /sdcard/test2.txt...");
  file.println("This is a test2!");
  file.close();
  Serial.println("Done writing to /sdcard/test2.txt.");

  Serial.println("Test abgeschlossen!");
  delay(5000); // Füge eine Verzögerung hinzu, um die Ausgabe zu sehen
}