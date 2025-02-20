#include <SD_MMC.h>

void setup() {
  Serial.begin(115200);

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

  // Asynchrones Schreiben starten (Beispiel)
  // ...
}

void loop() {
  // Andere Aufgaben hier erledigen, während die SD-Karte im Hintergrund arbeitet
  // ...
} 