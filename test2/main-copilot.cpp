void setup() {
  // Serial Monitor
  Serial.begin(115200);

  // WiFi und Bluetooth ausschalten
  WiFi.mode(WIFI_OFF);
  btStop();

  // Reduzieren der Clock-Rate auf 80 MHz
  setCpuFrequencyMhz(80);
  Serial.println("CPU frequency set to 80 MHz");

  // Initialisiere die SD-Karte
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // Überprüfen des Wakeup-Reasons
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: 
      debugPrintln("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1: 
      debugPrintln("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER: 
      debugPrintln("Wakeup caused by timer");
      isWakedUpFromDeepSleep = true;
      isWakedUpRTC = true; // Setze die RTC-Variable auf true
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: 
      debugPrintln("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP: 
      debugPrintln("Wakeup caused by ULP program");
      break;
    default: 
      debugPrintln("Wakeup was not caused by deep sleep");
      delay(7500);
      break;
  }

  // Load data from RTC memory only if waking up from deep sleep
  if (isWakedUpFromDeepSleep) {
    strcpy(gpstimeLast, rtcData.gpstimeLast);
    strcpy(dateLast, rtcData.dateLast);
    strcpy(latLast, rtcData.latLast);
    strcpy(lonLast, rtcData.lonLast);
    isMissionMode = rtcData.isMissionMode;
    timeDifference = rtcData.timeDifference;
    loadStationPositionsFromRTC(stationPositions);
  }

  // Debug-Ausgabe der geladenen Werte
  debugPrint("LatLast: ");
  debugPrint(rtcData.latLast);
  debugPrint(", LonLast: ");
  debugPrintln(rtcData.lonLast);

  // Initialisiere die LED-Pins
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  // Entfernen der FreeRTOS-Task-Erstellung
  // xTaskCreate(gpsTask, "GPSTask", 4096, NULL, 1, NULL);
}

void loop() {
  // Implementieren Sie die Hauptlogik hier
  // Beispiel: Aufrufen der GPS-Verarbeitungsfunktion
  processPosition();

  // Fügen Sie hier eine Verzögerung hinzu, um die Schleife zu verlangsamen
  delay(1000);
}