void loop() {
  ledMode(isMissionMode, TEST); // Grüne LED für Mission-Modus, Rote LED für Station-Modus

  // Read data from the GPS module
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  currentTime = millis();
  if (currentTime - lastPositionTime >= switchTime) { // Wartezeit von mindestens 0,25 Sekunde
    lastPositionTime = currentTime;

    if (gps.location.isUpdated() && gps.hdop.hdop() < hdopTreshold && gps.date.year() != 2000 && gps.date.month() != 0 && gps.date.day() != 0 && gps.time.hour() != 0 && gps.time.minute() != 0 && gps.time.second() != 0) {
      // Überprüfung ob die Position aktualisiert wurde und der HDOP-Wert unter dem Schwellenwert liegt
      // Aufrufen der Funktion zur Verarbeitung und Speicherung der Positionsdaten
      processPosition();

      // Berechne die Zeitdifferenz zwischen gpstime und gpstimeLast
      if (strlen(gpstimeLast) > 0) {
        timeDifference = getTimeDifference(gpstime, gpstimeLast);
        debugPrint("Time difference: " + String(timeDifference) + " seconds");
        debugPrintln("timeToLastPositionTreshold: " + String(timeToLastPositionTreshold) + " seconds");
      }

      if ((timeDifference > timeToLastPositionTreshold) || (strlen(gpstimeLast) == 0)) {
        // Überprüfe, ob die letzte Position lang zurückliegt, z.B. weil das GPS-Modul neu gestartet wurde
        // und die Zeitdifferenz größer als der Schwellenwert ist
        // Wenn true wird der Mission-Modus aktiviert und der Positionsspeicher geleert
        // neue Station-Positionen werden am Anfang der Liste hinzugefügt
        debugPrintln("Clear stationPositions due to time difference");
        isMissionMode = true;
        stationPositions.clear();
        stationPositions.push_back(std::make_pair(atof(lat), atof(lon)));
      }

      if (isMissionMode) {
        // Schreibe nur im Mission-Modus auf die SD-Karte
        if (strcmp(date, "2000/00/00") != 0) {
          // Schalte die LEDs entsprechend dem Modus
          ledMode(isMissionMode, TEST); // Grüne LED für Mission-Modus, Rote LED für Station-Modus
          // Schreibe die Daten in die Datei
          writeToCSV(logging);
        }

        if (millis() - lastSwitchTime >= switchInterval) {
          bool withinRange = false;
          for (const auto& pos : stationPositions) {
            if (isWithinRange(atof(lat), atof(lon), pos.first, pos.second, circleAroundPosition)) {
              withinRange = true;
              break;
            }
          }

          if (withinRange) {
            isMissionMode = false;
            ledMode(isMissionMode, TEST); // Grüne LED für Mission-Modus, Rote LED für Station-Modus
            lastSwitchTime = millis();
            debugPrintln("Switched to Station Mode");
          }
        }

        // Aktivieren des Light-Sleep-Modus im Mission-Modus
        enableLightSleep(sleepingTimeLightSleep);
        ledMode(isMissionMode, TEST); // Grüne LED für Mission-Modus, Rote LED für Station-Modus

      } else {  // Station-Modus
        // Überprüfen, ob die aktuelle Position außerhalb des doppelten Radius der stationPositions liegt
        bool outsideDoubleRadius = true;
        for (const auto& pos : stationPositions) {
          if (isWithinRange(atof(lat), atof(lon), pos.first, pos.second, 2 * circleAroundPosition)) {
            outsideDoubleRadius = false;
            break;
          }
        }

        if (outsideDoubleRadius) {
          isMissionMode = true;
          ledMode(isMissionMode, TEST); // Grüne LED für Mission-Modus, Rote LED für Station-Modus
          debugPrintln("Switched to Mission Mode due to position outside double radius");
        }

        // Aktivieren des Deep-Sleep-Modus im Station-Modus
        enableDeepSleep(sleepingTimeDeepSleep);
      }
    }
  }
}