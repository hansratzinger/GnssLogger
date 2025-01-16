#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPS++.h>
#include <EEPROM.h>
#include "GNSS_module.h"

// Funktion zur Aktivierung des ALP-Modus
void enableALPMode() {
  // Befehl zur Aktivierung des ALP-Modus
  const char* enableALPCommand = "$PMTK225,8*23\r\n";
  gpsSerial.print(enableALPCommand);
  Serial.println("ALP-Modus aktiviert");
}

// Funktion zur Deaktivierung des ALP-Modus
void disableALPMode() {
  // Befehl zur Deaktivierung des ALP-Modus
  const char* disableALPCommand = "$PMTK225,0*2B\r\n";
  gpsSerial.print(disableALPCommand);
  Serial.println("ALP-Modus deaktiviert");
}