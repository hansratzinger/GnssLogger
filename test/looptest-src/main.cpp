#include <Arduino.h>
#include <TinyGPSPlus.h>

/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It assumes that you have a serial GPS device hooked up on pins 16(rx) and 17(tx).
*/
static const int RXPin = 16, TXPin = 17; // Beispielpins für Serial2
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial gpsSerial(1); // Use UART1, adjust if needed

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void setup()
{
  delay(10000);
  Serial.begin(115200);
  Serial.println("Starting setup..."); // Hinzugefügt
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin); // Initialisierung von gpsSerial
  Serial.println("gpsSerial.begin() aufgerufen"); // Hinzugefügt
  delay(5000); // Warte 5 Sekunden, damit der GPS-Modul sich initialisieren kann
  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  delay(1000);
  Serial.println("Setup abgeschlossen"); // Hinzugefügt
}

void loop()
{
  Serial.println("Loop gestartet"); // Hinzugefügt
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read()))
      displayInfo();
    }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}