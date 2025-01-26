#include "Sleep.h"
#include "GNSS_module.h" // Einbinden der GNSS-Modul-Header-Datei
#include <Arduino.h>     // Einbinden der Arduino-Header-Datei für String-Typ
#include "Morse_LED.h"   // Einbinden der Morse_LED-Header-Datei
#include <deque>

extern const bool TEST; // in main.cpp definiert für Testzwecke
extern const int RED_LED_PIN;
extern const int GREEN_LED_PIN;
extern RTC_DATA_ATTR std::deque<std::pair<double, double>> stationPositionsRTC;


// Funktion zur Aktivierung des Light-Sleep-Modus
void enableLightSleep(unsigned long seconds)
{
  blinkMorseCode("A", RED_LED_PIN, 1,TEST);
  debugPrintln("Light-Sleep-Modus aktiviert");
  delay(100);                                       // Warte 100 Millisekunden
  esp_sleep_enable_timer_wakeup(seconds * 1000000); // Zeit in Mikrosekunden
  esp_light_sleep_start();
}

// Funktion zur Aktivierung des Deep-Sleep-Modus
void enableDeepSleep(unsigned long seconds)
{
  blinkMorseCode("J", RED_LED_PIN, 1,TEST);
  debugPrintln("Deep-Sleep-Modus aktiviert");
  delay(100);                                       // Warte 100 Millisekunden
  esp_sleep_enable_timer_wakeup(seconds * 1000000); // Zeit in Mikrosekunden
  esp_deep_sleep_start();
}

// Funktion zur Aktivierung des Modem-Sleep-Modus
void enableModemSleep()
{
  blinkMorseCode("M", RED_LED_PIN, 1,TEST);
  WiFi.mode(WIFI_OFF);                // Deaktivieren des WiFi-Moduls
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // Aktivieren des Modem-Sleep-Modus
  debugPrintln("Modem-Sleep-Modus aktiviert");
}

// Funktion zur Deaktivierung des Modem-Sleep-Modus
void disableModemSleep()
{
  blinkMorseCode("M", GREEN_LED_PIN, 1,TEST);
  esp_wifi_set_ps(WIFI_PS_NONE); // Deaktivieren des Modem-Sleep-Modus
  WiFi.mode(WIFI_STA);           // Aktivieren des WiFi-Moduls
  debugPrintln("Modem-Sleep-Modus deaktiviert");
}

// Funktion zum Speichern der Station-Positionen im RTC-Speicher
void saveStationPositionsToRTC(const std::deque<std::pair<double, double>> &stationPositions)
{
  stationPositionsRTC = stationPositions;
}

// Funktion zum Laden der Station-Positionen aus dem RTC-Speicher
void loadStationPositionsFromRTC(std::deque<std::pair<double, double>> &stationPositions)
{
  stationPositions = stationPositionsRTC;
}

