#include "Sleep.h"
#include <esp_sleep.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "GNSS_module.h" // Einbinden der GNSS-Modul-Header-Datei
#include "Morse_LED.h" // Einbinden der Morse_LED-Header-Datei

extern const bool TEST;
extern const int RED_LED_PIN;
extern const int GREEN_LED_PIN;

// Funktion zur Aktivierung des Light-Sleep-Modus
void enableLightSleep(unsigned long seconds) {
  if (TEST) {
    blinkMorseCode("A", RED_LED_PIN, 1);
  }
  debugPrintln("Light-Sleep-Modus aktiviert");
  delay(100); // Warte 100 Millisekunden
  esp_sleep_enable_timer_wakeup(seconds * 1000000); // Zeit in Mikrosekunden
  esp_light_sleep_start();
}

// Funktion zur Aktivierung des Deep-Sleep-Modus
void enableDeepSleep(unsigned long seconds) {
  if (TEST) {
    blinkMorseCode("J", RED_LED_PIN, 1);
  }
  debugPrintln("Deep-Sleep-Modus aktiviert");
  delay(100); // Warte 100 Millisekunden
  esp_sleep_enable_timer_wakeup(seconds * 1000000); // Zeit in Mikrosekunden
  esp_deep_sleep_start();
}

// Funktion zur Aktivierung des Modem-Sleep-Modus
void enableModemSleep() {
  if (TEST) {
    blinkMorseCode("M", RED_LED_PIN, 1);
  }
  WiFi.mode(WIFI_OFF); // Deaktivieren des WiFi-Moduls
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // Aktivieren des Modem-Sleep-Modus
  debugPrintln("Modem-Sleep-Modus aktiviert");
}

// Funktion zur Deaktivierung des Modem-Sleep-Modus
void disableModemSleep() {
  if (TEST) {
    blinkMorseCode("M", GREEN_LED_PIN, 1);
  }
  esp_wifi_set_ps(WIFI_PS_NONE); // Deaktivieren des Modem-Sleep-Modus
  WiFi.mode(WIFI_STA); // Aktivieren des WiFi-Moduls
  debugPrintln("Modem-Sleep-Modus deaktiviert");
}