#ifndef MY_HELPERS_H
#define MY_HELPERS_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// LED Funktionen
void ledON(int ledPin, bool TEST);
void ledOFF(int ledPin, bool TEST);
void ledMode(bool mode, bool TEST);


/**
 * @brief LED ein- oder ausschalten mit Mutex-Schutz
 * @param state true für ein, false für aus
 * @param ledPin GPIO Pin der LED
 * @param TEST Wenn true, wird die Funktion ausgeführt
 */
void setLed(bool state, uint8_t ledPin, bool TEST);

/**
 * @brief LED einschalten
 * @param ledPin PIN-Nummer der LED
 * @param TEST Wenn true, wird die Funktion ausgeführt
 */
void ledON(int ledPin, bool TEST);

/**
 * @brief LED ausschalten
 * @param ledPin PIN-Nummer der LED
 * @param TEST Wenn true, wird die Funktion ausgeführt
 */
void ledOFF(int ledPin, bool TEST);

/**
 * @brief LED-Modus setzen
 * @param mode true für an, false für aus
 * @param TEST Wenn true, wird die Funktion ausgeführt
 */
void ledMode(bool mode, bool TEST);

/**
 * @brief Verzögerung mit Test-Flag
 * @param mydelayTimeMillis Verzögerungszeit in Millisekunden
 * @param TEST Wenn true, wird die Verzögerung ausgeführt
 */
void mydelay(unsigned long mydelayTimeMillis, bool TEST);

/**
 * @brief Text anhängen
 * @param text Basis-Text
 * @param toappendText Anzuhängender Text
 */
// void appendText(char* text, const char* toappendText);

// ...existing code...

#endif // MY_HELPERS_H