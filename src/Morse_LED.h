#ifndef MORSE_LED_H
#define MORSE_LED_H

#include <Arduino.h>

void blinkMorseCode(const String &text, int ledPin, int repeatCount);

#endif // MORSE_LED_H
