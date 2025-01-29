#ifndef MORSE_LED_H
#define MORSE_LED_H

#include <Arduino.h>
#include "pins.h"

void blinkMorseCode(const String &text, int ledPin, int repeatCount, bool TEST) ;
void ledON(int ledPin,bool TEST) ;
void ledOFF(int ledPin,bool TEST) ; 
void ledMode(bool mode,bool TEST) ; // mode = true -> GREEN_LED_PIN ON, RED_LED_PIN OFF; mode = false -> GREEN_LED_PIN OFF, RED_LED_PIN ON

#endif // MORSE_LED_H
