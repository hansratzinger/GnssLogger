#ifndef MY_HELPERS_H
#define MY_HELPERS_H

#include <Arduino.h>
#include "pins.h"

void blinkMorseCode(const String &text, int ledPin, int repeatCount, bool TEST) ;
void ledON(int ledPin,bool TEST) ;
void ledOFF(int ledPin,bool TEST) ; 
void ledMode(bool mode,bool TEST) ; // mode = true -> GREEN_LED_PIN ON, RED_LED_PIN OFF; mode = false -> GREEN_LED_PIN OFF, RED_LED_PIN ON
void mydelay(unsigned long mydelayTimeMillis, bool TEST) ; // delayTime in msec, TEST = true -> delay, TEST = false -> no delay
void appendText(char* text, const char* toappendText); // Funktion zum Anhängen von Text an einen vorhandenen Text

#endif // MY_HELPERS_H
