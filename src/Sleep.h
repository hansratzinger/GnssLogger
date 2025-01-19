#ifndef SLEEP_H
#define SLEEP_H

void enableLightSleep(unsigned long seconds);
void enableDeepSleep(unsigned long seconds);
void enableModemSleep();
void disableModemSleep();

extern const int RED_LED_PIN; // in main.cpp definiert für rote LED
extern const int GREEN_LED_PIN; // in main.cpp definiert für grüne LED

#endif // SLEEP_H
