#ifndef SLEEP_H
#define SLEEP_H

#include <Arduino.h>
#include <deque>
#include "my_Helpers.h"

void enableLightSleep(unsigned long seconds);
void enableDeepSleep(unsigned long seconds);
void enableModemSleep();
void disableModemSleep();
void saveStationPositionsToRTC(const std::deque<std::pair<double, double>>& stationPositions);
void loadStationPositionsFromRTC(std::deque<std::pair<double, double>>& stationPositions);

extern const int RED_LED_PIN; // in main.cpp definiert für rote LED
extern const int GREEN_LED_PIN; // in main.cpp definiert für grüne LED

// Definition der RTC-Variablen

extern RTC_DATA_ATTR bool isMissionModeRTC;

#endif // SLEEP_H
