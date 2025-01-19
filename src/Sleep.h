#ifndef SLEEP_H
#define SLEEP_H

void enableLightSleep(unsigned long seconds);
void enableDeepSleep(unsigned long seconds);
void enableModemSleep();
void disableModemSleep();

#endif // SLEEP_H
