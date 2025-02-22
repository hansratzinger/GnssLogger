#ifndef PINS_H
#define PINS_H

/**
 * @file pins.h
 * @brief Pin-Definitionen für ESP32 WROOM-32 mit SD-Karte und LC76 GNSS Modul
 */
 
// SD-Karten Schnittstelle
#define SD_MISO     19    // SD Card: Master In Slave Out
#define SD_MOSI     23    // SD Card: Master Out Slave In  
#define SD_SCLK     18    // SD Card: Serial Clock
#define SD_CS       5     // SD Card: Chip Select
// LC76 GNSS Modul UART Schnittstelle
#define GPS_RX      16    // ESP32 RX (Input) <- LC76 TX (Output)
#define GPS_TX      17    // ESP32 TX (Output) -> LC76 RX (Input)

// Status LED 
#define GREEN_LED_PIN  25    // Grüne LED an GPIO25 (output-fähig)
#define RED_LED_PIN    26    // Rote LED an GPIO26 (output-fähig)

#define SD_CHECK(x) do { \
    int __err_rc = (x); \
    if (__err_rc != ESP_OK) { \
        Serial.printf("SD error: %d at %s:%d\n", __err_rc, __FILE__, __LINE__); \
        return false; \
    } \
} while(0)

#endif /* PINS_H */
