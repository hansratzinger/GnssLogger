#ifndef MY_HELPERS_H
#define MY_HELPERS_H
#include <Arduino.h>
#include <map>
#include "my_Helpers.h"
#include "pins.h"


// Vorwärtsdeklarationen der Task-Funktionen
void setLed(bool state, uint8_t ledPin, bool TEST);

// Define the Morse code for each character
std::map<char, String> morseCode = {
  {'A', ".-"}, {'B', "-..."}, {'C', "-.-."}, {'D', "-.."}, {'E', "."},
  {'F', "..-."}, {'G', "--."}, {'H', "...."}, {'I', ".."}, {'J', ".---"},
  {'K', "-.-"}, {'L', ".-.."}, {'M', "--"}, {'N', "-."}, {'O', "---"},
  {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."}, {'S', "..."}, {'T', "-"},
  {'U', "..-"}, {'V', "...-"}, {'W', ".--"}, {'X', "-..-"}, {'Y', "-.--"},
  {'Z', "--.."}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"}, {'4', "....-"},
  {'5', "....."}, {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."},
  {'0', "-----"}, {' ', " "}
};

unsigned long previousMillis = 0; // Speichert den letzten Zeitpunkt
const long interval = 1000; // Intervallzeit in Millisekunden

// Funktion zum Blinken der LED im Morse-Code-Rhythmus
/**
 * @brief Blinks an LED to represent the given text in Morse code.
 * 
 * This function converts the input text to Morse code and blinks an LED
 * connected to the specified pin accordingly. The blinking pattern for each
 * character is repeated a specified number of times.
 * 
 * @param text The text to be converted to Morse code and blinked.
 * @param ledPin The pin number where the LED is connected.
 * @param repeatCount The number of times the entire text should be repeated.
 * 
 * @note The function will only execute if the global constant TEST is true.
 * 
 * The Morse code for each character is defined in the global map `morseCode`.
 * Each dot (.) is represented by a 200 ms HIGH signal, and each dash (-) is
 * represented by a 500 ms HIGH signal. There is a 200 ms LOW signal between
 * each dot and dash, a 500 ms LOW signal between each character, and a 1000 ms
 * LOW signal between each repetition of the text.
 */
void blinkMorseCode(const String &text, uint8_t ledPin, int repeatCount, bool TEST) {
  if (!TEST) {
    return;
  } 
  pinMode(ledPin, OUTPUT);

  for (int r = 0; r < repeatCount; r++) {
    for (char c : text) {
      c = toupper(c); // Konvertiere den Buchstaben in Großbuchstaben
      if (morseCode.find(c) != morseCode.end()) {
        String code = morseCode[c];
        for (char m : code) {
          if (m == '.') {
            setLed(true, ledPin, true);   // Grüne LED ein
            delay(200);      // 500ms warten
            setLed(false, ledPin, true);  // Grüne LED aus
            delay(200);      // 500ms warten
          } else if (m == '-') {
            setLed(true, ledPin, true);   // Grüne LED ein
            delay(500);      // 500ms warten
            setLed(false, ledPin, true);  // Grüne LED aus
            delay(500);      // 500ms warten
          }
          delay(200);      // 500ms warten
        }
        delay(500);      // 500ms warten
      }
    }
    delay(1000);      // 500ms warten
  }
}

// void setLed(bool state, uint8_t ledPin, bool TEST) {
//     if (!TEST) return;
    
//     pinMode(ledPin, OUTPUT);
//     digitalWrite(ledPin, state);
// }

#endif