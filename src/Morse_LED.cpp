#include <Arduino.h>
#include <map>
#include "Morse_LED.h"

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

// Funktion zum Blinken der LED im Morse-Code-Rhythmus
void blinkMorseCode(const String &text, int ledPin, int repeatCount) {
  pinMode(ledPin, OUTPUT);

  for (int r = 0; r < repeatCount; r++) {
    for (char c : text) {
      c = toupper(c); // Konvertiere den Buchstaben in Großbuchstaben
      if (morseCode.find(c) != morseCode.end()) {
        String code = morseCode[c];
        for (char m : code) {
          if (m == '.') {
            digitalWrite(ledPin, HIGH);
            delay(200); // Punkt: 200 ms
            digitalWrite(ledPin, LOW);
          } else if (m == '-') {
            digitalWrite(ledPin, HIGH);
            delay(500); // Strich: 500 ms
            digitalWrite(ledPin, LOW);
          }
          delay(200); // Pause zwischen Punkten und Strichen
        }
        delay(500); // Pause zwischen Buchstaben
      }
    }
    delay(1000); // Pause zwischen Wiederholungen
  }
}

// Beispielaufruf der Funktion in setup oder loop
// void setup() {
//   Serial.begin(115200);
//   String text = "HELLO";
//   int ledPin = RED_LED_PIN;
//   int repeatCount = 3;
//   blinkMorseCode(text, ledPin, repeatCount);
// }

// void loop() {
//   // Ihr Code hier
// }