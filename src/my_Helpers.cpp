#include <Arduino.h>
#include <map>
#include "my_Helpers.h"

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
void blinkMorseCode(const String &text, int ledPin, int repeatCount, bool TEST) {
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
            digitalWrite(ledPin, HIGH);
            mydelay(200,TEST); // Punkt: 200 ms
            digitalWrite(ledPin, LOW);
          } else if (m == '-') {
            digitalWrite(ledPin, HIGH);
            mydelay(500,TEST); // Strich: 500 ms
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


void ledON(int ledPin, bool TEST) {
  if (!TEST) {
    return;
  }
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
}

void ledOFF(int ledPin, bool TEST) {
  if (!TEST) {
    return;
  }
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}

void ledMode(bool mode, bool TEST) {
  if (!TEST) {
    return;
  } 
  if (mode) {
    // ledON(GREEN_LED_PIN, TEST);
    ledOFF(RED_LED_PIN, TEST);
  } else {
    ledON(RED_LED_PIN, TEST);
    // ledOFF(GREEN_LED_PIN, TEST);
  }
}

// mydelayTime in msec, TEST = true -> delay, TEST = false -> no delay
void mydelay(unsigned long mydelayTimeMillis, bool TEST) {
  if (!TEST) {
    return;
  }  
  unsigned long currentMillis = millis();
  while (millis() - currentMillis < mydelayTimeMillis) {
    // Warten
  }
}

void appendText(char* text, const char* toappendText) {
    strcat(text, toappendText); // Fügt 'text' an 'base' an

    // Verwendung
    // char message[50] = "Hello";
    // appendText(message, " World!");
}
