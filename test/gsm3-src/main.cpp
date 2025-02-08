#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

// set GSM PIN, if any
#define GSM_PIN "1796"

// Your GPRS credentials, if any
const char apn[]  = "web";     //SET TO YOUR APN
const char gprsUser[] = "web@telering.at";
const char gprsPass[] = "web";



#include <Arduino.h>
#include <TinyGsmClient.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Ticker.h>

#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, Serial);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  60          // Time ESP32 will go to sleep (in seconds)
#define GSM_NL "\r\n" 
#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4

#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13
#define LED_PIN     12

int counter, lastIndex, numberOfPieces = 24;
String pieces[24], input;

// Set phone number, if you want to test SMS
// Set a recipient phone number to test sending SMS (it must be in international format including the "+" sign)
#define SMS_TARGET  "+4367687837602"

void setup() {
  // Set console baud rate
  Serial.begin(115200);
  delay(10);

  // Set GSM module baud rate
  SerialAT.begin(9600);
  delay(3000);

  // Restart the modem
  modem.restart();

    Serial.println("Modem restart");

  // Enter PIN if required
  String atCPIN = "AT+CPIN=" + String(GSM_PIN) ;
  SerialAT.println(atCPIN); // Ersetzen Sie "1234" durch Ihre PIN
      Serial.println("PIN entered");
  delay(5000);

  // Set network mode to automatic (2)
  SerialAT.println("AT+CNMP=2");
      Serial.println("Network mode set to automatic");

  delay(1000);

  // Set preferred mode to NB-IoT (2)
  SerialAT.println("AT+CMNB=2");
      Serial.println("Preferred mode set to NB-IoT");
  delay(1000);

  // Set full functionality
  SerialAT.println("AT+CFUN=1");
      Serial.println("Full functionality set");
  delay(1000);

  // Check network registration status
      Serial.println("Checking network registration status");
  SerialAT.println("AT+CEREG?");
  delay(100);
  while (SerialAT.available()) {
    String response = SerialAT.readString();
    Serial.println(response);
  }
}

void loop() {
  // Your code here
}