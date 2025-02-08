#define TINY_GSM_MODEM_SIM7000 // Definieren Sie das Modemmodell
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <SPI.h> // Fügen Sie die SPI-Bibliothek hinzu
#include <Wire.h> // Fügen Sie die Wire-Bibliothek hinzu

// Define the serial connections
#define SerialMon Serial
#define SerialAT Serial1

#define GSM_PIN "1796"

// Define the serial connections
#define MODEM_RST 5
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26

TinyGsm modem(SerialAT);

// Telefonnummer und Nachricht
const char phone_number[] = "+4367687837602"; // Ersetzen Sie "+491234567890" durch die tatsächliche Telefonnummer
const char message[] = "Testnachricht von ESP32 mit SIM7000"; // Ersetzen Sie "Testnachricht von ESP32 mit SIM7000" durch Ihre Nachricht

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  // Set GSM module baud rate
  SerialAT.begin(9600);
  delay(3000);

  // Restart the modem
  modem.restart();

  // Enter PIN if required
  SerialAT.println("AT+CPIN=\"" + String(GSM_PIN) + "\"");
  delay(5000);
  while (SerialAT.available()) {
    String response = SerialAT.readString();
    SerialMon.println(response);
    if (response.indexOf("SIM PIN") != -1) {
      SerialMon.println("PIN required, sending PIN...");
      SerialAT.println("AT+CPIN=\"" + String(GSM_PIN) + "\"");
      delay(5000);
    } else if (response.indexOf("SIM PUK") != -1) {
      SerialMon.println("PUK required, sending PUK...");
      // Hier müssen Sie den PUK und die neue PIN senden, falls erforderlich
      // SerialAT.println("AT+CPIN=\"PUK\",\"neue PIN\"");
      delay(5000);
    }
  }

  // Set preferred mode to NB-IoT (2)
  SerialMon.println("Setting preferred mode to NB-IoT (2)...");
  String res;
  res = modem.setPreferredMode(2);
  SerialMon.print("Preferred mode response: ");
  SerialMon.println(res);
  if (res != "1") {
    SerialMon.println("setPreferredMode false");
    return;
  }
  delay(200);

  // Enable full functionality
  SerialMon.println("Enabling full functionality...");
  modem.sendAT("+CFUN=1");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println(" +CFUN=1 false");
  }
  delay(200);
}

void loop() {
  // Versuchen, alle möglichen Netzwerkmodi zu durchlaufen
  for (int i = 0; i < 4; i++) {
    uint8_t network[] = {
        2,  /*Automatic*/
        13, /*GSM only*/
        38, /*LTE only*/
        51  /*GSM and LTE only*/
    };
    Serial.printf("Try %d method\n", network[i]);
    modem.setNetworkMode(network[i]);
    delay(2000);
    bool isConnected = false;
    int tryCount = 5;
    while (tryCount--) {
      int16_t signal = modem.getSignalQuality();
      Serial.print("Signal: ");
      Serial.print(signal);
      Serial.print(" ");
      Serial.print("isNetworkConnected: ");
      isConnected = modem.isNetworkConnected();
      Serial.println(isConnected ? "CONNECT" : "NO CONNECT");
      if (isConnected) {
        break;
      }
      delay(1000);
    }
    if (isConnected) {
      break;
    }
  }

  // Check network registration status
  SerialMon.println("Checking network registration status...");
  SerialAT.println("AT+CEREG?");
  delay(100);
  bool isRegistered = false;
  while (SerialAT.available()) {
    String response = SerialAT.readString();
    SerialMon.println(response);
    if (response.indexOf("+CEREG: 1") != -1 || response.indexOf("+CEREG: 5") != -1) {
      isRegistered = true;
      break;
    }
  }

  if (isRegistered) {
    // Send SMS
    SerialMon.println("Sending SMS...");
    if (modem.sendSMS(phone_number, message)) {
      SerialMon.println("SMS sent successfully");
    } else {
      SerialMon.println("Failed to send SMS");
    }
  } else {
    SerialMon.println("Not registered to network, retrying...");
    delay(10000); // Warten Sie 10 Sekunden, bevor Sie es erneut versuchen
  }
}