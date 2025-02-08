#define TINY_GSM_MODEM_SIM7000 // Definieren Sie das Modemmodell
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialMon Serial
#define SerialAT Serial1

#include <TinyGsmClient.h>
// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial connections
#define SerialMon Serial
#define SerialAT Serial1

// set GSM PIN, if any
#define GSM_PIN "1796"

// Define the serial connections
#define MODEM_RST 5
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  60          // Time ESP32 will go to sleep (in seconds)

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


#include <SPI.h> // Fügen Sie die SPI-Bibliothek hinzu
#include <Wire.h> // Fügen Sie die Wire-Bibliothek hinzu
#include <Ticker.h> // Fügen Sie die Ticker-Bibliothek hinzu


#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

// APN, user and password
const char apn[]  = "web"; // Ersetzen Sie "your_apn" durch den APN Ihres Mobilfunkanbieters
const char user[] = "web@telering.at"; // Ersetzen Sie "your_user" durch den Benutzernamen Ihres Mobilfunkanbieters
const char pass[] = "web"; // Ersetzen Sie "your_pass" durch das Passwort Ihres Mobilfunkanbieters
const char pin[]  = "1796"; // Ersetzen Sie "1796" durch Ihre PIN
const char puk[]  = "07390870"; // Ersetzen Sie "12345678" durch Ihren PUK
const char rechargeCode[] = "70291784037218"; // Ersetzen Sie "70291784037218" durch den tatsächlichen Aufladecode

void modemPowerOn(){
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff(){
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500);
  digitalWrite(PWR_PIN, HIGH);
}


void modemRestart(){
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}


void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

    // Set LED OFF
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);


  while (modem.init()) {
    modem.restart();
    if (!modem.init()) {
      SerialMon.println("Failed to restart the modem, retrying...");
      delay(2000);
    }
  }
  SerialMon.println("Modem initialized");

  // Set GSM module baud rate
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(15000);

  // Enter PIN if required
  SerialAT.println("AT+CPIN=\"" + String(pin) + "\"");
  delay(5000);
  while (SerialAT.available()) {
    String response = SerialAT.readString();
    SerialMon.println(response);
    if (response.indexOf("SIM PIN") != -1) {
      SerialMon.println("PIN required, sending PIN...");
      SerialAT.println("AT+CPIN=\"" + String(pin) + "\"");
      delay(5000);
    } else if (response.indexOf("SIM PUK") != -1) {
      SerialMon.println("PUK required, sending PUK...");
      SerialAT.println("AT+CPIN=\"" + String(puk) + "\",\"" + String(pin) + "\"");
      delay(5000);
    }
  }
}

void loop() {

  String res;

  Serial.println("========INIT========");

  if (!modem.init()) {
    modemRestart();
    delay(2000);
    Serial.println("Failed to restart modem, attempting to continue without restarting");
    return;
  }

  Serial.println("========SIMCOMATI======");
  modem.sendAT("+SIMCOMATI");
  modem.waitResponse(1000L, res);
  res.replace("\nOK\n", "");
  Serial.println(res);
  res = "";
  Serial.println("=======================");

  Serial.println("=====Preferred mode selection=====");
  modem.sendAT("+CNMP?");
  if (modem.waitResponse(1000L, res) == 1) {
    res.replace("\nOK\n", "");
    Serial.println(res);
  }
  res = "";
  Serial.println("=======================");


  Serial.println("=====Preferred selection between CAT-M and NB-IoT=====");
  modem.sendAT("+CMNB?");
  if (modem.waitResponse(1000L, res) == 1) {
    res.replace("\nOK\n", "");
    Serial.println(res);
  }
  res = "";
  Serial.println("=======================");


  String name = modem.getModemName();
  Serial.println("Modem Name: " + name);

  String modemInfo = modem.getModemInfo();
  Serial.println("Modem Info: " + modemInfo);

  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }

  for (int i = 0; i <= 4; i++) {
    uint8_t network[] = {
        2,  /*Automatic*/
        13, /*GSM only*/
        38, /*LTE only*/
        51  /*GSM and LTE only*/
    };
    Serial.printf("Try %d method\n", network[i]);
    modem.setNetworkMode(network[i]);
    delay(3000);
    bool isConnected = false;
    int tryCount = 60;
    while (tryCount--) {
      int16_t signal =  modem.getSignalQuality();
      Serial.print("Signal: ");
      Serial.print(signal);
      Serial.print(" ");
      Serial.print("isNetworkConnected: ");
      isConnected = modem.isNetworkConnected();
      Serial.println( isConnected ? "CONNECT" : "NO CONNECT");
      if (isConnected) {
        break;
      }
      delay(1000);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    if (isConnected) {
        break;
    }
  }
  digitalWrite(LED_PIN, HIGH);

  Serial.println();
  Serial.println("Device is connected .");
  Serial.println();

  Serial.println("=====Inquiring UE system information=====");
  modem.sendAT("+CPSI?");
  if (modem.waitResponse(1000L, res) == 1) {
    res.replace("\nOK\n", "");
    Serial.println(res);
  }

  Serial.println("/**********************************************************/");
  Serial.println("After the network test is complete, please enter the  ");
  Serial.println("AT command in the serial terminal.");
  Serial.println("/**********************************************************/\n\n");

  while (1) {
    while (SerialAT.available()) {
      SerialMon.write(SerialAT.read());
    }
    while (SerialMon.available()) {
      SerialAT.write(SerialMon.read());
    }
  }
  // // Check network registration status
  // SerialAT.println("AT+CEREG?");
  // delay(100);
  // while (SerialAT.available()) {
  //   String response = SerialAT.readString();
  //   SerialMon.println(response);
  // }

  // Set network mode to automatic (2)
  // SerialMon.println("Setting network mode to automatic (2)...");
  // String res;
  // res = modem.setNetworkMode(2);
  // SerialMon.print("Network mode response: ");
  // SerialMon.println(res);
  // if (res != "1") {
  //   SerialMon.println("setNetworkMode false");
  //   return;
  // }
  // delay(200);

  // Set preferred mode to NB-IoT (2)
  // SerialMon.println("Setting preferred mode to NB-IoT (2)...");
  // res = modem.setPreferredMode(2);
  // SerialMon.print("Preferred mode response: ");
  // SerialMon.println(res);
  // if (res != "1") {
  //   SerialMon.println("setPreferredMode false");
  //   return;
  // }
  // delay(200);

  // Optional: Configure band for NB-IoT if needed
  // modem.sendAT("+CBANDCFG=\"NB-IOT\",8");
  // if (modem.waitResponse(10000L) != 1) {
  //   SerialMon.println(" +CBANDCFG=\"NB-IOT\" false");
  // }
  // delay(200);


  
  // Enable full functionality
  // SerialMon.println("Enabling full functionality...");
  // modem.sendAT("+CFUN=1");
  // SerialMon.write(SerialAT.read());
  // if (modem.waitResponse(10000L) != 1) {
  //   SerialMon.println(" +CFUN=1 false");
  // }
  // delay(200);

  // // Set APN, user and password
  // SerialMon.print("Connecting to APN: ");
  // SerialMon.println(apn);
  // modem.gprsConnect(apn, user, pass);
  // if (modem.isGprsConnected()) {
  //   SerialMon.println("GPRS connected");
  // } else {
  //   SerialMon.println("GPRS not connected");
  // }


//   // Send recharge code
//   String ussdCode = "*104*" + String(rechargeCode) + "#";
//   SerialMon.print("Sending recharge code: ");
//   SerialMon.println(ussdCode);
//   modem.sendAT("ATD" + ussdCode + ";");
//   SerialMon.write(SerialAT.read());

//   if (modem.waitResponse(10000L) != 1) {
//     SerialMon.println("Sending recharge code failed");
//   } else {
//     SerialMon.println("Recharge code sent successfully");
//   }
// }



}