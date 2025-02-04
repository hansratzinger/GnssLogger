#include <TinyGsmClient.h>
#include <SPI.h> // Fügen Sie die SPI-Bibliothek hinzu
#include <Wire.h> // Fügen Sie die Wire-Bibliothek hinzu

// Define the serial connections
#define MODEM_RST 5
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM7000
#define SerialMon Serial
#define SerialAT Serial1

TinyGsm modem(SerialAT);

// APN, user and password
const char apn[]  = "web"; // Ersetzen Sie "your_apn" durch den APN Ihres Mobilfunkanbieters
const char user[] = "web@telering.at"; // Ersetzen Sie "your_user" durch den Benutzernamen Ihres Mobilfunkanbieters
const char pass[] = "web"; // Ersetzen Sie "your_pass" durch das Passwort Ihres Mobilfunkanbieters
const char pin[]  = "1796"; // Ersetzen Sie "1796" durch Ihre PIN
const char puk[]  = "07390870"; // Ersetzen Sie "12345678" durch Ihren PUK
const char cargingCode[] = "70291784037218"; // Ersetzen Sie "70291784037218" durch den tatsächlichen Aufladecode

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

  // Check network registration status
  SerialAT.println("AT+CEREG?");
  delay(100);
  while (SerialAT.available()) {
    String response = SerialAT.readString();
    SerialMon.println(response);
  }

  // Set network mode to automatic (2)
  String res;
  res = modem.setNetworkMode(2);
  if (res != "1") {
    SerialMon.println("setNetworkMode false");
    return;
  }
  delay(200);

  // Set preferred mode to NB-IoT (2)
  res = modem.setPreferredMode(2);
  if (res != "1") {
    SerialMon.println("setPreferredMode false");
    return;
  }
  delay(200);

  // Optional: Configure band for NB-IoT if needed
  // modem.sendAT("+CBANDCFG=\"NB-IOT\",8");
  // if (modem.waitResponse(10000L) != 1) {
  //   SerialMon.println(" +CBANDCFG=\"NB-IOT\" false");
  // }
  // delay(200);

  // Enable full functionality
  modem.sendAT("+CFUN=1");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println(" +CFUN=1 false");
  }
  delay(200);

  // Set APN, user and password
  SerialMon.print("Connecting to APN: ");
  SerialMon.println(apn);
  modem.gprsConnect(apn, user, pass);
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  } else {
    SerialMon.println("GPRS not connected");
  }

  // Send recharge code
  String rechargeCode = "*104*" + String(cargingCode) + "#";
  SerialMon.print("Sending recharge code: ");
  SerialMon.println(rechargeCode);
  modem.sendAT("ATD" + rechargeCode + ";");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println("Sending recharge code failed");
  } else {
    SerialMon.println("Recharge code sent successfully");
  }
}

void loop() {
  // Your code here
}