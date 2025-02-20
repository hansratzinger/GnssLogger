#include <Arduino.h>

// Funktion zur Konvertierung des MAC-Adress-Strings in ein Byte-Array
void stringToMac(const String &mac, uint8_t *bytes) {
  int i = 0;
  int start = 0;
  int end;
  while (i < 6) {
    end = mac.indexOf(':', start);
    if (end == -1) {
      end = mac.length();
    }
    String byteString = mac.substring(start, end);
    bytes[i] = strtol(byteString.c_str(), NULL, 16);
    start = end + 1;
    i++;
  }
}

void setup() {
  Serial.begin(115200);
  delay(10000);

  // Array von MAC-Adressen als Strings
  String macAddresses[] = {
    "34:98:7A:86:F5:CC",
    "08:3A:F2:B5:7E:24",
    "78:21:84:9C:62:7C",
    "C0:49:EF:F0:9D:6C",
    "B0:B2:1C:F8:6E:70",
    "B0:B2:1C:F8:6E:71",
    "B0:B2:1C:F8:6E:C8",
    "54:43:B2:E4:02:B4"
  };

  // Anzahl der MAC-Adressen im Array
  int numMacAddresses = sizeof(macAddresses) / sizeof(macAddresses[0]);

  // Array für die Hex-Bytes
  uint8_t macBytes[6];

  // Schleife durch das Array von MAC-Adressen
  for (int j = 0; j < numMacAddresses; j++) {
    // MAC-Adresse in Hex-Bytes konvertieren
    stringToMac(macAddresses[j], macBytes);

    // Hex-Bytes im gewünschten Format ausgeben
    Serial.print("MAC Address " + String(j + 1) + " in Byte Format: {");
    for (int i = 0; i < 6; i++) {
      Serial.printf("0x%02X", macBytes[i]);
      if (i < 5) {
        Serial.print(", ");
      }
    }
    Serial.println("}");
  }
}

void loop() {
  // Nichts zu tun
}