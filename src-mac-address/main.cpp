#include <Arduino.h>
#include <WiFi.h>

void setup() {
  delay(10000);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
}

void loop() {
  while (1) {
    delay(1000);  
    Serial.print("ESP32 MAC Address: ");
    Serial.println(WiFi.macAddress());
  }
}