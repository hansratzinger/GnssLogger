# Topologie RNLI Sensorproject

## MainUnit

placed on deck

### GnssLogger

- ESP32 WROOM-32  
- LC76G GNSS module  
- SC-card reader  
- ESPnow connection to GprsSender -> Position & RPM data  
- ESPnow connection from LoRaBridge <- RPM data  

### GprsSender

- LILYGO T-SIM7000G ESP32  
or  
- TTGO T-Call ESP32 with SIM800L GSM/GPRS  
- ESPnow connection from GnssLogger <- Position & RPM data  

### LoRaBridge

- HELTEC LoRa32 OLED module  
- LoRa connection from RpmSensor <- RPM data  
- ESPnow connection to GnssLogger -> RPM data  

## RpmUnit

placed in engine room  

### RpmSensor

- HELTEC LoRa32 OLED module  
- Infrared sensor module  
- LoRa connection to LoRaBridge -> RPM data  

