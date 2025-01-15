// GnssLogger
// HR 2025-01-15 NK
#include <TinyGPS++.h>
#include "SD_card.h"

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 115200

unsigned long start = millis();
String gpstime, date, lat, lon, speed, altitude ,hdop, satellites, logging, firstline;
String gpstimeLast, dateLast, latLast, lonLast, speedLast, altitudeLast ,hdopLast, satellitesLast, loggingLast, firstlineLast;  
double distanceLast, latDifference, lonDifference;

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

void setup() {
  // Serial Monitor
  Serial.begin(115200);
  
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started at 9600 baud rate");

  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else if (cardType == CARD_UNKNOWN) {
    Serial.println("UNKNOWN CARD");
  } else if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.println("SD Card Size: " + String(cardSize) + "MB");

  listDir(SD, "/", 0);
}

void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated()) {
      lat = String(gps.location.lat(), 6);      
      lon = String(gps.location.lng(), 6);

      // Bestimme die Himmelsrichtung
      String directionLat = getDirectionLat(gps.location.lat());
      String directionLng = getDirectionLng(gps.location.lng());

      char timeBuffer[10];
      sprintf(timeBuffer, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      gpstime = String(timeBuffer);
      date = String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day());
      hdop = String(gps.hdop.hdop());
      satellites = String(gps.satellites.value());
      speed = String(gps.speed.knots());
      altitude = String(gps.altitude.meters());
      firstline = "Date,UTC,Lat,N/S,Lon,E/W,knots,Alt/m,HDOP,Satellites,Fix-distance/m,LatDiff,LonDiff\n";
      logging = date + "," + gpstime + "," + lat + "," + directionLat + "," + lon + "," +  directionLng + "," + speed + "," + altitude + "," + hdop + "," + satellites;

      // Berechne die Entfernung zum letzten Punkt
      if (latLast != "" && lonLast != "") {
      distanceLast = calculateDistance(lat.toDouble(), lon.toDouble(), latLast.toDouble(), lonLast.toDouble());
      logging += "," + String(distanceLast);
      latDifference = calculateDifference(lat.toDouble(), latLast.toDouble());
      lonDifference = calculateDifference(lon.toDouble(), lonLast.toDouble());  
      logging += "," + String(latDifference,6) + "," + String(lonDifference,6);
      }
      logging += "\n";

      if (date != "2000/0/0") {
      // SD card    
      // Generiere den Dateinamen basierend auf dem aktuellen Datum
      String fileName = generateFileName(gps);

      // Überprüfe, ob die Datei bereits existiert
      if (!SD.exists(fileName.c_str())) {
        // Datei existiert nicht, erstelle die Datei und schreibe die erste Zeile
        writeFile(SD, fileName.c_str(), firstline.c_str());
      }
   
      // Schreibe die Daten in die Datei
      appendFile(SD, fileName.c_str(), logging.c_str());

      // Serial monitor          
      Serial.print("Date: ");
      Serial.println(date);
      Serial.print("Time: ");
      Serial.println(gpstime);
      Serial.print("LAT: ");
      Serial.print(lat);
      Serial.println(" " + directionLat);
      Serial.print("LON: "); 
      Serial.print(lon);
      Serial.println(" " + directionLng);
      Serial.print("SPEED (knots) = "); 
      Serial.println(speed); 
      Serial.print("Alt = "); 
      Serial.println(altitude); 
      Serial.print("HDOP = "); 
      Serial.println(hdop); 
      Serial.print("Satellites = "); 
      Serial.println(satellites); 
      
      Serial.print("Distance (m) = ");
      Serial.println(distanceLast);
      Serial.println("----------------------------");

      // Save the last values
      gpstimeLast = gpstime;
      dateLast = date;
      latLast = lat;
      lonLast = lon;

      delay(1000);
      }
    delay(3000);
  }
}