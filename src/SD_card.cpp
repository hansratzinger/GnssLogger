/*
 * pin 1 - not used          |  Micro SD card     |
 * pin 2 - CS (SS)           |                   /
 * pin 3 - DI (MOSI)         |                  |__
 * pin 4 - VDD (3.3V)        |                    |
 * pin 5 - SCK (SCLK)        | 8 7 6 5 4 3 2 1   /
 * pin 6 - VSS (GND)         | ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄  /
 * pin 7 - DO (MISO)         | ▀ ▀ █ ▀ █ ▀ ▀ ▀ |
 * pin 8 - not used          |_________________|
 *                             ║ ║ ║ ║ ║ ║ ║ ║
 *                     ╔═══════╝ ║ ║ ║ ║ ║ ║ ╚═════════╗
 *                     ║         ║ ║ ║ ║ ║ ╚══════╗    ║
 *                     ║   ╔═════╝ ║ ║ ║ ╚═════╗  ║    ║
 * Connections for     ║   ║   ╔═══╩═║═║═══╗   ║  ║    ║
 * full-sized          ║   ║   ║   ╔═╝ ║   ║   ║  ║    ║
 * SD card             ║   ║   ║   ║   ║   ║   ║  ║    ║
 * Pin name         |  -  DO  VSS SCK VDD VSS DI CS    -  |
 * SD pin number    |  8   7   6   5   4   3   2   1   9 /
 *                  |                                  █/
 *                  |__▍___▊___█___█___█___█___█___█___/
 *
 * Note:  The SPI pins can be manually configured by using `SPI.begin(sck, miso, mosi, cs).`
 *        Alternatively, you can change the CS pin and use the other default settings by using `SD.begin(cs)`.
 *
 * +--------------+---------+-------+----------+----------+----------+----------+----------+
 * | SPI Pin Name | ESP8266 | ESP32 | ESP32‑S2 | ESP32‑S3 | ESP32‑C3 | ESP32‑C6 | ESP32‑H2 |
 * +==============+=========+=======+==========+==========+==========+==========+==========+
 * | CS (SS)      | GPIO15  | GPIO5 | GPIO34   | GPIO10   | GPIO7    | GPIO18   | GPIO0    |
 * +--------------+---------+-------+----------+----------+----------+----------+----------+
 * | DI (MOSI)    | GPIO13  | GPIO23| GPIO35   | GPIO11   | GPIO6    | GPIO19   | GPIO25   |
 * +--------------+---------+-------+----------+----------+----------+----------+----------+
 * | DO (MISO)    | GPIO12  | GPIO19| GPIO37   | GPIO13   | GPIO5    | GPIO20   | GPIO11   |
 * +--------------+---------+-------+----------+----------+----------+----------+----------+
 * | SCK (SCLK)   | GPIO14  | GPIO18| GPIO36   | GPIO12   | GPIO4    | GPIO21   | GPIO10   |
 * +--------------+---------+-------+----------+----------+----------+----------+----------+
 *
 * For more info see file README.md in this library or on URL:
 * https://github.com/espressif/arduino-esp32/tree/master/libraries/SD
 */

#include <SD.h>
#include "SD_card.h"
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Deklaration von gpsSerial
extern HardwareSerial gpsSerial; // Serial 2 verwenden

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 115200

// Funktion zum Schreiben in die Datei debug.txt
// filepath: /c:/esp32/GnssLogger/src/SD_card.cpp
void writeDebug(const String &message) {
  // Überprüfen, ob die Datei existiert
  if (!SD.exists("/debug.txt")) {
    // Datei erstellen, falls sie nicht existiert
    File file = SD.open("/debug.txt", FILE_WRITE);
    if (!file) {
      Serial.println("Fehler beim Erstellen der Datei debug.txt");
      return;
    }
    file.close();
  }

  // Datei im Anhängemodus öffnen
  File file = SD.open("/debug.txt", FILE_APPEND);
  if (!file) {
    Serial.println("Fehler beim Öffnen der Datei debug.txt");
    return;
  }

  // Nachricht in die Datei schreiben
  file.println(message);
  file.close();
}

// Wrapper-Funktion für Serial.print
void debugPrint(const String &message) {
    Serial.print(message);
    writeDebug(message);
  }

// Wrapper-Funktion für Serial.println
void debugPrintln(const String &message) {
  Serial.println(message);
  writeDebug(message + "\n");
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    // Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    // Serial.println("Message appended");
  } else {
    // Serial.println("Append failed");
  }
  file.close(); // Stellen Sie sicher, dass die Datei geschlossen wird
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %lu ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %lu ms\n", 2048 * 512, end);
  file.close();
}

String generateFileName(TinyGPSPlus &gps) {
  char fileName[32];
  snprintf(fileName, sizeof(fileName), "/log_%04d_%02d_%02d.csv", gps.date.year(), gps.date.month(), gps.date.day());
  return String(fileName);
}

// filepath: /c:/esp32/GnssLogger/src/SD_card.cpp
char getDirectionOfLat(double latitude) {
  return (latitude >= 0) ? 'N' : 'S';
}

char getDirectionOfLng(double longitude) {
  return (longitude >= 0) ? 'E' : 'W';
}

String convertToDMM(double decimalDegrees) {
  int degrees = (int)decimalDegrees;
  double minutes = (decimalDegrees - degrees) * 60;
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%d°%.6f'", degrees, minutes);
  return String(buffer);
}

void writeCreationAndModificationDate(fs::FS &fs, const char *path, TinyGPSPlus &gps) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  String creationDate = "Creation Date: " + String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + " " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "\n";
  String modificationDate = "Modification Date: " + String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + " " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "\n";
  if (file.print(creationDate) && file.print(modificationDate)) {
    Serial.println("Creation and modification dates written");
  } else {
    Serial.println("Failed to write dates");
  }
  file.close();
}

void initializeSDCard() {
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  debugPrintln("Serial 2 started at " + String(GPS_BAUD) + " baud rate");

  if (!SD.begin()) {
    debugPrintln("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    debugPrintln("No SD card attached");
    return;
  }

  debugPrint("SD Card Type: ");
  if (cardType == CARD_MMC) {
    debugPrintln("MMC");
  } else if (cardType == CARD_SD) {
    debugPrintln("SDSC");
  } else if (cardType == CARD_SDHC) {
    debugPrintln("SDHC");
  } else if (cardType == CARD_UNKNOWN) {
    debugPrintln("UNKNOWN CARD");
  } else if (cardType == CARD_NONE) {
    debugPrintln("No SD card attached");
    return;
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  debugPrintln("SD Card Size: " + String(cardSize) + "MB");

  listDir(SD, "/", 0);
}