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

#include <pins.h>
#include "SD.h"
#include "SD_card.h"
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Deklaration von gpsSerial
extern HardwareSerial gpsSerial; // Serial 2 verwenden

// Funktion zum Schreiben in die Datei debug.txt
void writeDebug(const String &message) {
  if (TEST) {
      File file = SD.open("/debug.txt", FILE_APPEND);
    if (file) {
      file.println(message);
      file.close();
    } else {
      Serial.println("Fehler beim Öffnen der Datei debug.txt");
    }
  }

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


String generateFileName(TinyGPSPlus& gps) {
    char fileName[32];
    if (gps.date.isValid()) {
        snprintf(fileName, sizeof(fileName), "/gps/GPS_%04d%02d%02d.csv", 
            gps.date.year(), 
            gps.date.month(), 
            gps.date.day());
    } else {
        // Fallback wenn kein gültiges GPS-Datum verfügbar
        snprintf(fileName, sizeof(fileName), "/gps/GPS_data.csv");
    }
    Serial.printf("Generated filename: %s\n", fileName);
    return String(fileName);
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

bool listDirectory(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root) {
      Serial.println("Failed to open directory");
      return false;
  }
  if(!root.isDirectory()) {
      Serial.println("Not a directory");
      return false;
  }

  File file = root.openNextFile();
  while(file) {
      if(file.isDirectory()) {
          Serial.print("  DIR : ");
          Serial.println(file.name());
          if(levels) {
              listDirectory(fs, file.name(), levels -1);
          }
      } else {
          Serial.print("  FILE: ");
          Serial.print(file.name());
          Serial.print("  SIZE: ");
          Serial.println(file.size());
      }
      file = root.openNextFile();
  }
  return true;
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

bool writeFile(fs::FS &fs, const char * path, const char * message) {
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file) {
        Serial.println("Failed to open file for writing");
        return false;
    }
    
    size_t bytesWritten = file.print(message);
    if(bytesWritten == 0) {
        Serial.println("Write failed");
        file.close();
        return false;
    }
    
    Serial.printf("Wrote %d bytes to file\n", bytesWritten);
    file.close();
    return true;
}

bool appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return false;
  }
  if(file.print(message)){
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
    file.close();
    return false;
  }
  file.flush(); // Daten auf die SD-Karte schreiben
  file.close();
  return true;
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

bool initSDCard() {
    Serial.println("initSDCard: Starting SD card initialization...");

    // SD initialisieren
    const int chipSelect = 5; // Ändern Sie dies entsprechend Ihrem Setup
    Serial.printf("initSDCard: Initializing SD card with chipSelect %d\n", chipSelect);
    if (!SD.begin(chipSelect)) {
        Serial.println("initSDCard: SD Card Mount Failed");
        return false;
    }

    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE) {
        Serial.println("initSDCard: No SD Card attached");
        return false;
    }

    Serial.print("initSDCard: SD Card Type: ");
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    Serial.println("initSDCard: SD Card Mount erfolgreich");
    return true;
}

bool writeDebugFile(fs::FS &fs, const char * message) {
    const char* path = "/debug.txt";
    
    if(!fs.exists(path)) {
        File file = fs.open(path, FILE_WRITE);
        if(!file) {
            Serial.println("Failed to create debug file");
            return false;
        }
        file.close();
    }

    File file = fs.open(path, FILE_APPEND);
    if(!file) {
        Serial.println("Failed to open debug file");
        return false;
    }

    size_t bytesWritten = file.print(message);
    file.close();

    return bytesWritten > 0;
}

void checkFile(fs::FS &fs, const char * path) {
    File file = fs.open(path);
    if(!file) {
        Serial.printf("Failed to open file for reading: %s\n", path);
        return;
    }

    Serial.printf("File %s exists, size: %d bytes\n", path, file.size());
    
    if(file.size() > 0) {
        Serial.println("First 128 bytes of file:");
        char buffer[129] = {0};
        file.read((uint8_t*)buffer, 128);
        Serial.println(buffer);
    }
    
    file.close();
}
