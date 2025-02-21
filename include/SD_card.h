#ifndef SD_CARD_H   // Wenn die Header-Datei noch nicht eingebunden wurde
#define SD_CARD_H   // Makrodefinition zur Vermeidung der Mehrfacheinbindung
#include <pins.h>
#include "SD_MMC.h"
#include <FS.h>
#include <TinyGPSPlus.h>

extern const bool TEST; // Deklaration der Konstante TEST

void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
bool writeDebugFile(fs::FS &fs, const char * message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
void writeDebug(const String &message);
bool initSDCard();
bool appendFile(fs::FS &fs, const char * path, const char * message);
bool writeFile(fs::FS &fs, const char * path, const char * message);  // Diese Zeile hinzufügen
bool listDirectory(fs::FS &fs, const char * dirname, uint8_t levels);
String generateFileName(TinyGPSPlus& gps);

bool initSDCard();

// Wrapper-Funktion für Serial.print
void debugPrint(const String &message);
void debugPrintln(const String &message);

#endif // SD_CARD_H