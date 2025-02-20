#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPS++.h>
#include "GNSS_module.h"
#include <WiFi.h>


const bool TEST = true;

// Hilfsfunktion für Distanzberechnung zwischen zwei GPS-Koordinaten
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000; // Radius der Erde in Metern
    double latRad1 = radians(lat1);
    double latRad2 = radians(lat2);
    double deltaLat = radians(lat2 - lat1);
    double deltaLon = radians(lon2 - lon1);

    double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
               cos(latRad1) * cos(latRad2) *
               sin(deltaLon / 2) * sin(deltaLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c; // Entfernung in Metern
}

// Hilfsfunktion für einfache Differenzberechnung
double calculateDifference(double firstData, double secoundData) {
    return firstData - secoundData;
}

// CSV Header-Zeile 
const char* CSV_HEADER = "timestamp,latitude,longitude,altitude,speed,course,satellites,hdop,distance\n";

// Funktion zum Schreiben der GNSS-Daten in CSV
void writeGNSSDataToSD(File* file, TinyGPSPlus* gps, double distance) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "%lu,%.6f,%.6f,%.2f,%.2f,%.2f,%d,%.2f,%.2f\n",
             millis(),
             gps->location.lat(),
             gps->location.lng(), 
             gps->altitude.meters(),
             gps->speed.kmph(),
             gps->course.deg(),
             gps->satellites.value(),
             gps->hdop.hdop(),
             distance);
             
    file->print(buffer);
}

unsigned long getTimeDifference(const char* gpstime, const char* gpstimeLast) {
    if (!gpstime || !gpstimeLast) {
        return 0;
    }

    int hour1 = 0, minute1 = 0, second1 = 0;
    int hour2 = 0, minute2 = 0, second2 = 0;
 
    // Validate time format and parse
    if (sscanf(gpstime, "%2d:%2d:%2d", &hour1, &minute1, &second1) != 3 ||
        sscanf(gpstimeLast, "%2d:%2d:%2d", &hour2, &minute2, &second2) != 3) {
        return 0;
    }

    // Validate time values
    if (hour1 > 23 || minute1 > 59 || second1 > 59 ||
        hour2 > 23 || minute2 > 59 || second2 > 59) {
        return 0;
    }

    // Convert to seconds
    unsigned long time1 = hour1 * 3600 + minute1 * 60 + second1;
    unsigned long time2 = hour2 * 3600 + minute2 * 60 + second2;

    // Handle day rollover
    if (abs((long)time1 - (long)time2) > 43200) { // More than 12 hours difference
        if (time1 > time2) {
            time2 += 86400; // Add 24 hours
        } else {
            time1 += 86400;
        }
    }

    return abs((long)time1 - (long)time2);
}
