#include <Arduino.h> // Include the Arduino core library
#include <SD.h> // Include the SD_MMC library 
#include <FS.h>
#include "pins.h"

void setup() {
  delay (5000);
  Serial.begin(115200); // Initialize serial communication
  for (int i = 0; i < 50; i++) {    
      Serial.println("Initializing SD card..."); // Print initialization message
    if (!SD.begin()) { // Check if SD card is mounted successfully
      Serial.println("Failed to mount SD card"); // Print error message if SD card failed to mount
    delay(500);
    } else {
      Serial.println("SD card mounted successfully"); // Print success message if SD card is mounted successfully
      break;
    }
  }
    // Create a file in the root directory of SD card and open it in write mode.
    File file = SD.open("/test.txt", FILE_WRITE); 
    if (!file) {
      Serial.println("Failed to open file for writing"); // Print error message if file failed to open
      return;
    } else {
      Serial.println("File opened successfully"); // Print success message if file opened successfully
    }
    
    if (file.println("Test file write")) { // Write a line of text to the file
      Serial.println("File write successful"); // Print success message if write operation is successful
    } else {
      Serial.println("File write failed"); // Print error message if write operation failed
    }
    file.close(); // Close the file

    File root = SD.open("/"); // Open the root directory of SD card
    if (!root) {
      Serial.println("Failed to open directory"); // Print error message if directory failed to open
      return;
    }
    Serial.println("Files found in root directory:"); // Print the list of files found in the root directory
    while (File file = root.openNextFile()) { // Loop through all the files in the root directory
      Serial.print("  ");
      Serial.print(file.name()); // Print the filename
      Serial.print("\t");
      Serial.println(file.size()); // Print the filesize
      file.close(); // Close the file
    }
    root.close(); // Close the root directory
    
}

void loop() {} // Empty loop function, does nothing
