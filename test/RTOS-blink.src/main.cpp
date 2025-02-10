
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <vfs_api.h>

#define LED1 25 // red  LED
#define LED2 26 // green LED

SemaphoreHandle_t sem; // Create semaphore handle 

void blink1(void *pvParameters){
    pinMode(LED1, OUTPUT);
    while(1){
        for(int i=0; i<5; i++){
             digitalWrite(LED1, HIGH);
             delay(250);
             digitalWrite(LED1, LOW);
             delay(250);
         }
         xSemaphoreGive(sem); // Release semaphore
         for(int i=0; i<5; i++){
             digitalWrite(LED1, HIGH);
             delay(250);
             digitalWrite(LED1, LOW);
             delay(250);
         }
         delay(2000);
     }
}

void blink2(void *pvParameters){
    pinMode(LED2, OUTPUT);
    while(1){
    
        xSemaphoreTake(sem,portMAX_DELAY); // Take semaphore
        for(int i=0; i<10; i++){
            digitalWrite(LED2, HIGH);
            delay(333);
            digitalWrite(LED2, LOW);
            delay(333);   
        }
    }
}



void setup() {
    sem = xSemaphoreCreateBinary(); // Create a binary semaphore
         
    xTaskCreate(
        blink1,      // Function name of the task
        "Blink 1",   // Name of the task (e.g. for debugging)
        2048,        // Stack size (bytes)
        NULL,        // Parameter to pass
        1,           // Task priority
        NULL         // Task handle
    );
    xTaskCreate(
        blink2,     // Function name of the task
        "Blink 2",  // Name of the task (e.g. for debugging)
        2048,       // Stack size (bytes)
        NULL,       // Parameter to pass
        1,          // Task priority
        NULL        // Task handle
    );
}

void loop(){}