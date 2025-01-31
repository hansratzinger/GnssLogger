#include "MPU6050.h"

Adafruit_MPU6050 mpu;

void setupMPU6050() {
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 initialized");
}

void readMPU6050(char* logging) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Format the accelerometer values and append to the logging variable
    snprintf(logging + strlen(logging), 100 - strlen(logging), ", Accel: [%.2f, %.2f, %.2f]",
             a.acceleration.x, a.acceleration.y, a.acceleration.z);
}