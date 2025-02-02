#include "MPU6050.h"
#include "my_Helpers.h"

Adafruit_MPU6050 mpu;

float accelOffsets[3] = {0, 0, 0};
float gyroOffsets[3] = {0, 0, 0};
float accelX = 0, accelY = 0, accelZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0, temp = 0;

void setupMPU6050() {
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            myDelay(10,0);
        }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 initialized");

    // Calibrate the MPU6050
    calibrateMPU6050();
}

void calibrateMPU6050() {
    const int numReadings = 1000;
    float accelSum[3] = {0, 0, 0};
    float gyroSum[3] = {0, 0, 0};

    for (int i = 0; i < numReadings; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        accelSum[0] += a.acceleration.x;
        accelSum[1] += a.acceleration.y;
        accelSum[2] += a.acceleration.z;

        gyroSum[0] += g.gyro.x;
        gyroSum[1] += g.gyro.y;
        gyroSum[2] += g.gyro.z;

        myDelay(10,0);
    }

    accelOffsets[0] = accelSum[0] / numReadings;
    accelOffsets[1] = accelSum[1] / numReadings;
    accelOffsets[2] = accelSum[2] / numReadings - 9.81; // Adjust for gravity

    gyroOffsets[0] = gyroSum[0] / numReadings;
    gyroOffsets[1] = gyroSum[1] / numReadings;
    gyroOffsets[2] = gyroSum[2] / numReadings;

    Serial.println("MPU6050 calibrated");
    Serial.print("Accel Offsets: ");
    Serial.print(accelOffsets[0]); Serial.print(", ");
    Serial.print(accelOffsets[1]); Serial.print(", ");
    Serial.println(accelOffsets[2]);
    Serial.print("Gyro Offsets: ");
    Serial.print(gyroOffsets[0]); Serial.print(", ");
    Serial.print(gyroOffsets[1]); Serial.print(", ");
    Serial.println(gyroOffsets[2]);
}

void readMPU6050() {
    sensors_event_t a, g, tempEvent;
    mpu.getEvent(&a, &g, &tempEvent);

    // Apply offsets to the accelerometer and gyroscope values
    accelX = a.acceleration.x - accelOffsets[0];
    accelY = a.acceleration.y - accelOffsets[1];
    accelZ = a.acceleration.z - accelOffsets[2];

    gyroX = g.gyro.x - gyroOffsets[0];
    gyroY = g.gyro.y - gyroOffsets[1];
    gyroZ = g.gyro.z - gyroOffsets[2];

 
    temp = tempEvent.temperature;

}
