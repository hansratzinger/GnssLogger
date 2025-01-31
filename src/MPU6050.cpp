#include "MPU6050.h"

Adafruit_MPU6050 mpu;

float accelOffsets[3] = {0, 0, 0};
float gyroOffsets[3] = {0, 0, 0};

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

        delay(10);
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

void readMPU6050(char* logging) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Apply offsets to the accelerometer and gyroscope values
    float accelX = a.acceleration.x - accelOffsets[0];
    float accelY = a.acceleration.y - accelOffsets[1];
    float accelZ = a.acceleration.z - accelOffsets[2];

    float gyroX = g.gyro.x - gyroOffsets[0];
    float gyroY = g.gyro.y - gyroOffsets[1];
    float gyroZ = g.gyro.z - gyroOffsets[2];

    // Format the accelerometer values and append to the logging variable
    snprintf(logging + strlen(logging), 100 - strlen(logging), ", Accel: [%.2f, %.2f, %.2f]",
             accelX, accelY, accelZ);

    // Print the accelerometer values to the Serial Monitor
    Serial.print("Accel X: "); Serial.print(accelX); Serial.print(" m/s^2, ");
    Serial.print("Accel Y: "); Serial.print(accelY); Serial.print(" m/s^2, ");
    Serial.print("Accel Z: "); Serial.print(accelZ); Serial.println(" m/s^2");

    // Print the gyroscope values to the Serial Monitor
    Serial.print("Gyro X: "); Serial.print(gyroX); Serial.print(" rad/s, ");
    Serial.print("Gyro Y: "); Serial.print(gyroY); Serial.print(" rad/s, ");
    Serial.print("Gyro Z: "); Serial.print(gyroZ); Serial.println(" rad/s");
}