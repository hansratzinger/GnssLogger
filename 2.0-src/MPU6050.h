#ifndef MPU6050_H
#define MPU6050_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

void setupMPU6050();
void readMPU6050();
void calibrateMPU6050();

extern float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp;

#endif // MPU6050_H