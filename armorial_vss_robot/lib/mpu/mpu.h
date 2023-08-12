#ifndef ARMORIAL_SUASSUNA_MPU_H
#define ARMORIAL_SUASSUNA_MPU_H

#define PIN_MPU_SCL 22
#define PIN_MPU_SDA 21

#define MPU_RESOLUTION 10000

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class MPU {
public:
  static void setup();
  static void computeMPUCallback(void *arg);
  static float getGyroX();
  static float getGyroY();
  static float getGyroZ();
  static float getGyroXDeg();
  static float getGyroYDeg();
  static float getGyroZDeg();
  static void start();
  static void stop();

protected:
  static void readFromMPU();

private:
  static Adafruit_MPU6050 _mpu;
  static float _mpuAngularSpeed;
  static float _gyro_x;
  static float _gyro_y;
  static float _gyro_z;
  static esp_timer_handle_t _timer;
  static bool _startedTimer;

};

#endif // ARMORIAL_SUASSUNA_MPU_H
