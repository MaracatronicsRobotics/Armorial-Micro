#ifndef ARMORIAL_SUASSUNA_MPU_H
#define ARMORIAL_SUASSUNA_MPU_H

#define PIN_MPU_SCL 22
#define PIN_MPU_SDA 21

#define MPU_RESOLUTION 10000
#define MPU_KP 11
#define MPU_KI 0.09
#define MPU_KD 0

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class MPU {
public:
  static void setup();
  static void computeMPUCallback(void *arg);
  static float getComputedAngleCorrection();
  static float getComputedPitchOutput();
  static void start();
  static void stop();

protected:
  static void readFromMPU();

private:
  static Adafruit_MPU6050 _mpu;
  static float _mpuAngularSpeed;
  static float _gyro_x_calibration;
  static float _gyro_y_calibration;
  static float _gyro_z_calibration;
  static float _gyro_x;
  static float _gyro_y;
  static float _gyro_z;
  static float _acc_x;
  static float _acc_y;
  static float _acc_z;
  static float _angle_pitch;
  static float _angle_roll;
  static float _angle_pitch_acc;
  static float _angle_roll_acc;
  static bool _set_gyro_angles;
  static float _angle;
  static float _angle_pitch_output;
  static float _angle_roll_output;
  static float error;
  static float integral;
  static float derivative;
  static float last_error;
  static esp_timer_handle_t _timer;
};

#endif // ARMORIAL_SUASSUNA_MPU_H
