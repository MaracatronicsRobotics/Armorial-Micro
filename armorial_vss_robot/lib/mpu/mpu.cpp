#include "mpu.h"

Adafruit_MPU6050 MPU::_mpu = Adafruit_MPU6050();
float MPU::_mpuAngularSpeed = 0.0f;
float MPU::_gyro_x = 0.0f;
float MPU::_gyro_y = 0.0f;
float MPU::_gyro_z = 0.0f;
esp_timer_handle_t MPU::_timer = {};
bool MPU::_startedTimer = false;

void MPU::setup() {
  _mpu.begin();
  _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  _mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  // Setup timer
  esp_timer_create_args_t encoder_timer_args = {
      .callback = &MPU::computeMPUCallback, .name = "mpu"};
  ESP_ERROR_CHECK(esp_timer_create(&encoder_timer_args, &_timer));
}

void MPU::start() {
  if (!_startedTimer) {
    ESP_ERROR_CHECK(esp_timer_start_periodic(_timer, MPU_RESOLUTION));
    _startedTimer = true;
  }
}

void MPU::stop() {
  if (_startedTimer) {
    ESP_ERROR_CHECK(esp_timer_stop(_timer));
    _startedTimer = false;
  }
}

void MPU::readFromMPU() {
  sensors_event_t a, g, temp;
  _mpu.getEvent(&a, &g, &temp);
  _gyro_x = g.gyro.x;
  _gyro_y = g.gyro.y;
  _gyro_z = g.gyro.z;
}

float MPU::getGyroX() { return _gyro_x; }

float MPU::getGyroY() { return _gyro_y; }

float MPU::getGyroZ() { return _gyro_z; }

float MPU::getGyroXDeg() {
  if (fabs((_gyro_x * 180 / M_PI)) >= 2.0) {
    return (_gyro_x * 180 / M_PI);
  } else {
    return 0.0f;
  }
}

float MPU::getGyroYDeg() {
  if (fabs((_gyro_y * 180 / M_PI)) >= 2.0) {
    return (_gyro_y * 180 / M_PI);
  } else {
    return 0.0f;
  }
}

float MPU::getGyroZDeg() {
  if (fabs((_gyro_z * 180 / M_PI)) >= 2.0) {
    return (_gyro_z * 180 / M_PI);
  } else {
    return 0.0f;
  }
}

void MPU::computeMPUCallback(void *arg) { readFromMPU(); }