#include "mpu.h"

MPU6050 MPU::_mpu = MPU6050(Wire);
SimpleKalmanFilter MPU::_kalman = SimpleKalmanFilter(0.1, 0.1, 0.01);
float MPU::_mpuAngularSpeed = 0.0f;
float MPU::_gyro_x = 0.0f;
float MPU::_gyro_y = 0.0f;
float MPU::_gyro_z = 0.0f;
esp_timer_handle_t MPU::_timer = {};
bool MPU::_startedTimer = false;

void MPU::setup() {
  Wire.begin();
  _mpu.begin();
  _mpu.calcGyroOffsets();

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
  _mpu.update();
  _gyro_x = (_mpu.getGyroX() * (180 / M_PI));
  _gyro_y = (_mpu.getGyroY() * (180 / M_PI));
  _gyro_z = _kalman.updateEstimate((_mpu.getGyroZ() * (180 / M_PI)));
}

float MPU::getGyroX() { return _gyro_x; }

float MPU::getGyroY() { return _gyro_y; }

float MPU::getGyroZ() { return _gyro_z; }

void MPU::computeMPUCallback(void *arg) { readFromMPU(); }