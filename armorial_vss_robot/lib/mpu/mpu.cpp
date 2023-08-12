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

  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    readFromMPURaw(); 
    //Add the gyro x offset to the gyro_x_cal variable                                            
    gyro_x_cal += _gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable                                              
    gyro_y_cal += _gyro_y; 
    //Add the gyro z offset to the gyro_z_cal variable                                             
    gyro_z_cal += _gyro_z; 
    //Delay 3us to have 250Hz for-loop                                             
    delay(3);                                            
}
 // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;
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
  _gyro_x = g.gyro.x - gyro_x_cal;
  _gyro_y = g.gyro.y - gyro_y_cal;
  _gyro_z = g.gyro.z - gyro_z_cal;
}

void MPU::readFromMPURaw() {
  sensors_event_t a, g, temp;
  _mpu.getEvent(&a, &g, &temp);
  _gyro_x = g.gyro.x;
  _gyro_y = g.gyro.y;
  _gyro_z = g.gyro.z;
}

float MPU::getGyroX() { return _gyro_x; }

float MPU::getGyroY() { return _gyro_y; }

float MPU::getGyroZ() { return _gyro_z; }

void MPU::computeMPUCallback(void *arg) { readFromMPU(); }