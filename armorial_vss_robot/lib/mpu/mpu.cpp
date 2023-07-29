#include "mpu.h"

Adafruit_MPU6050 MPU::_mpu = Adafruit_MPU6050();
float MPU::_mpuAngularSpeed = 0.0f;
float MPU::_gyro_x_calibration = 0.0f;
float MPU::_gyro_y_calibration = 0.0f;
float MPU::_gyro_z_calibration = 0.0f;
float MPU::_gyro_x = 0.0f;
float MPU::_gyro_y = 0.0f;
float MPU::_gyro_z = 0.0f;
float MPU::_acc_x = 0.0f;
float MPU::_acc_y = 0.0f;
float MPU::_acc_z = 0.0f;
float MPU::_angle_pitch = 0.0f;
float MPU::_angle_roll = 0.0f;
float MPU::_angle_pitch_acc = 0.0f;
float MPU::_angle_roll_acc = 0.0f;
bool MPU::_set_gyro_angles = false;
float MPU::_angle = 0.0f;
float MPU::_angle_pitch_output = 0.0f;
float MPU::_angle_roll_output = 0.0f;
float MPU::error = 0.0f;
float MPU::integral = 0.0f;
float MPU::derivative = 0.0f;
float MPU::last_error = 0.0f;
esp_timer_handle_t MPU::_timer = {};

void MPU::setup() {
  _mpu.begin();
  _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  _mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Read the raw acc and gyro data from the MPU-6050 1000 times
  for (int cal_int = 0; cal_int < 1000; cal_int++) {
    readFromMPU();
    // Add the gyro x offset to the gyro_x_cal variable
    _gyro_x_calibration += _gyro_x;
    // Add the gyro y offset to the gyro_y_cal variable
    _gyro_y_calibration += _gyro_y;
    // Add the gyro z offset to the gyro_z_cal variable
    _gyro_z_calibration += _gyro_z;
    // Delay 3us to have 250Hz for-loop
    delay(3);
  }

  // Divide all results by 1000 to get average offset
  _gyro_x_calibration /= 1000;
  _gyro_y_calibration /= 1000;
  _gyro_z_calibration /= 1000;

  // Setup timer
  esp_timer_create_args_t encoder_timer_args = {
      .callback = &MPU::computeMPUCallback, .name = "mpu"};
  ESP_ERROR_CHECK(esp_timer_create(&encoder_timer_args, &_timer));
}

void MPU::start() {
  ESP_ERROR_CHECK(esp_timer_start_periodic(_timer, MPU_RESOLUTION));
}

void MPU::stop() {
  ESP_ERROR_CHECK(esp_timer_stop(_timer));
  integral = 0.0f;
}

void MPU::readFromMPU() {
  sensors_event_t a, g, temp;
  _mpu.getEvent(&a, &g, &temp);
  _gyro_x = g.gyro.x;
  _gyro_y = g.gyro.y;
  _gyro_z = g.gyro.z;
  _acc_x = a.acceleration.x;
  _acc_y = a.acceleration.y;
  _acc_z = a.acceleration.z;
}

float MPU::getComputedAngleCorrection() { return _angle; }

float MPU::getComputedPitchOutput() { return _angle_pitch_output; }

void MPU::computeMPUCallback(void *arg) {
  readFromMPU();
  _gyro_x -= _gyro_x_calibration;
  _gyro_y -= _gyro_y_calibration;
  _gyro_z -= _gyro_z_calibration;

  // Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  //  Calculate the traveled pitch angle and add this to the angle_pitch
  //  variable
  _angle_pitch += _gyro_x * 0.0000611;
  // Calculate the traveled roll angle and add this to the angle_roll variable
  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is
  // in radians
  _angle_roll += _gyro_y * 0.0000611;

  // If the IMU has yawed transfer the roll angle to the pitch angle
  _angle_pitch += _angle_roll * sin(_gyro_z * 0.000001066);
  // If the IMU has yawed transfer the pitch angle to the roll angle
  _angle_roll -= _angle_pitch * sin(_gyro_z * 0.000001066);

  // Accelerometer angle calculations

  // Calculate the total accelerometer vector
  float acc_total_vector =
      sqrt((_acc_x * _acc_x) + (_acc_y * _acc_y) + (_acc_z * _acc_z));

  // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  // Calculate the pitch angle
  _angle_pitch_acc = asin((float)_acc_y / acc_total_vector) * 57.296;
  // Calculate the roll angle
  _angle_roll_acc = asin((float)_acc_x / acc_total_vector) * -57.296;

  // Accelerometer calibration value for pitch
  _angle_pitch_acc -= 0.0;
  // Accelerometer calibration value for roll
  _angle_roll_acc -= 0.0;

  if (_set_gyro_angles) {

    // If the IMU has been running
    // Correct the drift of the gyro pitch angle with the accelerometer pitch
    // angle
    _angle_pitch = _angle_pitch * 0.9996 + _angle_pitch_acc * 0.0004;
    // Correct the drift of the gyro roll angle with the accelerometer roll
    // angle
    _angle_roll = _angle_roll * 0.9996 + _angle_roll_acc * 0.0004;
  } else {
    // IMU has just started
    // Set the gyro pitch angle equal to the accelerometer pitch angle
    _angle_pitch = _angle_pitch_acc;
    // Set the gyro roll angle equal to the accelerometer roll angle
    _angle_roll = _angle_roll_acc;
    // Set the IMU started flag
    _set_gyro_angles = true;
  }

  // To dampen the pitch and roll angles a complementary filter is used
  // Take 90% of the output pitch value and add 10% of the raw pitch value
  _angle_pitch_output = _angle_pitch_output * 0.9 + _angle_pitch * 0.1;
  // Take 90% of the output roll value and add 10% of the raw roll value
  _angle_roll_output = _angle_roll_output * 0.9 + _angle_roll * 0.1;
  // Wait until the loop_timer reaches 4000us (250Hz) before starting the next
  // loop
  //-----------------done with mpu6050
  // calibration--------------------------------------//
  error = 0 - _angle_pitch_output; // proportional
  integral = integral + error;     // integral
  derivative = error - last_error; // derivative

  last_error = error;

  _angle = (error * MPU_KP) + (integral * MPU_KI) + (derivative * MPU_KD);

  integral = integral * 0.9;

  Serial.print("angle_pitch_output = ");
  Serial.println(_angle_pitch_output);
  Serial.print("angulo = ");
  Serial.println(_angle);
}