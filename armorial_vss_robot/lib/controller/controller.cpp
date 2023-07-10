#include "controller.h"
#include "encoder.h"

#include <InterpolationLib.h>
#include <algorithm>

// Interpolate points (radSec, PWM)
double interpolate_x[] = {5.f,
                          10.471975499999981,
                          16.54572128999997,
                          20.943950999999963,
                          24.504422669999958,
                          27.22713629999995,
                          29.21681164499995,
                          30.682888214999945,
                          32.04424502999994,
                          33.19616233499994,
                          34.033920374999944,
                          34.97639816999994,
                          35.60471669999994,
                          36.33775498499994,
                          36.861353759999936,
                          37.38495253499993,
                          37.69911179999993,
                          38.11799081999993,
                          38.43215008499993,
                          38.74630934999993,
                          39.060468614999934,
                          39.374627879999935,
                          39.68878714499993,
                          40.f};

double interpolate_y[] = {0.f,   40.0,  50.0,  60.0,  70.0,  80.0,
                          90.0,  100.0, 110.0, 120.0, 130.0, 140.0,
                          150.0, 160.0, 170.0, 180.0, 190.0, 200.0,
                          210.0, 220.0, 230.0, 240.0, 250.0, 255.0};

int interpolate_numPoints = sizeof(interpolate_x) / sizeof(interpolate_x[0]);

Controller::Controller(Encoder *encoder) : _encoder(encoder) {
  ControlPacket controlPacket;
  controlPacket.control = ROBOT_ID;
  controlPacket.crc = 0;
  controlPacket.solenoidPower = 0;
  controlPacket.vw1 = 0;
  controlPacket.vw2 = 0;
  controlPacket.vw3 = 0;
  controlPacket.vw4 = 0;
  setControlPacket(controlPacket);
}

void Controller::setControlPacket(const ControlPacket &controlPacket) {
  _control_packet = controlPacket;
}

ControlPacket Controller::getControlPacket() { return _control_packet; }

/// TODO: use PID class with captured Encoder class data
void Controller::drive() {
  // Make conversion from rad/s to PWM
  float wheel_left_setpoint =
      getControlPacket().vw1 > 0
          ? Interpolation::ConstrainedSpline(interpolate_x, interpolate_y,
                                             interpolate_numPoints,
                                             getControlPacket().vw1)
          : -Interpolation::ConstrainedSpline(interpolate_x, interpolate_y,
                                              interpolate_numPoints,
                                              abs(getControlPacket().vw1));
  float wheel_right_setpoint =
      getControlPacket().vw2 > 0
          ? Interpolation::ConstrainedSpline(interpolate_x, interpolate_y,
                                             interpolate_numPoints,
                                             getControlPacket().vw2)
          : -Interpolation::ConstrainedSpline(interpolate_x, interpolate_y,
                                              interpolate_numPoints,
                                              abs(getControlPacket().vw2));

  // // Saturation
  // wheel_left_setpoint = std::max(
  //     -SATURATION_VALUE, std::min(wheel_left_setpoint, SATURATION_VALUE));
  // wheel_right_setpoint = std::max(
  //     -SATURATION_VALUE, std::min(wheel_right_setpoint, SATURATION_VALUE));

  // Set PWM pin values
  ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID,
            wheel_left_setpoint > 0 ? int(wheel_left_setpoint) : 0);
  ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID,
            wheel_left_setpoint <= 0 ? int(-wheel_left_setpoint) : 0);

  ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID,
            wheel_right_setpoint > 0 ? int(wheel_right_setpoint) : 0);
  ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID,
            wheel_right_setpoint <= 0 ? int(-wheel_right_setpoint) : 0);
}

void Controller::setupPWMPins() {
  ledcSetup(WHEEL_LEFT_FORWARD_PIN_ID, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(WHEEL_LEFT_BACKWARD_PIN_ID, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(WHEEL_RIGHT_FORWARD_PIN_ID, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(WHEEL_RIGHT_BACKWARD_PIN_ID, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(WHEEL_LEFT_FORWARD_PIN, WHEEL_LEFT_FORWARD_PIN_ID);
  ledcAttachPin(WHEEL_LEFT_BACKWARD_PIN, WHEEL_LEFT_BACKWARD_PIN_ID);
  ledcAttachPin(WHEEL_RIGHT_FORWARD_PIN, WHEEL_RIGHT_FORWARD_PIN_ID);
  ledcAttachPin(WHEEL_RIGHT_BACKWARD_PIN, WHEEL_RIGHT_BACKWARD_PIN_ID);
}

void Controller::setupHBridge() {
  pinMode(PIN_H_BRIDGE, OUTPUT);
  digitalWrite(PIN_H_BRIDGE, HIGH);
}