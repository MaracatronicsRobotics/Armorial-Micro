#include "controller.h"
#include "encoder.h"

#include <algorithm>

Controller::Controller(Encoder *encoder) : _encoder(encoder) { _pid = PID(); }

void Controller::setControlPacket(const ControlPacket &controlPacket) {
  _control_packet = controlPacket;
}

ControlPacket Controller::getControlPacket() { return _control_packet; }

/// TODO: use PID class with captured Encoder class data
void Controller::drive() {
  // Make conversion from rad/s to PWM
  float wheel_left_setpoint = (getControlPacket().vw1 / 32.0) * 255;
  float wheel_right_setpoint = (getControlPacket().vw2 / 32.0) * 255;

  // Saturation
  wheel_left_setpoint = std::max(
      -SATURATION_VALUE, std::min(wheel_left_setpoint, SATURATION_VALUE));
  wheel_right_setpoint = std::max(
      -SATURATION_VALUE, std::min(wheel_right_setpoint, SATURATION_VALUE));

  // Set PWM pin values
  ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID,
            wheel_left_setpoint > 0 ? int(wheel_left_setpoint) : 0);
  ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID,
            wheel_left_setpoint <= 0 ? 0 : -int(wheel_left_setpoint));

  ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID,
            wheel_right_setpoint > 0 ? int(wheel_right_setpoint) : 0);
  ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID,
            wheel_right_setpoint <= 0 ? 0 : -int(wheel_right_setpoint));
}

void Controller::setupPWMPins() {
  ledcAttachPin(WHEEL_LEFT_FORWARD_PIN, WHEEL_LEFT_FORWARD_PIN_ID);
  ledcAttachPin(WHEEL_LEFT_BACKWARD_PIN, WHEEL_LEFT_BACKWARD_PIN_ID);
  ledcAttachPin(WHEEL_RIGHT_FORWARD_PIN, WHEEL_RIGHT_FORWARD_PIN_ID);
  ledcAttachPin(WHEEL_RIGHT_BACKWARD_PIN, WHEEL_RIGHT_BACKWARD_PIN_ID);

  ledcSetup(WHEEL_LEFT_FORWARD_PIN_ID, 1000, 8);
  ledcSetup(WHEEL_LEFT_BACKWARD_PIN_ID, 1000, 8);
  ledcSetup(WHEEL_RIGHT_FORWARD_PIN_ID, 1000, 8);
  ledcSetup(WHEEL_RIGHT_BACKWARD_PIN_ID, 1000, 8);
}

void Controller::setupHBridge() {
  pinMode(PIN_H_BRIDGE, OUTPUT);
  digitalWrite(PIN_H_BRIDGE, HIGH);
}