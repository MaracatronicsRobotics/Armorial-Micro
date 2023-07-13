#include "controller.h"
#include <algorithm>
#include <interpolation.h>

// Interpolate points (radSec, PWM)
double interpolate_x[] = {
    3.f,         4.796164779, 7.414158654, 10.62905513, 15.55088362,
    19.17418714, 22.15870016, 24.50442267, 26.51504197, 27.41563186,
    28.64085299, 29.67757857, 30.72477612, 31.35309465, 32.18038071,
    32.71445146, 33.11238653, 33.5103216,  33.86636877, 34.22241593,
    34.54704717, 34.97639817, 35.35338929, 35.47905299, 36.36917091};
double interpolate_y[] = {0.f,   40.0,  45.0,  50.0,  60.0,  70.0,  80.0,
                          90.0,  100.0, 110.0, 120.0, 130.0, 140.0, 150.0,
                          160.0, 170.0, 180.0, 190.0, 200.0, 210.0, 220.0,
                          230.0, 240.0, 250.0, 255.0};

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

  _wheel1 = new PID_velocity();
  _wheel2 = new PID_velocity();
}

void Controller::setControlPacket(const ControlPacket &controlPacket) {
  _control_packet = controlPacket;
}

ControlPacket Controller::getControlPacket() { return _control_packet; }

void Controller::drive() {

  Serial.print("\n\n\nWheel2: ");
  Serial.println(String(getControlPacket().vw2));

  _wheel1->setSetPoint(abs(getControlPacket().vw1));
  _wheel2->setSetPoint(abs(getControlPacket().vw2));

  _wheel1->setInput(abs(_encoder->getAngularSpeedWL()));
  _wheel2->setInput(abs(_encoder->getAngularSpeedWR()));
  Serial.print("Wheel2 Encoder: ");
  Serial.println(String(_encoder->getAngularSpeedWR()));

  // PID compute
  _wheel1->update();
  _wheel2->update();
  PID_velocity::setForce(false);

  Serial.print("Wheel2 PID: ");
  Serial.println(String(_wheel2->getOutput()));

  // Make conversion from rad/s to PWM
  int wheel_pwm_left = int(round(Interpolation::ConstrainedSpline(
      interpolate_x, interpolate_y, interpolate_numPoints,
      _wheel1->getOutput())));
  int wheel_pwm_right = int(round(Interpolation::ConstrainedSpline(
      interpolate_x, interpolate_y, interpolate_numPoints,
      _wheel2->getOutput())));

  Serial.print("Wheel2 PWM: ");
  Serial.println(String(wheel_pwm_right));

  // Set PWM pin values
  ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID,
            getControlPacket().vw1 > 0 ? wheel_pwm_left : 0);
  ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID,
            getControlPacket().vw1 <= 0 ? wheel_pwm_left : 0);

  ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID,
            getControlPacket().vw2 > 0 ? wheel_pwm_right : 0);
  ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID,
            getControlPacket().vw2 <= 0 ? wheel_pwm_right : 0);
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