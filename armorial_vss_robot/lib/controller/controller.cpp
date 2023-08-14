#include "controller.h"
#include <algorithm>
#include <interpolation.h>

// Interpolate points (radSec, PWM)
double interpolate_x[] = {0.0,  3.69, 5.0,  10.0, 13.0,  16.0,  20.0, 22.0,
                          23.0, 24.0, 25.0, 26.0, 26.92, 28.72, 30.0};
double interpolate_y[] = {0.0,   60.0,  70.0,  80.0,  90.0,
                          100.0, 110.0, 120.0, 130.0, 140.0,
                          150.0, 170.0, 190.0, 220.0, 255.0};

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

  _wheel1 = new PID();
  _wheel2 = new PID();
  _mpu_pid = new PID();

  _mpu = new MPU();
}

void Controller::setControlPacket(const ControlPacket &controlPacket) {
  _control_packet = controlPacket;
}

ControlPacket Controller::getControlPacket() { return _control_packet; }

void Controller::setLastControlPacket(const ControlPacket &controlPacket) {
  _last_control_packet.solenoidPower = controlPacket.solenoidPower;
  _last_control_packet.vw1 = controlPacket.vw1;
  _last_control_packet.vw2 = controlPacket.vw2;
  _last_control_packet.vw3 = controlPacket.vw3;
  _last_control_packet.vw4 = controlPacket.vw4;
}

void Controller::drive() {
  float velR = 0.0f;
  float velL = 0.0f;

  if (fabs(getControlPacket().vx) >= 0.01f) {
    float vx = getControlPacket().vx;
    float vw = getControlPacket().vw * (180.0f / M_PI);

    _mpu_pid->setActualValue(_mpu->getGyroZDeg());
    _mpu_pid->setSetPoint(vw);
    float linear = 0.0f;
    float angular = _mpu_pid->getOutput();

    if (vx >= 0.0f) {
      linear = map(vx, 0.0f, 1.0f, 50, 255);
    } else {
      linear = map(vx, -1.0f, 0.0f, -255, -50);
    }

    velR = linear - angular;
    velL = linear + angular;

    if (velR < 15 && velR > -15)
      velR = 0;
    if (velR > 255)
      velR = 255;
    if (velR < -255)
      velR = -255;

    if (velL < 15 && velL > -15)
      velL = 0;
    if (velL > 255)
      velL = 255;
    if (velL < -255)
      velL = -255;
  }

  setLastControlPacket(getControlPacket());

  // Set PWM pin values
  ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID, velL > 0 ? abs(velL) : 0);
  ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID, velL <= 0 ? abs(velL) : 0);

  ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID, velR > 0 ? abs(velR) : 0);
  ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID, velR <= 0 ? abs(velR) : 0);
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

float Controller::getVX(float leftWheelVelocity, float rightWheelVelocity) {
  return (leftWheelVelocity + rightWheelVelocity) * 0.053f / 4;
}

float Controller::getVW(float leftWheelVelocity, float rightWheelVelocity) {
  return (rightWheelVelocity - leftWheelVelocity) * 0.053f / (2 * 0.075f);
}