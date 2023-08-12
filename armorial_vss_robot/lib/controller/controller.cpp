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

  float vw1_comand, vw2_comand;
  float Vel_R = 0.0f; //Velocidade final da roda direita
  float Vel_L = 0.0f; //Velocidade final da roda esquerda

  if (getControlPacket().vw1 == 0.f)
    _last_control_packet.vw1 < 0.f ? vw1_comand = -ZERO : vw1_comand = ZERO;
  else
    vw1_comand = getControlPacket().vw1;

  if (getControlPacket().vw2 == 0.f || getControlPacket().vw2 == -0.f)
    _last_control_packet.vw2 < 0.f ? vw2_comand = -ZERO : vw2_comand = ZERO;
  else
    vw2_comand = getControlPacket().vw2;

  // float vw =
  //     (getControlPacket().vw1 + getControlPacket().vw2) / (2 * 0.075f * 0.053f);

  float vx = getVX(vw1_comand, vw2_comand);
  float vw = getVW(vw1_comand, vw2_comand);
  float mappedLinearVelocity;
  float angularVelocityDegrees;
  
  if (vx > 0.0f) {
    mappedLinearVelocity = map(vx, 0.0f, 1.0f, 50, 255);
  } else {
    mappedLinearVelocity = map(vx, 0.0f, -1.0f, -50, -255);
  }
  // if (vw > 0.0f) {
  //   mappedAngularVelocity = map(vw, 0.0f, 1.0f, 1, 37);
  // } else {
  //   mappedAngularVelocity = map(vw, 0.0f, -1.0f, -1, -37);
  // }
  angularVelocityDegrees = vw * (180 / M_PI);

  // PID set points
  _mpu_pid->setSetPoint(angularVelocityDegrees);

  // PID actual values
  float actualAngularVelocity = _mpu->getGyroZ() * 180 / M_PI;
  _mpu_pid->setActualValue(actualAngularVelocity);

  // PID output
  float angularPIDOutput = _mpu_pid->getOutput();

  if(mappedLinearVelocity >= 51.0f){
    Vel_R = mappedLinearVelocity - angularPIDOutput; //ao somar o angular com linear em cada motor conseguimos a ideia de direcao do robo
    Vel_L = mappedLinearVelocity + angularPIDOutput;

  }
  

  // Make conversion from rad/s to PWM
  // int wheel_pwm_left = int(round(Interpolation::ConstrainedSpline(
  //     interpolate_x, interpolate_y, interpolate_numPoints,
  //     _wheel1->getOutput())));
  // int wheel_pwm_right = int(round(Interpolation::ConstrainedSpline(
  //     interpolate_x, interpolate_y, interpolate_numPoints,
  //     _wheel2->getOutput())));

  // int wheel_pwm_left = int(round(Interpolation::ConstrainedSpline(
  //     interpolate_x, interpolate_y, interpolate_numPoints, abs(vw1_comand))));
  // int wheel_pwm_right = int(round(Interpolation::ConstrainedSpline(
  //     interpolate_x, interpolate_y, interpolate_numPoints, abs(vw2_comand))));

  setLastControlPacket(getControlPacket());

  // Set PWM pin values
  ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID, vw1_comand > 0 ? fabs(Vel_L) : 0);
  ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID, vw1_comand <= 0 ? fabs(Vel_L) : 0);

  ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID, vw2_comand > 0 ? fabs(Vel_R) : 0);
  ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID, vw2_comand <= 0 ? fabs(Vel_R) : 0);
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

float Controller::getVX(float leftWheelVelocity, float rightWheelVelocity) {
  return (leftWheelVelocity + rightWheelVelocity) * 0.053f / 4;
}

float Controller::getVW(float leftWheelVelocity, float rightWheelVelocity) {
  return (rightWheelVelocity - leftWheelVelocity) * 0.053f / (2 * 0.075f);
}