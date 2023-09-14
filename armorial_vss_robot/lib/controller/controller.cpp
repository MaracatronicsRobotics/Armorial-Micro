#include "controller.h"
#include <algorithm>
#include <interpolation.h>

#define WHEELS_DISTANCE 0.075f
#define WHEELS_DIAMETER 0.052f

Controller::Controller(Encoder *encoder) : _encoder(encoder) {
  ControlPacket controlPacket;
  controlPacket.control = ROBOT_ID;
  controlPacket.crc = 0;
  controlPacket.solenoidPower = 0;
  controlPacket.vx = 0;
  controlPacket.vw = 0;
  setControlPacket(controlPacket);

  _leftWheel = new PID();
  _rightWheel = new PID();
  _mpu_pid = new PID();

  _mpu_pid->setConstants(1.0f, 0.0f, 0.0f);
  _mpu_pid->setOutputLimits(-50, 50);

  _mpu = new MPU();
}

void Controller::setControlPacket(const ControlPacket &controlPacket) {
  _control_packet = controlPacket;
}

ControlPacket Controller::getControlPacket() { return _control_packet; }

void Controller::setLastControlPacket(const ControlPacket &controlPacket) {
  _last_control_packet.solenoidPower = controlPacket.solenoidPower;
  _last_control_packet.vx = controlPacket.vx;
  _last_control_packet.vy = controlPacket.vy;
  _last_control_packet.vw = controlPacket.vw;
}

void Controller::drive() {
  float velR = 0.0f;
  float velL = 0.0f;

  if (fabs(getControlPacket().vx) >= 0.01f ||
      fabs(getControlPacket().vw) >= 0.20) {
    float vx = getControlPacket().vx;
    float vw = getControlPacket().vw;

  // // if (abs(_last_control_packet.vx - vx) > VX_MAX_VARIANCE) {
  //   //   _motor1->resetPID();
  //   //   _motor2->resetPID();

  //   //   _motor1->calculatePWM();
  //   //   _motor2->calculatePWM();
  //   // }
  //   // _mpu_pid->setActualValue(_mpu->getGyroZ());
  //   // _mpu_pid->setSetPoint(vw);
  //   // float linear = 0.0f;
  //   // float angular;
    
  //   // if(fabs(vw)<=2){
  //   //   angular = 0;
  //   // }else{
  //   //   angular = vw + _mpu_pid->getOutput();
  //   // }

  //   // if (vx >= 0.0f) {
  //   //   linear = Utils::fmap(vx, 0.0f, 1.0f, 0.0f, 255.0f);
  //   // } else {
  //   //   linear = Utils::fmap(vx, -1.0f, 0.0f, -255.0f, 0.0f);
  //   // }

  //   // velR = -linear + angular;
  //   // velL = -linear - angular;

    velR = (2 * vx + WHEELS_DISTANCE * vw) / WHEELS_DIAMETER;
    velL = (2 * vx - WHEELS_DISTANCE * vw) / WHEELS_DIAMETER;
  }

  setLastControlPacket(getControlPacket());

  // Obtendo respostas dos encoders
  float rightEncoder = _encoder->getAngularSpeedWR();
  float leftEncoder = _encoder->getAngularSpeedWL();

  // Definindo relações lineares e angulares das rodas
  float deltaW = rightEncoder - leftEncoder;
  float sumW = rightEncoder + leftEncoder;

  // Definição da realimentação
  float realRightWheel = (sumW + deltaW) / 2;
  float realLeftWheel = (sumW - deltaW) / 2;

  // Set PIDs setpoints
  _rightWheel->setSetPoint(velR);
  _leftWheel->setSetPoint(velL);

  // Set PIDs actual values
  _rightWheel->setActualValue(realRightWheel);
  _leftWheel->setActualValue(realLeftWheel);

  // Set PIDs constants
  _rightWheel->setConstants(100.0f, 0.0f, 0.0f);
  _leftWheel->setConstants(100.0f, 0.0f, 0.0f);

  // Get PIDs outputs
  float rightOutput = _rightWheel->getOutput();
  float leftOutput = _leftWheel->getOutput();

  // Creating PWM convertion
  int rightPWM = getPWMConversion(rightOutput);
  int leftPWM = getPWMConversion(leftOutput);

  // Defining wheels dead zone
  if (fabs(rightPWM) < 50) rightPWM = 0;
  if (fabs(leftPWM) < 50) leftPWM = 0;

  // Defining right wheel saturation
  rightPWM = std::min(rightPWM, 255);
  rightPWM = std::max(rightPWM, -255);
  
  // Defining left wheel saturation
  leftPWM = std::min(leftPWM, 255);
  leftPWM = std::max(leftPWM, -255);

  // Set PWM pin values
  ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID, leftPWM > 0 ? fabs(leftPWM) : 0);
  ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID, leftPWM <= 0 ? fabs(leftPWM) : 0);

  ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID, rightPWM > 0 ? fabs(rightPWM) : 0);
  ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID, rightPWM <= 0 ? fabs(rightPWM) : 0);
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

float Controller::getPWMConversion(float radianSpeed) {
  return static_cast<int>(0.0113f * pow(radianSpeed, 3) - 
    0.5414f * pow(radianSpeed, 2) + 9.1374f * radianSpeed - 3.3815f);
}

float Controller::getVX(float leftWheelVelocity, float rightWheelVelocity) {
  return (leftWheelVelocity + rightWheelVelocity) * 0.052f / 4;
}

float Controller::getVW(float leftWheelVelocity, float rightWheelVelocity) {
  return (rightWheelVelocity - leftWheelVelocity) * 0.052f / (2 * 0.075f);
}