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
  int leftPWM = 0;
  int rightPWM = 0;

  if (fabs(getControlPacket().vx) != 0.00f ||
      fabs(getControlPacket().vw) != 0.0f) {
    float vx = getControlPacket().vx;
    float vw = getControlPacket().vw;

    // Convertendo velocidades do robô em velocidades de roda
    float velR = (2 * vx + WHEELS_DISTANCE * vw) / WHEELS_DIAMETER;
    float velL = (2 * vx - WHEELS_DISTANCE * vw) / WHEELS_DIAMETER;

    Serial.print("Vx: ");
    Serial.println(vx);
    Serial.print("Vw: ");
    Serial.println(vw);
    Serial.print("VelR: ");
    Serial.println(velR);
    Serial.print("VelL: ");
    Serial.println(velL);

    // Obtendo respostas dos encoders
    float rightEncoder = _encoder->getAngularSpeedWR();
    float leftEncoder = _encoder->getAngularSpeedWL();

    // Definindo relações lineares e angulares das rodas
    float deltaW = rightEncoder - leftEncoder;
    float sumW = rightEncoder + leftEncoder;
    // if (vx < 0.0f) deltaW = leftEncoder - rightEncoder;
    // else if (vx >= 0.0f) deltaW = rightEncoder - leftEncoder;

    // Definição da realimentação
    float realRightWheel = (sumW + deltaW) / 2;
    float realLeftWheel = (sumW - deltaW) / 2;
    Serial.print("realRightWheel: ");
    Serial.println(realRightWheel);
    Serial.print("realLeftWheel: ");
    Serial.println(realLeftWheel);
    // if (vx >= 0.0f) {
    //   realRightWheel = (sumW + deltaW) / 2;
    //   realLeftWheel = (sumW - deltaW) / 2;
    // } else if (vx < 0.0f) {
    //   realRightWheel = (sumW - deltaW) / 2;
    //   realLeftWheel = (sumW + deltaW) / 2;
    // }

    // Set PIDs setpoints
    _rightWheel->setSetPoint(velR);
    _leftWheel->setSetPoint(velL);

    // Set PIDs actual values
    _rightWheel->setActualValue(realRightWheel);
    _leftWheel->setActualValue(realLeftWheel);

    // Set PIDs constants
    _rightWheel->setConstants(5.0f, 0.3f, 0.0f);
    _leftWheel->setConstants(5.0f, 0.3f, 0.0f);
    // if (velR > 28.0f) {
    //   _rightWheel->setConstants(3.5f, 45.0f, 0.0f);
    // } else {
    //   _rightWheel->setConstants(4.5f, 30.0f, 0.0f);
    // }
    // if (velL > 28.0f) {
    //   _leftWheel->setConstants(3.5f, 0.0f, 0.0f);
    // } else {
    //   _leftWheel->setConstants(4.5f, 30.0f, 0.0f);
    // }

    // Get PIDs outputs
    float rightOutput = _rightWheel->getOutput();
    float leftOutput = _leftWheel->getOutput();

    float rightError = _rightWheel->getError();
    float rightI = _rightWheel->getI();
    // float leftError = _rightWheel->getError();
    // float leftI = _rightWheel->getI();

    Serial.print("Error: ");
    Serial.println(rightError);
    Serial.print("rightI: ");
    Serial.println(rightI);

    Serial.print("rightOutput: ");
    Serial.println(rightOutput);
    Serial.print("leftOutput: ");
    Serial.println(leftOutput);

    // Creating PWM convertion
    rightPWM = getPWMConversion(fabs(rightOutput));
    leftPWM = getPWMConversion(fabs(leftOutput));

    // Defining wheels dead zone
    if (fabs(rightPWM) < 50) rightPWM = 50;
    if (fabs(leftPWM) < 50) leftPWM = 50;

    // Defining side of movement
    rightPWM = (velR > 0.0f) ? rightPWM : -rightPWM;
    leftPWM = (velL > 0.0f) ? leftPWM : -leftPWM;

    Serial.print("rightPWM: ");
    Serial.println(rightPWM);
    Serial.print("leftPWM: ");
    Serial.println(leftPWM);

    // Defining right wheel saturation
    rightPWM = std::min(rightPWM, 255);
    rightPWM = std::max(rightPWM, -255);
    
    // Defining left wheel saturation
    leftPWM = std::min(leftPWM, 255);
    leftPWM = std::max(leftPWM, -255);

    Serial.print("rightPWM 2: ");
    Serial.println(rightPWM);
    Serial.print("leftPWM 2: ");
    Serial.println(leftPWM);

    _rightPWM = rightPWM;
    _leftPWM = leftPWM;

    // Set PWM pin values
    ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID, leftPWM >= 0 ? fabs(leftPWM) : 0);
    ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID, leftPWM < 0 ? fabs(leftPWM) : 0);

    ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID, rightPWM >= 0 ? fabs(rightPWM) : 0);
    ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID, rightPWM < 0 ? fabs(rightPWM) : 0);
  } else {
    // Set PWM pin values
    ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID, 0);
    ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID, 0);

    ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID, 0);
    ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID, 0);
  }
  setLastControlPacket(getControlPacket());
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

int Controller::getPWMConversion(float radianSpeed) {
  return static_cast<int>(0.000029f * pow(radianSpeed, 5) - 
    0.001367f * pow(radianSpeed, 4) + 0.001111f * pow(radianSpeed, 3) + 
    0.78095f * pow(radianSpeed, 2) - 9.802875f * radianSpeed + 83.592186f);
}

float Controller::getVX(float leftWheelVelocity, float rightWheelVelocity) {
  return (leftWheelVelocity + rightWheelVelocity) * 0.052f / 4;
}

float Controller::getVW(float leftWheelVelocity, float rightWheelVelocity) {
  return (rightWheelVelocity - leftWheelVelocity) * 0.052f / (2 * 0.075f);
}