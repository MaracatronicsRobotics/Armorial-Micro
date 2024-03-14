#ifndef ARMORIAL_SUASSUNA_CONTROLLER_H
#define ARMORIAL_SUASSUNA_CONTROLLER_H

#include <pid.h>
#include <encoder.h>
#include <esp32-hal-ledc.h>
#include <mpu.h>
#include <packets/packets.h>
#include <utils.h>

#define WHEEL_LEFT_FORWARD_PIN 26
#define WHEEL_LEFT_BACKWARD_PIN 25
#define WHEEL_RIGHT_FORWARD_PIN 33
#define WHEEL_RIGHT_BACKWARD_PIN 32

#define WHEEL_LEFT_FORWARD_PIN_ID 1
#define WHEEL_LEFT_BACKWARD_PIN_ID 2
#define WHEEL_RIGHT_FORWARD_PIN_ID 3
#define WHEEL_RIGHT_BACKWARD_PIN_ID 4

#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8

#define SATURATION_VALUE 250.0f

class Controller {
public:
  Controller(Encoder *encoder);

  void setControlPacket(const ControlPacket &controlPacket);
  void setLastControlPacket(const ControlPacket &controlPacket);
  ControlPacket getControlPacket();
  int getPWMConversion(float radianSpeed);
  float getVX(float leftWheelVelocity, float rightWheelVelocity);
  float getVW(float leftWheelVelocity, float rightWheelVelocity);

  float getRightPWM() { return _rightPWM; }
  float getLeftPWM() { return _leftPWM; }

  void drive();

  static void setupPWMPins();

private:
  Encoder *_encoder;
  MPU *_mpu;
  ControlPacket _control_packet;
  LastControlPacket _last_control_packet;
  PID *_leftWheel;
  PID *_rightWheel;
  PID *_mpu_pid;
  float _rightPWM;
  float _leftPWM;
};

#endif // ARMORIAL_SUASSUNA_CONTROLLER_H
