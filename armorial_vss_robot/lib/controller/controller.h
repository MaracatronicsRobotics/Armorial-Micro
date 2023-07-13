#ifndef ARMORIAL_SUASSUNA_CONTROLLER_H
#define ARMORIAL_SUASSUNA_CONTROLLER_H

#include "pid.h"
#include <encoder.h>
#include <esp32-hal-ledc.h>
#include <packets/packets.h>

#define WHEEL_LEFT_FORWARD_PIN 12
#define WHEEL_LEFT_BACKWARD_PIN 13
#define WHEEL_RIGHT_FORWARD_PIN 2
#define WHEEL_RIGHT_BACKWARD_PIN 26

#define PIN_H_BRIDGE 27

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
  ControlPacket getControlPacket();

  void drive();

  static void setupPWMPins();
  static void setupHBridge();

private:
  Encoder *_encoder;
  ControlPacket _control_packet;
  PID_velocity *_wheel1;
  PID_velocity *_wheel2;
};

#endif // ARMORIAL_SUASSUNA_CONTROLLER_H
