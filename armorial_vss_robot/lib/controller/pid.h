#ifndef ARMORIAL_SUASSUNA_PID_H
#define ARMORIAL_SUASSUNA_PID_H

#include "pid_lib.h"

#define ENCODER_RESOLUTION 20000 // microsseconds

#define MAX_VEL 30.f

class PID_velocity {
public:
  PID_velocity();
  void setConsts(double kp, double ki, double kd);
  void setInput(double input) { _Input = input; }
  void setSetPoint(double setpoint) { _setPoint = setpoint; }
  static void setForce(bool force);
  double getOutput() { return _Output; }
  void update();

private:
  PID *_wheel;
  double _Kp = 1.0f, _Ki = 0.0f, _Kd = 0.0f;
  double _Input, _setPoint, _Output;
};

#endif // ARMORIAL_SUASSUNA_PID_H
