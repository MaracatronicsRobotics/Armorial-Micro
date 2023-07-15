#ifndef ARMORIAL_SUASSUNA_PID_H
#define ARMORIAL_SUASSUNA_PID_H

#include "pid_lib.h"

#define ENCODER_RESOLUTION 10000 // microsseconds

#define MAX_VEL 40.f

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
  double _Kp = 0.4f, _Ki = 5.97f, _Kd = 0.f;
  double _Input, _setPoint, _Output;
};

#endif // ARMORIAL_SUASSUNA_PID_H
