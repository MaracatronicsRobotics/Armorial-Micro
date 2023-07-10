#ifndef ARMORIAL_SUASSUNA_PID_H
#define ARMORIAL_SUASSUNA_PID_H

#include "controller.h"
#include <PID_v1.h>

#define MAX_VEL 40.f

// PID parameters

class PID_velocity {
public:
  PID_velocity();
  void setConsts(double kp, double ki, double kd);
  void setInput(double input) { _Input = input; }
  void setSetPoint(double setpoint) { _setPoint = setpoint; }
  void setForce(bool force) { _computingForce = force; }
  double getOutput() { return _Output; }
  void update();

private:
  PID *_wheel;
  double _Kp = 5.f, _Ki = 2.f, _Kd = 0.f;
  double _Input, _setPoint, _Output;
  bool _computingForce = false;
};

#endif // ARMORIAL_SUASSUNA_PID_H
