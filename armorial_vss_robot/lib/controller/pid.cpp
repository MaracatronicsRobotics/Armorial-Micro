#include "pid.h"

bool _computingForce;

PID_velocity::PID_velocity() {
  _wheel = new PID(&_Input, &_Output, &_setPoint, _Kp, _Ki, _Kd, P_ON_M, DIRECT);
  _wheel->SetMode(AUTOMATIC);
  _wheel->SetOutputLimits(0.f, MAX_VEL);
  _wheel->SetSampleTime(ENCODER_RESOLUTION / 1000);
  _computingForce = false;
}

void PID_velocity::setConsts(double kp, double ki, double kd) {
  _Kp = kp;
  _Ki = ki;
  _Kd = kd;
}

void PID_velocity::setForce(bool force) { _computingForce = force; }

void PID_velocity::update() { _wheel->Compute(false, _computingForce); }