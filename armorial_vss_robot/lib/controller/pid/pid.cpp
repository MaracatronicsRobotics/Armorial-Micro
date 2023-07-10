#include "pid.h"

PID_velocity::PID_velocity() {
  _wheel = new PID(&_Input, &_Output, &_setPoint, _Kp, _Ki, _Kd, DIRECT);
  _wheel->SetMode(AUTOMATIC);
  _wheel->SetOutputLimits(0.f, MAX_VEL);
  _wheel->SetSampleTime(ENCODER_RESOLUTION / 1000);
}

void PID_velocity::setConsts(double kp, double ki, double kd) {
  _Kp = kp;
  _Ki = ki;
  _Kd = kd;
}

void PID_velocity::update() {
  _wheel->Compute(false, _computingForce);
  setForce(false);
}