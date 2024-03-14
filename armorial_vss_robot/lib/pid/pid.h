#ifndef ARMORIAL_SUASSUNA_PID_H
#define ARMORIAL_SUASSUNA_PID_H

#define MAX_VEL 30.f

class PID {
public:
  PID();
  void setSetPoint(float setPoint);
  void setActualValue(float actualValue);
  void setConstants(float kp, float ki, float kd);
  void setOutputLimits(float min, float max);
  float getOutput();
  float getError();
  float getI();

private:
  float _lastError = 0.0f;
  float _lastI = 0.0f;

  // Constants
  float _Kp, _Ki, _Kd;
  float _minOutput, _maxOutput;
  float _setPoint, _actualValue;
};

#endif // ARMORIAL_SUASSUNA_PID_H
