#ifndef ARMORIAL_SUASSUNA_PID_H
#define ARMORIAL_SUASSUNA_PID_H

#define ENCODER_RESOLUTION 10000 // microsseconds

#define MAX_VEL 30.f

class PID {
public:
  PID();
  void setSetPoint(float setPoint);
  void setActualValue(float actualValue);
  void setConstants(float kp, float ki, float kd);
  void setOutputLimits(float min, float max);
  float getOutput();

private:
  float _lastError = 0.0f;

  // Constants
  float _Kp, _Ki, _Kd;
  float _minOutput, _maxOutput;
  float _setPoint, _actualValue;
};

#endif // ARMORIAL_SUASSUNA_PID_H
