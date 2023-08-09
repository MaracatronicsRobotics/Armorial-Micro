#include "pid.h"

PID::PID() {
  _setPoint = 0.0f;
  _actualValue = 0.0f;
  _minOutput = -300.0f;
  _maxOutput = 300.0f;
  _Kp = 1.0f;
  _Ki = 0.0f;
  _Kd = 0.0f;
}

void PID::setSetPoint(float setPoint) {
  _setPoint = setPoint;
}

void PID::setActualValue(float actualValue) {
  _actualValue = actualValue;
}

void PID::setConstants(float kp, float ki, float kd) {
  _Kp = kp;
  _Ki = ki;
  _Kd = kd;
}

void PID::setOutputLimits(float min, float max) {
  _minOutput = min;
  _maxOutput = max;
}

float PID::getOutput() {
  float error = _setPoint - _actualValue;

  // Considerando um degrau, o erro deve ser cada vez menor durante as interações, logo
  // se o erro anterior for menor, ocorreu uma nova perturbação e o PID deve ser resetado
  if (error > _lastError) {
    _errorSum = 0.0f;
    _lastError = 0.0f;
  }
  _errorSum += error;
 
  float P = error * _Kp;
  float I = _errorSum * _Ki;
  float D = (error - _lastError) * _Kd;
  
  _lastError = error;

	float output = P + I + D;

  if (output > _maxOutput) {
    output = _maxOutput;
  } else if (output < _minOutput) {
    output = _minOutput;
  }
	return output;
}