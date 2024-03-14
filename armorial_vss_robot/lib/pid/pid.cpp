#include "pid.h"

PID::PID() {
  _setPoint = 0.0f;
  _actualValue = 0.0f;
  _minOutput = -300.0f;
  _maxOutput = 300.0f;
  _Kp = 0.0f;
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

float PID::getError() {
  return _lastError;
}

float PID::getI() {
  return _lastI;
}

float PID::getOutput() {
  float error = _setPoint - _actualValue;
 
  float P = _Kp * error;
  float I = _lastI + _Ki * error;
  float D = _Kd * (error - _lastError);
  
  _lastError = error;
  _lastI = I;

  // Serial.print("Error: ");
  // Serial.println(error);
  // Serial.print("I: ");
  // Serial.println(I);

	float output = P + I + D;

  if (output > _maxOutput) {
    output = _maxOutput;
  } else if (output < _minOutput) {
    output = _minOutput;
  }
	return output;
}