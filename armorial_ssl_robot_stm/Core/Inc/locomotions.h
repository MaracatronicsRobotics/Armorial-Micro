#ifndef ARMORIAL_SUASSUNA_LOCOMOTIONS_H
#define ARMORIAL_SUASSUNA_LOCOMOTIONS_H

#include <math.h>

#define MOTOR_1_PWM TIM3->CCR3
#define MOTOR_2_PWM TIM3->CCR1
#define MOTOR_3_PWM TIM3->CCR2
#define MOTOR_4_PWM TIM3->CCR4

static float degToRad = M_PI / 180;

static float wheel1Angle = 60;
static float wheel2Angle = 135;
static float wheel3Angle = 225;
static float wheel4Angle = 300;

static float wheelRadius = 0.030;
static float robotRadius = 0.089;

float wheelFrontLeft = 0;
float wheelBottomLeft = 0;
float wheelBottomRight = 0;
float wheelFrontRight = 0;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void calcWheelsPWM(float vx, float vy, float vw) {
	wheelFrontLeft = (-27.9557 * vx) + (18.1546 * vy) + (2.697 * vw);
	wheelBottomLeft = (-23.5702 * vx) + (-23.5702 * vy) + (2.697 * vw);
	wheelBottomRight = (23.5702 * vx) + (-23.5702 * vy) + (2.697 * vw);
	wheelFrontRight = -((-27.9557 * vx) + (18.1546 * vy) + (2.697 * vw));

	wheelFrontLeft = (1.0 / wheelRadius) *
			(( (robotRadius * vw) - (vx * sin(wheel1Angle * degToRad)) + (vy * cos(wheel1Angle * degToRad)) ));
	wheelBottomLeft = (1.0 / wheelRadius) *
			(( (robotRadius * vw) - (vx * sin(wheel2Angle * degToRad)) + (vy * cos(wheel2Angle * degToRad)) ));
	wheelBottomRight = (1.0 / wheelRadius) *
			(( (robotRadius * vw) - (vx * sin(wheel3Angle * degToRad)) + (vy * cos(wheel3Angle * degToRad)) ));
	wheelFrontRight = (1.0 / wheelRadius) *
			(( (robotRadius * vw) - (vx * sin(wheel4Angle * degToRad)) + (vy * cos(wheel4Angle * degToRad)) ));

	if(fabs(wheelFrontLeft) >= 30.0f) wheelFrontLeft = (wheelFrontLeft < 0) ? -30.0f : 30.0f;
	if(fabs(wheelBottomLeft) >= 30.0f) wheelBottomLeft = (wheelBottomLeft < 0) ? -30.0f : 30.0f;
	if(fabs(wheelBottomRight) >= 30.0f) wheelBottomRight = (wheelBottomRight < 0) ? -30.0f : 30.0f;
	if(fabs(wheelFrontRight) >= 30.0f) wheelFrontRight = (wheelFrontRight < 0) ? -30.0f : 30.0f;

	MOTOR_1_PWM = map(fabs(wheelFrontLeft), 0, 30, 10, 100);
	HAL_GPIO_WritePin(FWD_REV_M1_GPIO_Port, FWD_REV_M1_Pin, (wheelFrontLeft >= 0));
	HAL_GPIO_WritePin(EN_M1_GPIO_Port, EN_M1_Pin, fabs(wheelFrontLeft) > 2);

	MOTOR_2_PWM = map(fabs(wheelBottomLeft), 0, 30, 10, 100);
	HAL_GPIO_WritePin(FWD_REV_M2_GPIO_Port, FWD_REV_M2_Pin, (wheelBottomLeft >= 0));
	HAL_GPIO_WritePin(EN_M2_GPIO_Port, EN_M2_Pin, fabs(wheelBottomLeft) > 2);

	MOTOR_3_PWM = map(fabs(wheelBottomRight), 0, 30, 10, 100);
	HAL_GPIO_WritePin(FWD_REV_M4_GPIO_Port, FWD_REV_M4_Pin, (wheelBottomRight >= 0));
	HAL_GPIO_WritePin(EN_M4_GPIO_Port, EN_M4_Pin, fabs(wheelBottomRight) > 2);

	MOTOR_4_PWM = map(fabs(wheelFrontRight), 0, 30, 10, 100);
	HAL_GPIO_WritePin(FWD_REV_M3_GPIO_Port, FWD_REV_M3_Pin, (wheelFrontRight >= 0));
	HAL_GPIO_WritePin(EN_M3_GPIO_Port, EN_M3_Pin, fabs(wheelFrontRight) > 2);
}

static void getWheels(float *wheels) {
	wheels[0] = MOTOR_1_PWM;
	wheels[1] = MOTOR_2_PWM;
	wheels[2] = MOTOR_3_PWM;
	wheels[3] = MOTOR_4_PWM;
}

#endif // ARMORIAL_SUASSUNA_LOCOMOTIONS_H
