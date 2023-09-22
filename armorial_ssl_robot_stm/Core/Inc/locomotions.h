#ifndef ARMORIAL_SUASSUNA_LOCOMOTIONS_H
#define ARMORIAL_SUASSUNA_LOCOMOTIONS_H

#include <math.h>

#define MOTOR_1_PWM TIM3->CCR3
#define MOTOR_2_PWM TIM3->CCR1
#define MOTOR_3_PWM TIM3->CCR2
#define MOTOR_4_PWM TIM3->CCR4

#define DEG_TO_RAD M_PI / 180

static float wheel1Angle = 60;
static float wheel2Angle = 135;
static float wheel3Angle = 225;
static float wheel4Angle = 300;

//TODO: fix wheel and robot radius
static float wheelRadius = 10;
static float robotRadius = 50;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void calcWheelsPWM(float vx, float vy, float vw) {
	float wheelFrontLeft = (-27.9557 * vx) + (18.1546 * vy) + (2.697 * vw);
	float wheelBottomLeft = (-23.5702 * vx) + (-23.5702 * vy) + (2.697 * vw);
	float wheelBottomRight = (23.5702 * vx) + (-23.5702 * vy) + (2.697 * vw);
	float wheelFrontRight = (-27.9557 * vx) + (18.1546 * vy) + (2.697 * vw);


	wheelFrontLeft = (1.0f / wheelRadius) * (( (robotRadius * vw) - (vx * sin(wheel1Angle * DEG_TO_RAD)) + (vy * cos(wheel1Angle * DEG_TO_RAD)) ));
	wheelBottomLeft = (1.0f / wheelRadius) * (( (robotRadius * vw) - (vx * sin(wheel2Angle * DEG_TO_RAD)) + (vy * cos(wheel2Angle * DEG_TO_RAD)) ));
	wheelBottomRight = (1.0f / wheelRadius) * (( (robotRadius * vw) - (vx * sin(wheel3Angle * DEG_TO_RAD)) + (vy * cos(wheel3Angle * DEG_TO_RAD)) ));
	wheelFrontRight = (1.0f / wheelRadius) * (( (robotRadius * vw) - (vx * sin(wheel4Angle * DEG_TO_RAD)) + (vy * cos(wheel4Angle * DEG_TO_RAD)) ));

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

#endif // ARMORIAL_SUASSUNA_LOCOMOTIONS_H
