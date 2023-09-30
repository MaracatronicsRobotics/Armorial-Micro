#ifndef ARMORIAL_SUASSUNA_LOCOMOTIONS_H
#define ARMORIAL_SUASSUNA_LOCOMOTIONS_H

#include <math.h>

#define MOTOR_1_PWM TIM3->CCR3
#define MOTOR_2_PWM TIM3->CCR1
#define MOTOR_3_PWM TIM3->CCR2
#define MOTOR_4_PWM TIM3->CCR4

#define HALL_SENSOR_TIMER_DELAY 1000

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

long long H1_M1_read_begin;
long long H1_M2_read_begin;
long long H1_M3_read_begin;
long long H1_M4_read_begin;

int H1_M1_Counter = 0;
int H1_M2_Counter = 0;
int H1_M3_Counter = 0;
int H1_M4_Counter = 0;
float wheels_RPM[4];

uint8_t M1_State = 1; // 1 or -1 depending on wheel direction
uint8_t M2_State = 1;
uint8_t M3_State = 1;
uint8_t M4_State = 1;

int direction = 1;

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


void hallSensorReadings(uint16_t GPIO_Pin) {
	long long timestamp = HAL_GetTick();
	switch (GPIO_Pin) {
		case H1_M1_Pin: {
			if (HAL_GPIO_ReadPin(H2_M1_GPIO_Port, H2_M1_Pin)) M1_State = -1 * direction;
			else M1_State = 1;

			if (timestamp - H1_M1_read_begin >= HALL_SENSOR_TIMER_DELAY) {
				H1_M1_read_begin = timestamp;
				wheels_RPM[0] = H1_M1_Counter * 60; // RPS -> RPM
				H1_M1_Counter = 0;
			} else {
				H1_M1_Counter = H1_M1_Counter + 1;
			}
		} break;
		case H1_M2_Pin: {
			if (HAL_GPIO_ReadPin(H2_M2_GPIO_Port, H2_M2_Pin)) M2_State = -1 * direction;
			else M2_State = 1;

			if (timestamp - H1_M2_read_begin >= HALL_SENSOR_TIMER_DELAY) {
				H1_M2_read_begin = timestamp;
				wheels_RPM[1] = H1_M2_Counter * 60; // RPS -> RPM
				H1_M2_Counter = 0;
			} else {
				H1_M2_Counter = H1_M2_Counter + 1;
			}
		} break;
		case H1_M3_Pin: {
			if (HAL_GPIO_ReadPin(H2_M3_GPIO_Port, H2_M3_Pin)) M3_State = -1 * direction;
			else M3_State = 1;

			if (timestamp - H1_M3_read_begin >= HALL_SENSOR_TIMER_DELAY) {
				H1_M3_read_begin = timestamp;
				wheels_RPM[2] = H1_M3_Counter * 60; // RPS -> RPM
				H1_M3_Counter = 0;
			} else {
				H1_M3_Counter = H1_M3_Counter + 1;
			}
		} break;
		case H1_M4_Pin: {
			if (HAL_GPIO_ReadPin(H2_M4_GPIO_Port, H2_M4_Pin)) M4_State = -1 * direction;
			else M4_State = 1;

			if (timestamp - H1_M4_read_begin >= HALL_SENSOR_TIMER_DELAY) {
				H1_M4_read_begin = timestamp;
				wheels_RPM[3] = H1_M4_Counter * 60; // RPS -> RPM
				H1_M4_Counter = 0;
			} else {
				H1_M4_Counter = H1_M4_Counter + 1;
			}
		} break;
	}
}

static void getWheels(float *wheels) {
	wheels[0] = (wheels_RPM[0] * 2.0 * M_PI * 0.0005) / 60.0;
	wheels[1] = (wheels_RPM[1] * 2.0 * M_PI * 0.0005) / 60.0;
	wheels[2] = (wheels_RPM[2] * 2.0 * M_PI * 0.0005) / 60.0;
	wheels[3] = (wheels_RPM[3] * 2.0 * M_PI * 0.0005) / 60.0;
}

#endif // ARMORIAL_SUASSUNA_LOCOMOTIONS_H
