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

int M1_Counter = 0;
GPIO_PinState M1_State = GPIO_PIN_RESET;
int M2_Counter = 0;
GPIO_PinState M2_State = GPIO_PIN_RESET;
int M3_Counter = 0;
GPIO_PinState M3_State = GPIO_PIN_RESET;
int M4_Counter = 0;
GPIO_PinState M4_State = GPIO_PIN_RESET;
float wheels_RPM[4];

int H1_M1_Counter = 0;
int H2_M1_Counter = 0;
int H1_M2_Counter = 0;
int H2_M2_Counter = 0;
int H1_M3_Counter = 0;
int H2_M3_Counter = 0;
int H1_M4_Counter = 0;
int H2_M4_Counter = 0;

uint8_t M1_Dir = 1; // 1 or -1 depending on wheel direction
uint8_t M2_Dir = 1;
uint8_t M3_Dir = 1;
uint8_t M4_Dir = 1;

enum HallSensors_State M1_HS_State;
enum HallSensors_State M1_HS_State_Prev;
enum HallSensors_State M2_HS_State;
enum HallSensors_State M2_HS_State_Prev;
enum HallSensors_State M3_HS_State;
enum HallSensors_State M3_HS_State_Prev;
enum HallSensors_State M4_HS_State;
enum HallSensors_State M4_HS_State_Prev;

int direction = 1;

enum HallSensors_State {
	DOWN_DOWN,
	DOWN_UP,
	UP_UP,
	UP_DOWN
};

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void calcWheelsPWM(float vx, float vy, float vw) {
//	wheelFrontLeft = (-27.9557 * vx) + (18.1546 * vy) + (2.697 * vw);
//	wheelBottomLeft = (-23.5702 * vx) + (-23.5702 * vy) + (2.697 * vw);
//	wheelBottomRight = (23.5702 * vx) + (-23.5702 * vy) + (2.697 * vw);
//	wheelFrontRight = -((-27.9557 * vx) + (18.1546 * vy) + (2.697 * vw));

	wheelFrontLeft = (1.0 / wheelRadius) *
			(( (robotRadius * vw) - (vx * sin(wheel1Angle * degToRad)) + (vy * cos(wheel1Angle * degToRad)) ));
	wheelBottomLeft = (1.0 / wheelRadius) *
			(( (robotRadius * vw) - (vx * sin(wheel2Angle * degToRad)) + (vy * cos(wheel2Angle * degToRad)) ));
	wheelBottomRight = (1.0 / wheelRadius) *
			(( (robotRadius * vw) - (vx * sin(wheel3Angle * degToRad)) + (vy * cos(wheel3Angle * degToRad)) ));
	wheelFrontRight = (1.0 / wheelRadius) *
			(( (robotRadius * vw) - (vx * sin(wheel4Angle * degToRad)) + (vy * cos(wheel4Angle * degToRad)) ));

	/*
	wheelFrontLeft = (-27.9557 * vx) + (18.1546 * vy) + (2.697 * vw);
	wheelBottomLeft = (23.5702 * vx) + (-23.5702 * vy) + (2.697 * vw);
	wheelBottomRight = (23.5702 * vx) + (-23.5702 * vy) + (2.697 * vw);
	wheelFrontRight = ((-27.9557 * vx) + (18.1546 * vy) + (2.697 * vw));
	*/

	if(fabs(wheelFrontLeft) >= 30.0f) wheelFrontLeft = (wheelFrontLeft < 0) ? -30.0f : 30.0f;
	if(fabs(wheelBottomLeft) >= 30.0f) wheelBottomLeft = (wheelBottomLeft < 0) ? -30.0f : 30.0f;
	if(fabs(wheelBottomRight) >= 30.0f) wheelBottomRight = (wheelBottomRight < 0) ? -30.0f : 30.0f;
	if(fabs(wheelFrontRight) >= 30.0f) wheelFrontRight = (wheelFrontRight < 0) ? -30.0f : 30.0f;

	MOTOR_1_PWM = map(fabs(wheelFrontLeft), 0, 30, 30, 100);
	HAL_GPIO_WritePin(FWD_REV_M1_GPIO_Port, FWD_REV_M1_Pin, (wheelFrontLeft >= 0));
	HAL_GPIO_WritePin(EN_M1_GPIO_Port, EN_M1_Pin, fabs(wheelFrontLeft) > 1);

	MOTOR_2_PWM = map(fabs(wheelBottomLeft), 0, 30, 30, 100);
	HAL_GPIO_WritePin(FWD_REV_M2_GPIO_Port, FWD_REV_M2_Pin, (wheelBottomLeft >= 0));
	HAL_GPIO_WritePin(EN_M2_GPIO_Port, EN_M2_Pin, fabs(wheelBottomLeft) > 1);

	MOTOR_3_PWM = map(fabs(wheelBottomRight), 0, 30, 30, 100);
	HAL_GPIO_WritePin(FWD_REV_M4_GPIO_Port, FWD_REV_M4_Pin, (wheelBottomRight >= 0));
	HAL_GPIO_WritePin(EN_M4_GPIO_Port, EN_M4_Pin, fabs(wheelBottomRight) > 1);

	MOTOR_4_PWM = map(fabs(wheelFrontRight), 0, 30, 30, 100);
	HAL_GPIO_WritePin(FWD_REV_M3_GPIO_Port, FWD_REV_M3_Pin, (wheelFrontRight >= 0));
	HAL_GPIO_WritePin(EN_M3_GPIO_Port, EN_M3_Pin, fabs(wheelFrontRight) > 1);

//	wheelFrontLeft = vx * 100;
//	wheelBottomLeft = vx * 100;
//	wheelBottomRight = vx * 100;
//	wheelFrontRight = vx * 100;
//
//	MOTOR_1_PWM = fabs(wheelFrontLeft);
//	HAL_GPIO_WritePin(FWD_REV_M1_GPIO_Port, FWD_REV_M1_Pin, (wheelFrontLeft >= 0));
//	HAL_GPIO_WritePin(EN_M1_GPIO_Port, EN_M1_Pin, fabs(wheelFrontLeft) > 2);
//
//	MOTOR_2_PWM = fabs(wheelBottomLeft);
//	HAL_GPIO_WritePin(FWD_REV_M2_GPIO_Port, FWD_REV_M2_Pin, (wheelBottomLeft >= 0));
//	HAL_GPIO_WritePin(EN_M2_GPIO_Port, EN_M2_Pin, fabs(wheelBottomLeft) > 2);
//
//	MOTOR_3_PWM = fabs(wheelBottomRight);
//	HAL_GPIO_WritePin(FWD_REV_M4_GPIO_Port, FWD_REV_M4_Pin, (wheelBottomRight >= 0));
//	HAL_GPIO_WritePin(EN_M4_GPIO_Port, EN_M4_Pin, fabs(wheelBottomRight) > 2);
//
//	MOTOR_4_PWM = fabs(wheelFrontRight);
//	HAL_GPIO_WritePin(FWD_REV_M3_GPIO_Port, FWD_REV_M3_Pin, (wheelFrontRight >= 0));
//	HAL_GPIO_WritePin(EN_M3_GPIO_Port, EN_M3_Pin, fabs(wheelFrontRight) > 2);
}


void hallSensorReadings(uint16_t GPIO_Pin) {
	long long timestamp = HAL_GetTick();
	switch (GPIO_Pin) {
		case H1_M1_Pin: {
			if (HAL_GPIO_ReadPin(H2_M1_GPIO_Port, H2_M1_Pin)) M1_Dir = -1 * direction;
			else M1_Dir = direction;

			if (timestamp - H1_M1_read_begin >= HALL_SENSOR_TIMER_DELAY) {
				H1_M1_read_begin = timestamp;
				wheels_RPM[0] = H1_M1_Counter * 60; // RPS -> RPM
				H1_M1_Counter = 0;
			} else {
				H1_M1_Counter += 1;
			}
		} break;
		case H1_M2_Pin: {
			if (HAL_GPIO_ReadPin(H2_M2_GPIO_Port, H2_M2_Pin)) M2_Dir = -1 * direction;
			else M2_Dir = direction;

			if (timestamp - H1_M2_read_begin >= HALL_SENSOR_TIMER_DELAY) {
				H1_M2_read_begin = timestamp;
				wheels_RPM[1] = H1_M2_Counter * 60; // RPS -> RPM
				H1_M2_Counter = 0;
			} else {
				H1_M2_Counter += 1;
			}
		} break;
		case H1_M3_Pin: {
			if (HAL_GPIO_ReadPin(H2_M3_GPIO_Port, H2_M3_Pin)) M3_Dir = -1 * direction;
			else M3_Dir = direction;

			if (timestamp - H1_M3_read_begin >= HALL_SENSOR_TIMER_DELAY) {
				H1_M3_read_begin = timestamp;
				wheels_RPM[2] = H1_M3_Counter * 60; // RPS -> RPM
				H1_M3_Counter = 0;
			} else {
				H1_M3_Counter += 1;
			}
		} break;
		case H1_M4_Pin: {
			if (HAL_GPIO_ReadPin(H2_M4_GPIO_Port, H2_M4_Pin)) M4_Dir = -1 * direction;
			else M4_Dir = direction;

			if (timestamp - H1_M4_read_begin >= HALL_SENSOR_TIMER_DELAY) {
				H1_M4_read_begin = timestamp;
				wheels_RPM[3] = H1_M4_Counter * 60; // RPS -> RPM
				H1_M4_Counter = 0;
			} else {
				H1_M4_Counter += 1;
			}
		} break;
	}
}

enum HallSensors_State getHallState(GPIO_TypeDef* H1_Port, uint16_t H1_Pin, GPIO_TypeDef* H2_Port, uint16_t H2_Pin, int motor) {
	GPIO_PinState H1_State = HAL_GPIO_ReadPin(H1_Port, H1_Pin);
	GPIO_PinState H2_State = HAL_GPIO_ReadPin(H2_Port, H2_Pin);


	switch (motor) {
		case(1): {
			H1_M1_Counter += H1_State;
			H2_M1_Counter += H2_State;
		} break;
		case (2): {
			H1_M2_Counter += H1_State;
			H2_M2_Counter += H2_State;
		} break;
		case (3): {
			H1_M3_Counter += H1_State;
			H2_M3_Counter += H2_State;
		} break;
		case (4): {
			H1_M4_Counter += H1_State;
			H2_M4_Counter += H2_State;
		} break;
	}

	if (H1_State && H1_State) {
		return UP_UP;
	} else if (H1_State) {
		return UP_DOWN;
	} else if (H2_State) {
		return DOWN_UP;
	} else {
		return DOWN_DOWN;
	}
	/*
	 * H1 = 0, H2 = 0 -> 0
	 * H1 = 0, H2 = 1 -> 1
	 * H1 = 1, H2 = 1 -> 2
	 * H1 = 1, H2 = 0 -> 3
	 */
}

void monitorateHall() {
	GPIO_PinState H1_State = HAL_GPIO_ReadPin(H1_M1_GPIO_Port, H1_M1_Pin);
	GPIO_PinState H2_State = HAL_GPIO_ReadPin(H1_M2_GPIO_Port, H1_M2_Pin);
	GPIO_PinState H3_State = HAL_GPIO_ReadPin(H1_M4_GPIO_Port, H1_M4_Pin);
	GPIO_PinState H4_State = HAL_GPIO_ReadPin(H1_M3_GPIO_Port, H1_M3_Pin);

	long long timestamp = HAL_GetTick();

	if (M1_State < H1_State) {
		M1_Counter += 1;
	}
	M1_State = H1_State;

	if (M2_State < H2_State) {
		M2_Counter += 1;
	}
	M2_State = H2_State;

	if (M3_State < H3_State) {
		M3_Counter += 1;
	}
	M3_State = H3_State;

	if (M4_State < H4_State) {
		M4_Counter += 1;
	}
	M4_State = H4_State;

	if (timestamp - H1_M1_read_begin >= 20) {
		wheels_RPM[0] = ((2 * 3.14 * M1_Counter) / (6 * 3 * 0.02)) * (60 / (2 * 3.14));

		// (2 * M_PI * encoder_count_wl) /
	    //  (PULSES_PER_REVOLUTION * GEAR_RATIO * (ENCODER_RESOLUTION / 1E6f)));
		H1_M1_read_begin = timestamp;
		M1_Counter = 0;
	}

	if (timestamp - H1_M2_read_begin >= 20) {
		wheels_RPM[1] = ((2 * 3.14 * M2_Counter) / (6 * 3 * 0.02)) * (60 / (2 * 3.14));

		// (2 * M_PI * encoder_count_wl) /
	    //  (PULSES_PER_REVOLUTION * GEAR_RATIO * (ENCODER_RESOLUTION / 1E6f)));
		H1_M2_read_begin = timestamp;
		M2_Counter = 0;
	}

	if (timestamp - H1_M3_read_begin >= 20) {
		wheels_RPM[2] = ((2 * 3.14 * M3_Counter) / (6 * 3 * 0.02)) * (60 / (2 * 3.14));

		// (2 * M_PI * encoder_count_wl) /
	    //  (PULSES_PER_REVOLUTION * GEAR_RATIO * (ENCODER_RESOLUTION / 1E6f)));
		H1_M3_read_begin = timestamp;
		M3_Counter = 0;
	}

	if (timestamp - H1_M4_read_begin >= 20) {
		wheels_RPM[3] = ((2 * 3.14 * M4_Counter) / (6 * 3 * 0.02)) * (60 / (2 * 3.14));

		// (2 * M_PI * encoder_count_wl) /
	    //  (PULSES_PER_REVOLUTION * GEAR_RATIO * (ENCODER_RESOLUTION / 1E6f)));
		H1_M4_read_begin = timestamp;
		M4_Counter = 0;
	}

	// Motor 1
//		M1_State = getHallState(H1_M1_GPIO_Port, H1_M1_Pin, H2_M1_GPIO_Port, H2_M1_Pin, 1);
//		if (M1_State != M1_State_Prev) {
//			if (M1_State > M1_State_Prev || M1_State + 3 == M1_State_Prev) M1_Counter += direction;
//			else M1_Counter -= direction;
//			M1_State_Prev = M1_State;
//		}
//		if (timeStamp - H1_M1_read_begin >= HALL_SENSOR_TIMER_DELAY) {
//			H1_M1_read_begin = timeStamp;
//			wheels_RPM[0] = M1_Counter * 60;
//			M1_Counter = 0;
//		}
//
//	// Motor 2
//		M2_State = getHallState(H1_M2_GPIO_Port, H1_M2_Pin, H2_M2_GPIO_Port, H2_M2_Pin, 2);
//		if (M2_State != M2_State_Prev) {
//			if (M2_State > M2_State_Prev || M2_State + 3 == M2_State_Prev) M2_Counter += direction;
//			else M2_Counter -= direction;
//			M2_State_Prev = M2_State;
//		}
//		if (timeStamp - H1_M2_read_begin >= HALL_SENSOR_TIMER_DELAY) {
//			H1_M2_read_begin = timeStamp;
//			wheels_RPM[1] = M2_Counter * 60;
//			M2_Counter = 0;
//		}
//
//	// Motor 3
//		M3_State = getHallState(H1_M3_GPIO_Port, H1_M3_Pin, H2_M3_GPIO_Port, H2_M3_Pin, 3);
//		if (M3_State != M3_State_Prev) {
//			if (M3_State > M3_State_Prev || M3_State + 3 == M3_State_Prev) M3_Counter += direction;
//			else M3_Counter -= direction;
//			M3_State_Prev = M3_State;
//		}
//		if (timeStamp - H1_M3_read_begin >= HALL_SENSOR_TIMER_DELAY) {
//			H1_M3_read_begin = timeStamp;
//			wheels_RPM[2] = M3_Counter * 60;
//			M3_Counter = 0;
//		}
//
//	// Motor 4
//		M4_State = getHallState(H1_M4_GPIO_Port, H1_M4_Pin, H2_M4_GPIO_Port, H2_M4_Pin, 4);
//		if (M4_State != M4_State_Prev) {
//			if ((M4_State + 3) % 4 == M4_State_Prev) M4_Counter += direction;
//			else if ((M4_State - 3) % 4 == M4_State_Prev) M4_Counter -= direction;
//			M4_State_Prev = M4_State;
//		}
//		if (timeStamp - H1_M4_read_begin >= HALL_SENSOR_TIMER_DELAY) {
//			H1_M4_read_begin = timeStamp;
//			wheels_RPM[1] = H1_M4_Counter;
//			wheels_RPM[2] = H2_M4_Counter;
//			wheels_RPM[3] = M4_Counter;
//			H1_M4_Counter = 0;
//			H2_M4_Counter = 0;
//			M4_Counter = 0;
//		}
}

static void getWheels(float *wheels) {
	wheels[0] = wheels_RPM[0];
	wheels[1] = wheels_RPM[1];
	wheels[2] = wheels_RPM[2];
	wheels[3] = wheels_RPM[3];
}

#endif // ARMORIAL_SUASSUNA_LOCOMOTIONS_H
