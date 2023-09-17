#ifndef ARMORIAL_SUASSUNA_DRIBLE_H
#define ARMORIAL_SUASSUNA_DRIBLE_H

#define PWM_DRIBLE TIM4->CCR1
#define CALIBRATION_DELAY 1000
#define RAMP_DELAY 100
#define BASE_PWM 5000
#define STEP 50
#define MAX_PWM 6000

static long long time_diff;
static long long ramp_begin;
static int isDribblingRampActivated;

static void calibrateESC() {
	/*
	ramp_begin = HAL_GetTick();
	int calibrationValues[3] = {20000, 10000, 5000};
	int delayValues[3] = {2, 1, 1};
	int index = 0;
	while(1) {
		PWM_DRIBLE = calibrationValues[index];
		if((HAL_GetTick() - ramp_begin) >= (delayValues[index] * 1000)) {
			index = index + 1;
			if(index == 3) break;
		}
		//PWM_DRIBLE = 20000;
		//HAL_Delay(CALIBRATION_DELAY * 2);
		//PWM_DRIBLE = 10000;
		//HAL_Delay(CALIBRATION_DELAY);
		//PWM_DRIBLE = 5000;
		//HAL_Delay(CALIBRATION_DELAY);
	}
	*/
	PWM_DRIBLE = 20000;
	HAL_Delay(CALIBRATION_DELAY * 2);
	PWM_DRIBLE = 10000;
	HAL_Delay(CALIBRATION_DELAY);
	PWM_DRIBLE = 5000;
	HAL_Delay(CALIBRATION_DELAY);
}

static void activateESC() {
	long long timestamp = HAL_GetTick();
	if (isDribblingRampActivated) {
		time_diff = timestamp - ramp_begin;
		if ((PWM_DRIBLE != MAX_PWM) && (time_diff >= RAMP_DELAY)) {
			PWM_DRIBLE = PWM_DRIBLE + STEP;
			ramp_begin = timestamp;
		}
	} else {
		isDribblingRampActivated = 1;
		ramp_begin = timestamp;
	}
}

static void deactivateESC() {
	PWM_DRIBLE = BASE_PWM + STEP;
	isDribblingRampActivated = 0;
}

#endif // ARMORIAL_SUASSUNA_DRIBLE_H
