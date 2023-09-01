/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define H2_M2_Pin GPIO_PIN_2
#define H2_M2_GPIO_Port GPIOE
#define H1_M2_Pin GPIO_PIN_4
#define H1_M2_GPIO_Port GPIOE
#define H1_M2_EXTI_IRQn EXTI4_IRQn
#define DISPARO_CHIP_KICK_Pin GPIO_PIN_5
#define DISPARO_CHIP_KICK_GPIO_Port GPIOE
#define FWD_REV_M2_Pin GPIO_PIN_6
#define FWD_REV_M2_GPIO_Port GPIOE
#define DISPARO_NORMAL_Pin GPIO_PIN_13
#define DISPARO_NORMAL_GPIO_Port GPIOC
#define EN_M2_Pin GPIO_PIN_0
#define EN_M2_GPIO_Port GPIOC
#define FWD_REV_M1_Pin GPIO_PIN_2
#define FWD_REV_M1_GPIO_Port GPIOC
#define LEITURA_CHUTE_Pin GPIO_PIN_3
#define LEITURA_CHUTE_GPIO_Port GPIOC
#define PWM_CARREGAMENTO_CHUTE_Pin GPIO_PIN_1
#define PWM_CARREGAMENTO_CHUTE_GPIO_Port GPIOA
#define EN_M1_Pin GPIO_PIN_4
#define EN_M1_GPIO_Port GPIOA
#define LEITURA_INFRA_Pin GPIO_PIN_5
#define LEITURA_INFRA_GPIO_Port GPIOA
#define PWM_M2_Pin GPIO_PIN_6
#define PWM_M2_GPIO_Port GPIOA
#define LEITURA_3V3_Pin GPIO_PIN_7
#define LEITURA_3V3_GPIO_Port GPIOA
#define LEITURA_5V_Pin GPIO_PIN_4
#define LEITURA_5V_GPIO_Port GPIOC
#define LEITURA_8V_Pin GPIO_PIN_5
#define LEITURA_8V_GPIO_Port GPIOC
#define PWM_M1_Pin GPIO_PIN_0
#define PWM_M1_GPIO_Port GPIOB
#define LEITURA_BATERIA_Pin GPIO_PIN_1
#define LEITURA_BATERIA_GPIO_Port GPIOB
#define H2_M1_Pin GPIO_PIN_13
#define H2_M1_GPIO_Port GPIOE
#define H1_M1_Pin GPIO_PIN_15
#define H1_M1_GPIO_Port GPIOE
#define H1_M1_EXTI_IRQn EXTI15_10_IRQn
#define H2_M3_Pin GPIO_PIN_15
#define H2_M3_GPIO_Port GPIOB
#define H1_M3_Pin GPIO_PIN_9
#define H1_M3_GPIO_Port GPIOD
#define H1_M3_EXTI_IRQn EXTI9_5_IRQn
#define PWM_DRIBLE_Pin GPIO_PIN_12
#define PWM_DRIBLE_GPIO_Port GPIOD
#define PWM_M3_Pin GPIO_PIN_7
#define PWM_M3_GPIO_Port GPIOC
#define PWM_M4_Pin GPIO_PIN_9
#define PWM_M4_GPIO_Port GPIOC
#define FWD_REV_M3_Pin GPIO_PIN_11
#define FWD_REV_M3_GPIO_Port GPIOA
#define EN_M3_Pin GPIO_PIN_4
#define EN_M3_GPIO_Port GPIOD
#define FWD_REV_M4_Pin GPIO_PIN_6
#define FWD_REV_M4_GPIO_Port GPIOD
#define EN_M4_Pin GPIO_PIN_3
#define EN_M4_GPIO_Port GPIOB
#define H1_M4_Pin GPIO_PIN_8
#define H1_M4_GPIO_Port GPIOB
#define H1_M4_EXTI_IRQn EXTI9_5_IRQn
#define H2_M4_Pin GPIO_PIN_0
#define H2_M4_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
