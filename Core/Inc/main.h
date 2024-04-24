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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN_ID_3_Pin GPIO_PIN_13
#define CAN_ID_3_GPIO_Port GPIOC
#define CAN_ID_1_Pin GPIO_PIN_14
#define CAN_ID_1_GPIO_Port GPIOC
#define CAN_ID_2_Pin GPIO_PIN_15
#define CAN_ID_2_GPIO_Port GPIOC
#define AUTO_TEST_Pin GPIO_PIN_0
#define AUTO_TEST_GPIO_Port GPIOA
#define DISTANCE_SENSOR1_Pin GPIO_PIN_1
#define DISTANCE_SENSOR1_GPIO_Port GPIOA
#define DISTANCE_SENSOR2_Pin GPIO_PIN_2
#define DISTANCE_SENSOR2_GPIO_Port GPIOA
#define DC_MOTOR2_CURRENT_Pin GPIO_PIN_3
#define DC_MOTOR2_CURRENT_GPIO_Port GPIOA
#define DC_MOTOR1_CURRENT_Pin GPIO_PIN_4
#define DC_MOTOR1_CURRENT_GPIO_Port GPIOA
#define DC_MOTOR1_PWM2_Pin GPIO_PIN_7
#define DC_MOTOR1_PWM2_GPIO_Port GPIOA
#define DC_MOTOR1_PWM1_Pin GPIO_PIN_0
#define DC_MOTOR1_PWM1_GPIO_Port GPIOB
#define DC_MOTOR1_FAULT_Pin GPIO_PIN_1
#define DC_MOTOR1_FAULT_GPIO_Port GPIOB
#define SYS_LED_Pin GPIO_PIN_12
#define SYS_LED_GPIO_Port GPIOB
#define DC_MOTOR2_PWM2_Pin GPIO_PIN_13
#define DC_MOTOR2_PWM2_GPIO_Port GPIOB
#define DC_MOTOR2_PWM1_Pin GPIO_PIN_14
#define DC_MOTOR2_PWM1_GPIO_Port GPIOB
#define DC_MOTOR2_FAULT_Pin GPIO_PIN_8
#define DC_MOTOR2_FAULT_GPIO_Port GPIOA
#define CAN_MODE2_Pin GPIO_PIN_9
#define CAN_MODE2_GPIO_Port GPIOA
#define CAN_MODE1_Pin GPIO_PIN_10
#define CAN_MODE1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
