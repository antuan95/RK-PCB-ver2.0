/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define Servo_Ctrl1_Pin GPIO_PIN_0
#define Servo_Ctrl1_GPIO_Port GPIOA
#define Reserve_Pin GPIO_PIN_1
#define Reserve_GPIO_Port GPIOA
#define Servo_Ctrl2_Pin GPIO_PIN_2
#define Servo_Ctrl2_GPIO_Port GPIOA
#define Button_Pin GPIO_PIN_4
#define Button_GPIO_Port GPIOA
#define Motor_sw_Pin GPIO_PIN_5
#define Motor_sw_GPIO_Port GPIOA
#define Hall_Sensor_Pin GPIO_PIN_6
#define Hall_Sensor_GPIO_Port GPIOA
#define ARM_SCL_Pin GPIO_PIN_7
#define ARM_SCL_GPIO_Port GPIOA
#define Servo_but1_Pin GPIO_PIN_0
#define Servo_but1_GPIO_Port GPIOB
#define Servo_but2_Pin GPIO_PIN_1
#define Servo_but2_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_8
#define CSN_GPIO_Port GPIOA
#define Motor_SCL_Pin GPIO_PIN_9
#define Motor_SCL_GPIO_Port GPIOA
#define Motor_SDA_Pin GPIO_PIN_10
#define Motor_SDA_GPIO_Port GPIOA
#define EncA_Pin GPIO_PIN_15
#define EncA_GPIO_Port GPIOA
#define EncA_EXTI_IRQn EXTI15_10_IRQn
#define EncB_Pin GPIO_PIN_3
#define EncB_GPIO_Port GPIOB
#define EncB_EXTI_IRQn EXTI3_IRQn
#define ARM_SDA_Pin GPIO_PIN_4
#define ARM_SDA_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOB
#define RFID_TX_Pin GPIO_PIN_6
#define RFID_TX_GPIO_Port GPIOB
#define RFID_RX_Pin GPIO_PIN_7
#define RFID_RX_GPIO_Port GPIOB
#define Arm_sw_Pin GPIO_PIN_3
#define Arm_sw_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
