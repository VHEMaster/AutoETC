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
#include "stm32f0xx_hal.h"

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
#define INPUT_CRUIZE_START_Pin GPIO_PIN_13
#define INPUT_CRUIZE_START_GPIO_Port GPIOC
#define OUTPUT_CRUIZE_G_Pin GPIO_PIN_0
#define OUTPUT_CRUIZE_G_GPIO_Port GPIOA
#define OUTPUT_CRUIZE_R_Pin GPIO_PIN_1
#define OUTPUT_CRUIZE_R_GPIO_Port GPIOA
#define OUTPUT_RSVD3_Pin GPIO_PIN_2
#define OUTPUT_RSVD3_GPIO_Port GPIOA
#define OUTPUT_RSVD4_Pin GPIO_PIN_3
#define OUTPUT_RSVD4_GPIO_Port GPIOA
#define MCU_OUTPUT_EN_Pin GPIO_PIN_4
#define MCU_OUTPUT_EN_GPIO_Port GPIOA
#define MOTOR_DIR_Pin GPIO_PIN_5
#define MOTOR_DIR_GPIO_Port GPIOA
#define MOTOR_DIS_Pin GPIO_PIN_6
#define MOTOR_DIS_GPIO_Port GPIOA
#define MOTOR_PWM_TIM3_CH2_Pin GPIO_PIN_7
#define MOTOR_PWM_TIM3_CH2_GPIO_Port GPIOA
#define INPUT_GND_Pin GPIO_PIN_2
#define INPUT_GND_GPIO_Port GPIOB
#define SPI2_NRST_Pin GPIO_PIN_10
#define SPI2_NRST_GPIO_Port GPIOB
#define SPI2_NSS_MOTOR_Pin GPIO_PIN_11
#define SPI2_NSS_MOTOR_GPIO_Port GPIOB
#define SPI2_NSS_OUT_Pin GPIO_PIN_12
#define SPI2_NSS_OUT_GPIO_Port GPIOB
#define INPUT_RSVD6_Pin GPIO_PIN_8
#define INPUT_RSVD6_GPIO_Port GPIOA
#define INPUT_RSVD5_Pin GPIO_PIN_9
#define INPUT_RSVD5_GPIO_Port GPIOA
#define INPUT_RSVD4_Pin GPIO_PIN_10
#define INPUT_RSVD4_GPIO_Port GPIOA
#define CAN_LBK_Pin GPIO_PIN_15
#define CAN_LBK_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_6
#define SPI1_NSS_GPIO_Port GPIOB
#define SPI1_NRST_Pin GPIO_PIN_7
#define SPI1_NRST_GPIO_Port GPIOB
#define INPUT_BRAKE_Pin GPIO_PIN_8
#define INPUT_BRAKE_GPIO_Port GPIOB
#define INPUT_CRUIZE_STOP_Pin GPIO_PIN_9
#define INPUT_CRUIZE_STOP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
