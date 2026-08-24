/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DI4_Pin GPIO_PIN_13
#define DI4_GPIO_Port GPIOC
#define DI3_Pin GPIO_PIN_14
#define DI3_GPIO_Port GPIOC
#define LSM_INT1_Pin GPIO_PIN_0
#define LSM_INT1_GPIO_Port GPIOC
#define LSM_INT1_EXTI_IRQn EXTI0_IRQn
#define LSM_INT2_Pin GPIO_PIN_1
#define LSM_INT2_GPIO_Port GPIOC
#define LSM_CS_Pin GPIO_PIN_0
#define LSM_CS_GPIO_Port GPIOA
#define MPU_CS_Pin GPIO_PIN_4
#define MPU_CS_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_4
#define MPU_INT_GPIO_Port GPIOC
#define MPU_INT_EXTI_IRQn EXTI4_IRQn
#define DO1_Pin GPIO_PIN_12
#define DO1_GPIO_Port GPIOB
#define DO2_Pin GPIO_PIN_13
#define DO2_GPIO_Port GPIOB
#define DI1_Pin GPIO_PIN_14
#define DI1_GPIO_Port GPIOB
#define DI2_Pin GPIO_PIN_15
#define DI2_GPIO_Port GPIOB
#define BMI270_CS_Pin GPIO_PIN_15
#define BMI270_CS_GPIO_Port GPIOA
#define BMI270_INT1_Pin GPIO_PIN_2
#define BMI270_INT1_GPIO_Port GPIOD
#define BMI270_INT1_EXTI_IRQn EXTI2_IRQn
#define BMI270_INT2_Pin GPIO_PIN_3
#define BMI270_INT2_GPIO_Port GPIOB
#define DO4_Pin GPIO_PIN_8
#define DO4_GPIO_Port GPIOB
#define DO3_Pin GPIO_PIN_9
#define DO3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
