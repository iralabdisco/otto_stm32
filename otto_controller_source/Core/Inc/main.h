/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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
#define user_button_Pin GPIO_PIN_13
#define user_button_GPIO_Port GPIOC
#define user_button_EXTI_IRQn EXTI15_10_IRQn
#define current2_Pin GPIO_PIN_0
#define current2_GPIO_Port GPIOC
#define encoder_sx1_Pin GPIO_PIN_0
#define encoder_sx1_GPIO_Port GPIOA
#define encoder_sx2_Pin GPIO_PIN_1
#define encoder_sx2_GPIO_Port GPIOA
#define current1_Pin GPIO_PIN_3
#define current1_GPIO_Port GPIOA
#define encoder_dx1_Pin GPIO_PIN_5
#define encoder_dx1_GPIO_Port GPIOA
#define fault2_Pin GPIO_PIN_6
#define fault2_GPIO_Port GPIOA
#define dir2_Pin GPIO_PIN_12
#define dir2_GPIO_Port GPIOF
#define dir1_Pin GPIO_PIN_13
#define dir1_GPIO_Port GPIOF
#define sleep2_Pin GPIO_PIN_14
#define sleep2_GPIO_Port GPIOF
#define sleep1_Pin GPIO_PIN_15
#define sleep1_GPIO_Port GPIOF
#define fault1_Pin GPIO_PIN_9
#define fault1_GPIO_Port GPIOE
#define pwm2_Pin GPIO_PIN_14
#define pwm2_GPIO_Port GPIOD
#define pwm1_Pin GPIO_PIN_15
#define pwm1_GPIO_Port GPIOD
#define encoder_dx2_Pin GPIO_PIN_3
#define encoder_dx2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
