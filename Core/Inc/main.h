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
#include "stm32h7xx_hal.h"

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
#define SPI4_CS_Pin GPIO_PIN_3
#define SPI4_CS_GPIO_Port GPIOE
#define SPI4_INT_Pin GPIO_PIN_4
#define SPI4_INT_GPIO_Port GPIOE
#define SPI4_INT_EXTI_IRQn EXTI4_IRQn
#define SPI4_RESET_Pin GPIO_PIN_13
#define SPI4_RESET_GPIO_Port GPIOC
#define UART2_RESET_Pin GPIO_PIN_1
#define UART2_RESET_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI1_RESET_Pin GPIO_PIN_5
#define SPI1_RESET_GPIO_Port GPIOC
#define SPI1_WAKE_Pin GPIO_PIN_0
#define SPI1_WAKE_GPIO_Port GPIOB
#define SPI1_INT_Pin GPIO_PIN_1
#define SPI1_INT_GPIO_Port GPIOB
#define SPI1_INT_EXTI_IRQn EXTI1_IRQn
#define SPI2_CS_Pin GPIO_PIN_11
#define SPI2_CS_GPIO_Port GPIOB
#define SPI2_INT_Pin GPIO_PIN_13
#define SPI2_INT_GPIO_Port GPIOB
#define SPI2_INT_EXTI_IRQn EXTI15_10_IRQn
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOD
#define SPI3_CS_Pin GPIO_PIN_0
#define SPI3_CS_GPIO_Port GPIOD
#define SW_U_Pin GPIO_PIN_2
#define SW_U_GPIO_Port GPIOD
#define SW_D_Pin GPIO_PIN_3
#define SW_D_GPIO_Port GPIOD
#define SW_L_Pin GPIO_PIN_4
#define SW_L_GPIO_Port GPIOD
#define SW_R_Pin GPIO_PIN_5
#define SW_R_GPIO_Port GPIOD
#define SW_P_Pin GPIO_PIN_6
#define SW_P_GPIO_Port GPIOD
#define I2C1_INT_Pin GPIO_PIN_5
#define I2C1_INT_GPIO_Port GPIOB
#define I2C1_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
