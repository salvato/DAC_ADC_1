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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define hadc1 hadc1
#define hadc2 hadc2
#define htim2 htim2
#define htim3 htim3

#define DAC_CHAN1
#ifdef DAC_CHAN1
    #define DAC1_CHANNEL DAC1_CHANNEL_1
    #define LD2_Pin GPIO_PIN_5  // In conflitto con DAC Out2
    #define LD2_GPIO_Port GPIOA // In conflitto con DAC Out2
#else
    #define DAC1_CHANNEL DAC1_CHANNEL_2
#endif

#define RampMinLed_Pin   GPIO_PIN_13
#define RampMaxLed_Pin   GPIO_PIN_14
#define RampStartLed_Pin GPIO_PIN_15

#define RampMinPB_Pin   GPIO_PIN_4
#define RampMaxPB_Pin   GPIO_PIN_5
#define RampStartPB_Pin GPIO_PIN_6

#define RampMinPB_GPIO_Port   GPIOB
#define RampMaxPB_GPIO_Port   GPIOB
#define RampStartPB_GPIO_Port GPIOB

#define RampMinPB_EXTI_IRQn   EXTI4_IRQn
#define RampMaxPB_EXTI_IRQn   EXTI9_5_IRQn
#define RampStartPB_EXTI_IRQn EXTI9_5_IRQn

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC

#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
