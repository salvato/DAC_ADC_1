/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_dac;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void
HAL_MspInit(void) {
    /* USER CODE BEGIN MspInit 0 */
    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* System interrupt init*/
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

    /* USER CODE BEGIN MspInit 1 */
    /* USER CODE END MspInit 1 */
}


/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void 
HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hadc->Instance==ADC1) {
        /* Peripheral clock enable */
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**ADC1 GPIO Configuration
        PA0-WKUP     ------> ADC1_IN0
        PA1          ------> ADC1_IN1
        */
        GPIO_InitStruct.Pin  = GPIO_PIN_0|GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* ADC1 DMA Init */
        /* ADC1 Init */
        hdma_adc1.Instance = DMA2_Stream0;
        hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode                = DMA_CIRCULAR;
        hdma_adc1.Init.Priority            = DMA_PRIORITY_MEDIUM;
        hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
          Error_Handler();
        }

        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

        /* ADC1 interrupt Init */
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }

    else if(hadc->Instance==ADC2) {
        /* Peripheral clock enable */
        __HAL_RCC_ADC2_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**ADC2 GPIO Configuration
        PC0     ------> ADC2_IN10
        PC1     ------> ADC2_IN11
        */
        GPIO_InitStruct.Pin  = GPIO_PIN_0|GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        hdma_adc2.Instance = DMA2_Stream2;
        hdma_adc2.Init.Channel             = DMA_CHANNEL_1;
        hdma_adc2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc2.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc2.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc2.Init.Mode                = DMA_CIRCULAR;
        hdma_adc2.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_adc2.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        hdma_adc2.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_HALFFULL;
        if(HAL_DMA_Init(&hdma_adc2)) {
            Error_Handler();
        }
        
        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);

        /* ADC2 interrupt Init */
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }
}


/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void 
HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {
  if(hadc->Instance==ADC1) {
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);

    /* ADC1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  }

  else if(hadc->Instance==ADC2) {
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();
      /**ADC2 GPIO Configuration
      PC0     ------> ADC2_IN10
      PC1     ------> ADC2_IN11
      */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1);

    /* ADC2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  }
}


/**
* @brief DAC MSP Initialization
* @param hdac: DAC handle pointer
* @retval None
*/
void 
HAL_DAC_MspInit(DAC_HandleTypeDef* hdac) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hdac->Instance==DAC1) {
        /* USER CODE BEGIN DAC_MspInit 0 */
        /* USER CODE END DAC_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_DAC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**DAC GPIO Configuration
            PA4     ------> DAC_OUT1
            PA5     ------> DAC_OUT2
        */
        #ifdef DAC_CHAN1
            GPIO_InitStruct.Pin  = GPIO_PIN_4;
        #else
            GPIO_InitStruct.Pin  = GPIO_PIN_5;
        #endif
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* DAC DMA Init */
        /* DMA controller clock enable */
        __HAL_RCC_DMA1_CLK_ENABLE();
        
        #ifdef DAC_CHAN1
            hdma_dac.Instance = DMA1_Stream5;
        #else
            hdma_dac.Instance = DMA1_Stream6;
        #endif
        hdma_dac.Init.Channel             = DMA_CHANNEL_7;
        hdma_dac.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_dac.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_dac.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_dac.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_dac.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_dac.Init.Mode                = DMA_CIRCULAR;
        hdma_dac.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_dac.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_dac) != HAL_OK) {
            Error_Handler();
        }

        #ifdef DAC_CHAN1
            __HAL_LINKDMA(hdac, DMA_Handle1, hdma_dac);
        #else
            __HAL_LINKDMA(hdac, DMA_Handle2, hdma_dac);
        #endif
        /* DAC interrupt Init */
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
        /* USER CODE BEGIN DAC_MspInit 1 */
        /* USER CODE END DAC_MspInit 1 */
    }
}


/**
* @brief DAC MSP De-Initialization
* @param hdac: DAC handle pointer
* @retval None
*/
void 
HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac) {
    if(hdac->Instance==DAC1) {
        /* USER CODE BEGIN DAC_MspDeInit 0 */
        /* USER CODE END DAC_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_DAC_CLK_DISABLE();

        #ifdef DAC_CHAN1
            HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
            HAL_DMA_DeInit(hdac->DMA_Handle1);
        #else
            HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
            HAL_DMA_DeInit(hdac->DMA_Handle2);
        #endif
        /* DAC interrupt DeInit */
        HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
        /* USER CODE BEGIN DAC_MspDeInit 1 */
        /* USER CODE END DAC_MspDeInit 1 */
    }
}


/**
* @brief TIM_Base MSP Initialization
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void
HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {

  if(htim_base->Instance==TIM2) {
    /* USER CODE BEGIN TIM2_MspInit 0 */
    /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    /* USER CODE BEGIN TIM2_MspInit 1 */
    /* USER CODE END TIM2_MspInit 1 */
  }

  if(htim_base->Instance==TIM3) {
    /* USER CODE BEGIN TIM3_MspInit 0 */
    /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    /* USER CODE BEGIN TIM3_MspInit 1 */
    /* USER CODE END TIM3_MspInit 1 */
  }

}


/**
* @brief TIM_Base MSP De-Initialization
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void 
HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
  if(htim_base->Instance==TIM2) {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */
    /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    /* USER CODE BEGIN TIM2_MspDeInit 1 */
    /* USER CODE END TIM2_MspDeInit 1 */
  }
  if(htim_base->Instance==TIM3) {
    /* USER CODE BEGIN TIM3_MspDeInit 0 */
    /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
    /* TIM3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    /* USER CODE BEGIN TIM3_MspDeInit 1 */
    /* USER CODE END TIM3_MspDeInit 1 */
  }

}


/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void 
HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(huart->Instance==USART2) {
        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        GPIO_InitStruct.Pin       = USART_TX_Pin|USART_RX_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART2 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        /* USER CODE BEGIN USART2_MspInit 1 */
        /* USER CODE END USART2_MspInit 1 */
    }
}


/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void 
HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
    if(huart->Instance==USART2) {
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();

        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

        /* USART2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
}


/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
