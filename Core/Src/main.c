#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Used Pins:
// ===========================================
// PA0  ADC1_IN0 Analog Input Values (Ramp)
// PA1  ADC1_IN1 Analog Input Values (Sensor)
// PA2  USART2_TX
// PA3  USART2_RX
// PA4  DAC_OUT1 Ramp Generator
// PA5  DAC_OUT2 Ramp Generator
// PB4  Ramp Min Push Button
// PB5  Ramp Max Push Button
// PB6  Start Ramp Push Button
// PB13 Ramp Running Led Indicator
// PB14 Ramp At Min Led Indicator
// PB15 Ramp At Max Led Indicator
// PC0  ADC2_IN10 Ramp Min Value Selection
// PC1  ADC2_IN11 Ramp Max Value Selection
// PC13 Blue Push Button
// ===========================================


// =========================================================
// DAC  Out2 ==> PA5 Ramp Generator
// ADC1 In0  ==> PA0 Ramp Input Values
// ADC1 In1  ==> PA1 Sensor Input Values
// ADC2_In10 ==> PC0 Ramp Min Value Selection
// ADC2_In11 ==> PC1 Ramp Max Value Selection
// LD2 Disabled because Conflicting with DAC Out2 <<=======
//==========================================================//
// ATTENZIONE:                                              //
// L'uscita 2 del DAC è connessa a PA5 che è FISICAMENTE    //
// connesso alla serie R31 (510 OHM) --> LD2. a meno di non //
// Interrompere il "soldering Bridge" SB21 (0 Ohm)          //
// Questo comporta che il DAC è "caricato" e non riesce     //
// ad erogare tutti i 3.3V che dovrebbe.                    //
//==========================================================//

DAC_HandleTypeDef  hdac;
DMA_HandleTypeDef  hdma_dac;

TIM_HandleTypeDef  htim2;

UART_HandleTypeDef huart2;


static void SystemClockHSE_Config(void) ;
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void buildRamp(uint16_t min, uint16_t max);
static void startAcquisition();
static void stopAcquisition();
static void execCommand();

//#define DEBUG        // Define this if debugging with a LED connected to DAC Out

#define BAUD_RATE 115200 //921600 //115200 //9600 //115200 //230400 //921600

#define HSE_BYPASS
//#define DAC_BUFFERED
#define TRIM_SAMPLING_FREQUENCY 10
#ifdef DEBUG
    #define RAMP_FREQUENCY 1 // Hz
#else
    #define RAMP_FREQUENCY 20 // Hz
#endif

typedef uint8_t bool;
#define false 0
#define true  !false

#define NS 4096
uint16_t Ramp[NS];      // Output Ramp

uint16_t rampMin;
uint16_t rampMax;

__IO bool pbPressed     = false;
__IO bool bCharPresent  = false;
__IO bool bUartReady    = false;


uint8_t rxBuffer[1];
uint8_t command;


void
buildRamp(uint16_t min, uint16_t max) {
    float factor = (float)(max-min)/(float)NS;
    for(int16_t i=0; i<NS; i++) {
        Ramp[i] = (uint16_t)(min+factor*i);
        //Ramp[i] = (uint16_t)(max-factor*i); // Rampa inversa...
    }
}


void
startAcquisition() {
    if(HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL, (uint32_t*)Ramp, NS, DAC_ALIGN_12B_R))
        Error_Handler(); 

    if(HAL_TIM_Base_Start(&htim2))
        Error_Handler();
    
    // start pwm generation (is This needed ?)
    // if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1))
    //     Error_Handler();
}


void
stopAcquisition() {
    if(HAL_TIM_Base_Stop(&htim2))
        Error_Handler();
    if(HAL_DAC_Stop_DMA(&hdac, DAC1_CHANNEL))
        Error_Handler(); 
}


static void
execCommand() {
    if(command == 'S') {
        stopAcquisition();
        buildRamp(rampMin, rampMax);
        startAcquisition();
        HAL_GPIO_WritePin(GPIOB, RampMinLed_Pin,   GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, RampMaxLed_Pin,   GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, RampStartLed_Pin, GPIO_PIN_SET);
    } // Command "S"
    else if(command == 'R') {
        stopAcquisition();
        buildRamp(rampMin, rampMax);
        startAcquisition();
        HAL_GPIO_WritePin(GPIOB, RampMinLed_Pin,   GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, RampMaxLed_Pin,   GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, RampStartLed_Pin, GPIO_PIN_SET);
    } // Command "R"
    else if(command == 'A') {
        stopAcquisition();
        buildRamp(rampMin, rampMax);
        startAcquisition();
        HAL_GPIO_WritePin(GPIOB, RampMinLed_Pin,   GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, RampMaxLed_Pin,   GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, RampStartLed_Pin, GPIO_PIN_SET);
    } // Command "A"
    else if(command == 'M') {
        stopAcquisition();
        buildRamp(rampMax, rampMax);
        startAcquisition();
        HAL_GPIO_WritePin(GPIOB, RampMinLed_Pin,   GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, RampMaxLed_Pin,   GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, RampStartLed_Pin, GPIO_PIN_RESET);

    } // Command "M"
    else if(command == 'm') {
        stopAcquisition();
        buildRamp(rampMin, rampMin);
        startAcquisition();
        HAL_GPIO_WritePin(GPIOB, RampMinLed_Pin,   GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, RampMaxLed_Pin,   GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, RampStartLed_Pin, GPIO_PIN_RESET);
    } // Command "m"
}


int 
main(void) {
    HAL_Init();
    SystemClockHSE_Config();
//    HAL_Delay(3000);
    
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_DAC_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    while(HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY);
    bCharPresent = false;
    if(HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuffer, 1) != HAL_OK) {
        Error_Handler();
    }

    rampMin = 0;
    rampMax = 4095;
    command = 'R';
    execCommand();

    while(true) {
        if(pbPressed) {
            pbPressed = false;
            execCommand();
        }
        if(bCharPresent) {
            bCharPresent = false;
            command = rxBuffer[0];
            execCommand();
            if(HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuffer, 1) != HAL_OK) {
                Error_Handler();
            }
        }
    } // while(true)
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE_CRYSTAL or HSE_BYPASS) 
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void
SystemClockHSE_Config(void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;
    
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* -1- Select HSI as system clock source to allow modification of the PLL configuration */
  RCC_ClkInitStruct.ClockType    = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
  
  /* -2- Enable HSE Oscillator, select it as PLL source and finally activate the PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  
#ifdef HSE_CRYSTAL  
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
#elif defined (HSE_BYPASS)
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
#endif /* HSE_CRYSTAL */
  RCC_OscInitStruct.PLL.PLLState  = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM      = 8;
  RCC_OscInitStruct.PLL.PLLN      = 360;
  RCC_OscInitStruct.PLL.PLLP      = RCC_PLLP_DIV2; 
  RCC_OscInitStruct.PLL.PLLQ      = 7;
  RCC_OscInitStruct.PLL.PLLR      = 6;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
  }
  
   /* Activate the OverDrive to reach the 180 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK) {
        while(1) { ; }
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | 
                                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
      Error_Handler();
  }
  
  /* -4- Optional: Disable HSI Oscillator (if the HSI is no more needed by the application) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState       = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
  }
}


static void
MX_DAC_Init(void) {
    DAC_ChannelConfTypeDef sConfig = {0};
    hdac.Instance = DAC1;
    if (HAL_DAC_Init(&hdac) != HAL_OK) {
        Error_Handler();
    }
    sConfig.DAC_Trigger          = DAC_TRIGGER_T2_TRGO;
    #ifdef DAC_BUFFERED
        sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    #else
        sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    #endif
    #ifdef DAC_CHAN1
        if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
            Error_Handler();
        }
    #else
        if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
            Error_Handler();
        }
    #endif
}


static void
MX_TIM2_Init(void) {
    // Timer2 is a 32 bit timer
    // Timer2 internal Clock is APB1 Clock
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    uint32_t prescalerValue = 1;
    uint32_t periodValue = (uint32_t)((clock)/(RAMP_FREQUENCY*NS));
    if(periodValue < 2) {
        Error_Handler();
    }
    periodValue -= 1;

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler         = prescalerValue;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = periodValue;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    // if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    //     Error_Handler();
    // }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}


static void 
MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate     = BAUD_RATE;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}


static void 
MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE(); // Used by DAC & UART2
    __HAL_RCC_DMA2_CLK_ENABLE(); // Used by ADC1 & ADC2
    #ifdef DAC_CHAN1
        /* DMA1_Stream5_IRQn interrupt configuration (DAC) */
        HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    #else
        /* DMA1_Stream6_IRQn interrupt configuration (DAC) */
        HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    #endif
}


static void
MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    #ifdef DAC_CHAN1
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin   = LD2_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
    #endif

    // Led Indicators: RampMinLed_Pin, RampMaxLed_Pin, RampStartLed_Pin
    GPIO_InitStruct.Pin   = RampMinLed_Pin | RampMaxLed_Pin | RampStartLed_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, RampMinLed_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, RampMaxLed_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, RampStartLed_Pin, GPIO_PIN_RESET);

    // Ramp Push Buttons: RampMinPB_Pin, RampMaxPB_Pin, RampStartPB_Pin
    GPIO_InitStruct.Pin = RampMinPB_Pin | RampMaxPB_Pin | RampStartPB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Blue Push Button
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    // EXTI interrupt init
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


void 
Error_Handler(void) {
    __disable_irq();
    while (1) {
        for(int i=0; i<10; i++) {
            #ifdef DAC_CHAN1
                 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            #endif
            for(int x=0; x<200; x++) {
                for(int j=0; j<15000; j++) {
                    asm __volatile__ ("nop");
                }
            }
        }
        for(int i=0; i<10; i++) {
            #ifdef DAC_CHAN1
                 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            #endif
            for(int x=0; x<600; x++) {
                for(int j=0; j<15000; j++) {
                    asm __volatile__ ("nop");
                }
            }
        }
    }
}


#ifdef  USE_FULL_ASSERT
void
assert_failed(uint8_t *file, uint32_t line) {
}
#endif /* USE_FULL_ASSERT */


/// Push Buttons callback
void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == B1_Pin) // On Board Blue Push Button
        command = 'S';
    else if(GPIO_Pin == RampMinPB_Pin)
        command = 'm';
    else if(GPIO_Pin == RampMaxPB_Pin)
        command = 'M';
    else if(GPIO_Pin == RampStartPB_Pin)
        command = 'R';
    else
        return;
    pbPressed = true;
}


/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
    /* Set transmission flag: transfer complete*/
    bUartReady = true;
}


/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void
HAL_UART_RxCpltCallback(UART_HandleTypeDef* UartHandle) {
    bCharPresent = true;
}


/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void 
HAL_UART_ErrorCallback(UART_HandleTypeDef* UartHandle) {
    Error_Handler();
}


