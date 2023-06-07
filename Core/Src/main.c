#include "main.h"
#include <string.h>
#include <stdio.h>

ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_adc1;
ADC_HandleTypeDef  hadc2;
DMA_HandleTypeDef  hdma_adc2;

DAC_HandleTypeDef  hdac;
DMA_HandleTypeDef  hdma_dac1;
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void SystemClockHSE_Config(void) ;
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);

#define HSE_BYPASS
//#define OVERCLOCK    // Define this to increase CPU Clock
#define DEBUG        // Define this if debugging with a LED connected to DAC Out

#ifdef DEBUG
    #define RAMP_FREQUENCY 1 // Hz
#else
    #define RAMP_FREQUENCY 20 // Hz
#endif

#define DAC_BUFFERED

typedef uint8_t bool;
#define false 0
#define true  !false

#define NS 4096
uint16_t Ramp[NS];
uint16_t adc1Val[2*NS];
uint16_t txBuff[NS];
uint32_t avg[NS];

#define AVERNUM RAMP_FREQUENCY
uint16_t nAvg;

#define ADC_RAMP_BUFFER_LENGTH NS
uint16_t adc2Val[2*ADC_RAMP_BUFFER_LENGTH];
uint16_t rampMin = 0;
uint16_t rampMax = 4095;

__IO bool adc1HalfReady=false;
__IO bool adc1FullReady=false;
__IO bool adc2HalfReady=false;
__IO bool adc2FullReady=false;

char outBuff[80];


void
buildRamp(int16_t np, uint16_t min, uint16_t max, uint16_t* pRamp) {
    float factor = (float)(max-min)/(float)np;
    for(int16_t i=0; i<np; i++) {
        pRamp[i] = (uint16_t)(min+factor*i);
    }
}


// ADC1 In0  ==> PA0
// DAC  Out  ==> PA4
// ADC2_In14 ==> PC4
// ADC2_In15 ==> PC5


int 
main(void) {
    #ifdef DEBUG
        buildRamp(NS, rampMin, rampMax, Ramp);
    #else
        buildRamp(NS, rampMin, rampMax, Ramp);
    #endif
    adc1HalfReady=false;
    adc1FullReady=false;
    adc2HalfReady=false;
    adc2FullReady=false;
    memset(avg, 0, sizeof(avg));
    nAvg = 0;

    HAL_Init();
    // SystemClock_Config();
    SystemClockHSE_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_USART2_UART_Init();
    MX_DAC_Init();
    MX_TIM2_Init();

    if(HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Ramp, NS, DAC_ALIGN_12B_R))
        Error_Handler(); 

    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc1Val, 2*NS))
        Error_Handler(); 

    if(HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc2Val, 2*ADC_RAMP_BUFFER_LENGTH))
        Error_Handler(); 

    if(HAL_TIM_Base_Start(&htim2))
        Error_Handler();
    
    // start pwm generation (is This needed ?)
    // if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1))
    //     Error_Handler();

    if(HAL_ADC_Start_IT(&hadc2) != HAL_OK)
        Error_Handler();

    while (1) {
        if(adc1HalfReady) {
            adc1HalfReady = false;
            HAL_GPIO_TogglePin (LD2_GPIO_Port, LD2_Pin);
            // memcpy(txBuff, adc_val, NS*sizeof(*adc_val));
            // bNewData = true;
        }
        if(adc1FullReady) {
            adc1FullReady = false;
            HAL_GPIO_TogglePin (LD2_GPIO_Port, LD2_Pin);
            // memcpy(txBuff, &adc_val[NS], NS*sizeof(*adc_val));
            // bNewData = true;
        }
        if(adc2HalfReady) {
            adc2HalfReady = false;
            HAL_GPIO_TogglePin (LD2_GPIO_Port, LD2_Pin);
            // memcpy(txBuff, adc_val, NS*sizeof(*adc_val));
            // bNewData = true;
        }
        if(adc2FullReady) {
            adc2FullReady = false;
            HAL_GPIO_TogglePin (LD2_GPIO_Port, LD2_Pin);
            // memcpy(txBuff, &adc_val[NS], NS*sizeof(*adc_val));
            // bNewData = true;
        }
/*        
        else {
            if(bNewData) {
                bNewData = false;
                for(int i=0; i<NS; i++) {
                    avg[i] += txBuff[i];
                }
                nAvg++;
                if(nAvg >= AVERNUM) {
                    HAL_TIM_Base_Stop(&htim2);
                    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
                    HAL_ADC_Stop_DMA(&hadc1);
                    for(int i=0; i<NS; i++) {
                        avg[i] = avg[i]/AVERNUM;
                        //sprintf(outBuff, "i=%d f=%ld\n\r", i, avg[i]);
                        //HAL_UART_Transmit(&huart2, (uint8_t*)outBuff, strlen(outBuff), 10);
                    }
                    nAvg = 0;
                    memset(avg, 0, sizeof(avg));
                    ready_1_half = false;
                    ready_2_half = false;
                    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Ramp, NS, DAC_ALIGN_12B_R);
                    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 2*NS);
                    HAL_TIM_Base_Start(&htim2);
                }
            }
            // HAL_GPIO_TogglePin (LD2_GPIO_Port, LD2_Pin);
        }
        if(bNewRamp) {
            bNewRamp = false;
            // if(hadc2.NbrOfCurrentConversionRank == 1)
                rampMin = HAL_ADC_GetValue(&hadc2);
            // else if(hadc2.NbrOfCurrentConversionRank == 2)
                rampMax = HAL_ADC_GetValue(&hadc2);
            sprintf(outBuff, "Ramp Min=%d Ramp Max=%d\n\r", rampMin, rampMax);
            HAL_UART_Transmit(&huart2, (uint8_t*)outBuff, strlen(outBuff), 10);
            if(HAL_ADC_Start_IT(&hadc2) != HAL_OK)
                Error_Handler();
        }
*/
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
    /* Initialization Error */
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



#ifdef OVERCLOCK
void
SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configure the main internal regulator output voltage
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  // Initializes the RCC Oscillators according to the specified parameters
  // in the RCC_OscInitTypeDef structure.

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 16;
  RCC_OscInitStruct.PLL.PLLN            = 360;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = 7;
  RCC_OscInitStruct.PLL.PLLR            = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  // Activate the Over-Drive mode
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  // Initializes the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                     RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

#else // Not OVERCLOCK

void 
SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                       RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}
#endif // OVERCLOCK


static void
MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4; // The clock is common for all the ADCs.
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    // The total conversion time is calculated as follows:
    // Tconv = ADC_SAMPLETIME + 12 cycles
    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;//ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}


/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void 
MX_ADC2_Init(void) {

    /* USER CODE BEGIN ADC2_Init 0 */
    /* USER CODE END ADC2_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC2_Init 1 */
    /* USER CODE END ADC2_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4; // The clock is common for all the ADCs.
    hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode          = ENABLE; // In scan mode, automatic conversion is performed on a selected group of analog inputs.
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion       = 2;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel      = ADC_CHANNEL_14;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfig.Channel      = ADC_CHANNEL_15;
    sConfig.Rank         = 2;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}


static void
MX_DAC_Init(void) {
    DAC_ChannelConfTypeDef sConfig = {0};
    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK) {
        Error_Handler();
    }
    sConfig.DAC_Trigger          = DAC_TRIGGER_T2_TRGO;
    #ifdef DAC_BUFFERED
        sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    #else
        sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    #endif
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}


static void
MX_TIM2_Init(void) {
    uint32_t periodValue = (uint32_t)((SystemCoreClock)/(RAMP_FREQUENCY*NS));
    if(periodValue < 2) {
        Error_Handler();
    }
    periodValue -= 1;
    uint32_t prescalerValue = 0;

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
    // Serve ?
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
    huart2.Init.BaudRate     = 115200;
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
    __HAL_RCC_DMA1_CLK_ENABLE(); // Used by DAC
    __HAL_RCC_DMA2_CLK_ENABLE(); // Used by ADC1 & ADC2

    /* DMA1_Stream5_IRQn interrupt configuration (DAC) */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    /* DMA2_Stream0_IRQn interrupt configuration (ADC1) */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    /* DMA2_Stream2_IRQn interrupt configuration (ADC2) */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}


static void
MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}


void 
Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}


#ifdef  USE_FULL_ASSERT
void
assert_failed(uint8_t *file, uint32_t line) {
}
#endif /* USE_FULL_ASSERT */


/// ADC Conversion_Half_Complete callback
void
HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hAdc) {
    if(hAdc == &hadc1) {
        adc1HalfReady = true;
    }
    else if(hAdc == &hadc2) {
        adc2HalfReady = true;
    }
}


/// ADC Conversion_Complete callback
void
HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hAdc) {
    if(hAdc == &hadc1) {
        adc1FullReady = true;
    }
    else if(hAdc == &hadc2) {
        adc2FullReady = true;
    }
}

