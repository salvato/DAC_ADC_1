#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Used Pins:
// ===========================================
// PA0  ADC1_IN0 Analog Input Values
// PA1  ADC1_IN1 Analog Input Values
// PA2  USART2_TX
// PA3  USART2_RX
// PA5  DAC_OUT2 Ramp Generator
// PC0  ADC2_IN10 Ramp Min Value Selection
// PC1  ADC2_IN11 Ramp Max Value Selection
// PC13 Blue Push Button


// =========================================================
// DAC  Out2 ==> PA5 Ramp Generator
// ADC1 In0  ==> PA0 Analog Input Values
// ADC1 In1  ==> PA1 Analog Input Values
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

ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_adc1;

ADC_HandleTypeDef  hadc2;
DMA_HandleTypeDef  hdma_adc2;

TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim3;

UART_HandleTypeDef huart2;


static void SystemClockHSE_Config(void) ;
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void buildRamp(uint16_t min, uint16_t max);
static void handlePotVals(int np, uint16_t* trimVal);
static void startAcquisition();
static void stopAcquisition();
static void execCommand();
static void transmitData();
static void transmitAscii();

//#define DEBUG        // Define this if debugging with a LED connected to DAC Out

#define BAUD_RATE 115200//921600 //115200 //9600 //115200 //230400 //921600

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
uint16_t adc1Val[4*NS]; // Space for two Ramps (double buffer)
uint32_t sensBuff[NS];
uint32_t rampBuff[NS];
uint32_t nAvgSens;
uint32_t maxAvgSens = (0xFFFFFFFF >> 12)-2;
uint32_t avgSens[NS];
uint32_t avgRamp[NS];

#define AVERNUM RAMP_FREQUENCY
uint16_t nAvg;

#define ADC_RAMP_BUFFER_LENGTH 16
uint16_t adc2Val[2*ADC_RAMP_BUFFER_LENGTH];

uint16_t rampMin = 3000;
uint16_t rampMax = 3500;

__IO bool adc1HalfReady = false;
__IO bool adc1FullReady = false;
__IO bool adc2HalfReady = false;
__IO bool adc2FullReady = false;
__IO bool pbPressed     = false;
__IO bool bUartReady    = false;


uint8_t outBuff[80];
uint8_t rxBuffer[1];


void
buildRamp(uint16_t min, uint16_t max) {
    float factor = (float)(max-min)/(float)NS;
    for(int16_t i=0; i<NS; i++) {
        Ramp[i] = (uint16_t)(min+factor*i);
        //Ramp[i] = (uint16_t)(max-factor*i); // Rampa inversa...
    }
}


void
handlePotVals(int np, uint16_t* trimVal) {
    return;
    uint32_t min = 0;
    uint32_t max = 0;
    for(int i=0; i<np; i+=2) {
        min += trimVal[i];
        max += trimVal[i+1];
    }
    min /= np/2;
    max /= np/2;
    min = min >> 3;        // Max RampMin = 511
    max = 3584+(max >> 3); // Min RampMax = 4095-511
    if((abs(min-rampMin) > 2) || (abs(max-rampMax) > 2)) {
        rampMin = min;
        rampMax = max;
        buildRamp(rampMin, rampMax);
        // sprintf((char*)outBuff, "Ramp Min=%d Ramp Max=%d\n\r", rampMin, rampMax);
        // HAL_UART_Transmit(&huart2, (uint8_t*)outBuff, strlen((char*)outBuff), 10);
    }
}


void
transmitData() {
    uint32_t nBytes = sizeof(avgRamp);
    for(int i=0; i<NS; i++) {
        avgRamp[i] /= nAvgSens;
        avgSens[i] /= nAvgSens;
    }
    // HAL_UART_Transmit(&huart2, (uint8_t*)&nBytes, sizeof(uint32_t), 10);
    HAL_UART_Transmit(&huart2, (uint8_t*)avgRamp, nBytes, 3000);  
    // HAL_UART_Transmit(&huart2, (uint8_t*)&nBytes, sizeof(uint32_t), 10);
    HAL_UART_Transmit(&huart2, (uint8_t*)avgSens, nBytes, 3000);
}


void
transmitAscii() {
    for(int i=0; i<NS; i++) {
        sprintf((char*)outBuff, "i=%d Ramp=%d Dac=%ld Sensor=%ld\n\r",
                        i, Ramp[i], avgRamp[i]/nAvgSens, avgSens[i]/nAvgSens);
        HAL_UART_Transmit(&huart2, (uint8_t*)outBuff, strlen((char*)outBuff), 10);
     }
 }


void
startAcquisition() {
    nAvgSens = 0;
    memset(avgSens, 0, sizeof(avgSens));
    memset(avgRamp, 0, sizeof(avgRamp));

    if(HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL, (uint32_t*)Ramp, NS, DAC_ALIGN_12B_R))
        Error_Handler(); 

    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc1Val, 4*NS))
        Error_Handler(); 

    if(HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc2Val, 2*ADC_RAMP_BUFFER_LENGTH))
        Error_Handler(); 

    if(HAL_TIM_Base_Start(&htim2))
        Error_Handler();

    if(HAL_TIM_Base_Start(&htim3))
        Error_Handler();
    
    // start pwm generation (is This needed ?)
    // if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1))
    //     Error_Handler();

    // start pwm generation (is This needed ?)
    // if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1))
    //     Error_Handler();
}


void
stopAcquisition() {
    if(HAL_TIM_Base_Stop(&htim2))
        Error_Handler();
    if(HAL_TIM_Base_Stop(&htim3))
        Error_Handler();
    if(HAL_DAC_Stop_DMA(&hdac, DAC1_CHANNEL))
        Error_Handler(); 
    if(HAL_ADC_Stop_DMA(&hadc1))
        Error_Handler(); 
    if(HAL_ADC_Stop_DMA(&hadc2))
        Error_Handler(); 
}


static void
execCommand() {
    if(rxBuffer[0] == 'S') {
        stopAcquisition();
        // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        transmitData();
        startAcquisition();
        // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    } // Command "S"
    else if(rxBuffer[0] == 'R') {
        stopAcquisition();
        buildRamp(rampMin, rampMax);
        startAcquisition();
    } // Command "R"
    else if(rxBuffer[0] == 'A') {
        stopAcquisition();
        transmitAscii();
        startAcquisition();
    } // Command "A"
    else if(rxBuffer[0] == 'M') {
        stopAcquisition();
        buildRamp(rampMax, rampMax);
        startAcquisition();
    } // Command "M"
    else if(rxBuffer[0] == 'm') {
        stopAcquisition();
        buildRamp(rampMin, rampMin);
        startAcquisition();
    } // Command "m"
}


int 
main(void) {
    rampMin = 0;
    rampMax = 4095;
    buildRamp(rampMin, rampMax);
    adc1HalfReady=false;
    adc1FullReady=false;
    adc2HalfReady=false;
    adc2FullReady=false;
    nAvg = 0;

    HAL_Init();

    SystemClockHSE_Config();
    HAL_Delay(3000);
    
    MX_GPIO_Init();
    // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_DAC_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();

    while(HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY);
    bool bNewData = false;

    startAcquisition(); // It also set nAvgSens=0 and sets 
                        // the vectors avgSens[] and avgRamp[] to zero;
    bUartReady = false;
    if(HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuffer, 1) != HAL_OK) {
        Error_Handler();
    }

    while (1) {
        if(adc1HalfReady) {
            adc1HalfReady = false;
            #ifdef DAC_CHAN1
                HAL_GPIO_TogglePin (LD2_GPIO_Port, LD2_Pin);
            #endif
            for(int i=0; i<NS; i++) {
                avgRamp[i] += adc1Val[2*i];
                avgSens[i] += adc1Val[2*i+1];
            }
            nAvgSens++;
            if(nAvgSens > maxAvgSens) {
                stopAcquisition();
                pbPressed = true;
            }
            bNewData = true;
        }
        if(adc1FullReady) {
            adc1FullReady = false;
            #ifdef DAC_CHAN1
                HAL_GPIO_TogglePin (LD2_GPIO_Port, LD2_Pin);
            #endif
            for(int i=0; i<NS; i++) {
                avgRamp[i] += adc1Val[2*NS+2*i];
                avgSens[i] += adc1Val[2*NS+2*i+1];
            }
            nAvgSens++;
            if(nAvgSens > maxAvgSens) {
                stopAcquisition();
                pbPressed = true;
            }
            bNewData = true;
        }

        if(adc2HalfReady) {
            adc2HalfReady = false;
            // HAL_GPIO_TogglePin (LD2_GPIO_Port, LD2_Pin);
            handlePotVals(ADC_RAMP_BUFFER_LENGTH, adc2Val);
        }
        if(adc2FullReady) {
            adc2FullReady = false;
            // HAL_GPIO_TogglePin (LD2_GPIO_Port, LD2_Pin);
            handlePotVals(ADC_RAMP_BUFFER_LENGTH, &adc2Val[ADC_RAMP_BUFFER_LENGTH]);
        }
        if(bNewData) {
            bNewData = false;
        }
        if(pbPressed) {
            stopAcquisition();
            // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            // uint32_t ticks = HAL_GetTick();
            transmitData();
            // ticks = HAL_GetTick() - ticks;
            pbPressed = false;
            startAcquisition();
            // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }
        if(bUartReady) {
            bUartReady = false;
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
MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4; // The clock is common for all the ADCs.
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 2;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
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
    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = 2;
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
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4; // The clock is common for all the ADCs.
    hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode          = ENABLE; // In scan mode, automatic conversion is performed on a selected group of analog inputs.
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion       = 2;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel      = ADC_CHANNEL_10;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfig.Channel      = ADC_CHANNEL_11;
    sConfig.Rank         = 2;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
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


/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void 
MX_TIM3_Init(void) {
    // Timer3 is a 16 bit timer
    // Timer3 internal Clock is APB2 Clock ???
    uint32_t clock = HAL_RCC_GetPCLK2Freq();
    uint32_t prescalerValue = 4000;
    uint32_t periodValue    = (uint32_t)((clock/(prescalerValue-1))/(TRIM_SAMPLING_FREQUENCY*ADC_RAMP_BUFFER_LENGTH));
    if(periodValue < 2) {
        Error_Handler();
    }
    periodValue -= 1;

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};
    TIM_OC_InitTypeDef sConfigOC              = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler         = prescalerValue;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = periodValue;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
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

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init */
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
        // HAL_TIM_Base_Stop(&htim2);
        adc1FullReady = true;
    }
    else if(hAdc == &hadc2) {
        adc2FullReady = true;
    }
}


/// Blue Push Button callback
void 
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
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
    /* Set transmission flag: transfer complete*/
    bUartReady = true;
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


