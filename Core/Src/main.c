#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


// =======================================================
// Per comunicare con il dispositivo tramite porta USB
// attraverso il programa "minicom" su Linux:
//
// $ minicom -D /dev/ttyACM0 -b 115200
// =======================================================


 
// =======================================================
// // Modificato main.h per scegliere il canale DAC_CHAN2.
// !!! Ho interrotto, sulla Board, SB21 in modo da 
//     scollegare LD2 da PA5 (DAC_OUT2) !!!
// =======================================================


// Used Pins:
// ===============================================================================+
// PA0  ADC1_IN0 Analog Input Values (Ramp)     (A0  sul connettore Arduino CN8)  |
// PA1  ADC1_IN1 Analog Input Values (Sensor)   (A1  sul connettore Arduino CN8)  |
// PA2  USART2_TX                               (D1  sul connettore Arduino CN9)  |
// PA3  USART2_RX                               (D0  sul connettore Arduino CN9)  |   
// PA4  DAC_OUT1 Ramp Generator                 (A2  sul connettore Arduino CN8)  |   
// PA5  DAC_OUT2 Ramp Generator                 (D13 sul connettore Arduino CN9)  |
// PA10 Ramp Trigger Output                     (D2  sul connettore Arduino CN9)  |
// PB4  Ramp Min Push Button                    (D5  sul connettore Arduino CN9)  |
// PB5  Ramp Max Push Button                    (D4  sul connettore Arduino CN9)  |
// PB6  Start Ramp Push Button                  (D10 sul connettore Arduino CN5)  |
// PB13 Ramp Running Led Indicator              (PIN 30 sul connettore CN10)      |
// PB14 Ramp At Min Led Indicator               (PIN 28 sul connettore CN10)      |
// PB15 Ramp At Max Led Indicator               (PIN 26 sul connettore CN10)      |
// PC13 Blue Push Button                                                          |
// ===============================================================================+


// ==========================================================+
// DAC  Out2 ==> PA5 Ramp Generator                          |
// ADC1 In0  ==> PA0 Ramp Input Values                       |
// ADC1 In1  ==> PA1 Sensor Input Values                     |
// LD2 Disabled since it conflicts with DAC Out2 <<=======   |
//===========================================================+
// ATTENZIONE:                                               |
// L'uscita 2 del DAC è connessa a PA5 che è FISICAMENTE     |
// connesso alla serie R31 (510 OHM) --> LD2. a meno di non  |
// Interrompere il "soldering Bridge" SB21 (0 Ohm)           |
// Questo comporta che il DAC è "caricato" e non riesce      |
// ad erogare tutti i 3.3V che dovrebbe.                     |
//                                                           |
// La scelta tra DAC Out1 e DAC Out2 dipende dalla           |
// definizione di DAC_CHAN1 in "main.h"                      | 
//===========================================================+

//============
// Error Codes
//============
#define ERROR_NONE           0
#define ERROR_DAC_INIT       1
#define ERROR_DAC_CHANNEL    2
#define ERROR_TIM2_INIT      3
#define ERROR_TIM2_START     4
#define ERROR_TIM2_STOP      5
#define ERROR_UART2_INIT     6 
#define ERROR_START_ACQ      7
#define ERROR_STOP_ACQ       8
#define ERROR_UART_TX        9
#define ERROR_UART_RX       10
#define ERROR_UART_CB       11
#define ERROR_UNKNOWN       20

//#define DEBUG        // Define this if debugging with a LED connected to DAC Out

#define BAUD_RATE 115200 //9600 //115200 //230400 //921600

#define HSE_BYPASS

// Using the DAC Buffer reduce the output impedance of the DAC output
// but reduces the maximum output voltage swing.
// If the DAC output is connected to a high impedance load (e.g. an ADC input)
// the buffer can be disabled to have the full output voltage swing (0 - 3.3V)
//#define DAC_BUFFERED

#define TRIM_SAMPLING_FREQUENCY 10
#ifdef DEBUG
    #define RAMP_FREQUENCY 1 // Hz
#else
    #define RAMP_FREQUENCY 20 // Hz
#endif

int errorCode = ERROR_NONE;

//==========================
// Peripheral Handles
//==========================
DAC_HandleTypeDef  hdac;
DMA_HandleTypeDef  hdma_dac;
ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_adc1;
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart2;
//DMA_HandleTypeDef  hdma_usart2_tx; // e' in conflitto con il canale 2 del DAC !!!

//==========================
// Function Prototypes
//==========================
void Error_Handler(void);
static void SystemClockHSE_Config(void) ;
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void buildRamp(uint16_t min, uint16_t max);
static void startAcquisition();
static void stopAcquisition();
static void execCommand();

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

uint16_t rampMin;
uint16_t rampMax;

__IO bool pbPressed     = false;
__IO bool bCharPresent  = false;
__IO bool bUartReady    = false;
__IO bool adc1HalfReady = false;
__IO bool adc1FullReady = false;


uint8_t outBuff[80];
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
    nAvgSens = 0;
    memset(avgSens, 0, sizeof(avgSens));
    memset(avgRamp, 0, sizeof(avgRamp));

    HAL_GPIO_WritePin(RampTrigger_GPIO_Port, RampTrigger_Pin, GPIO_PIN_RESET);

    if(HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL, (uint32_t*)Ramp, NS, DAC_ALIGN_12B_R)) {
        errorCode = ERROR_START_ACQ;
        Error_Handler();
    }

    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc1Val, 4*NS)) {
        errorCode = ERROR_START_ACQ;
        Error_Handler(); 
    }

    if(HAL_TIM_Base_Start(&htim2)) {
        errorCode = ERROR_TIM2_START;
        Error_Handler();
    }
}


void
stopAcquisition() {
    HAL_GPIO_WritePin(RampTrigger_GPIO_Port, RampTrigger_Pin, GPIO_PIN_RESET);
    if(HAL_TIM_Base_Stop(&htim2)) {
        errorCode = ERROR_TIM2_STOP;
        Error_Handler();
    }
    if(HAL_ADC_Stop_DMA(&hadc1)) {
        errorCode = ERROR_STOP_ACQ;
        Error_Handler();
    }
    if(HAL_DAC_Stop_DMA(&hdac, DAC1_CHANNEL)) {
        errorCode = ERROR_STOP_ACQ;
        Error_Handler();
    }
}


static void
execCommand() {
    //sprintf((char*)outBuff, "%c\n\r", (char)command);
    //HAL_UART_Transmit(&huart2, (uint8_t*)outBuff, strlen((char*)outBuff), 10);
    if(command == 'S') {
        stopAcquisition();
        for(int i=0; i<NS; i++) {
            avgSens[i] /= nAvgSens;
        }
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t*)avgSens, NS*4); // E' in conflitto con il DAC Canale 2 !!!
        HAL_UART_Transmit(&huart2, (uint8_t*)avgSens, NS*4, 5000);
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

    } // Command "m"

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
    // WAITING FOR A STABLE POWER: Power Supply is VERY BAD.
    HAL_Delay(2000); 

    SystemClockHSE_Config();
    
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_DAC_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    while(HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY);
    bCharPresent = false;
    if(HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuffer, 1) != HAL_OK) {
        errorCode = ERROR_UART_RX;
        Error_Handler();
    }

    rampMin = 0;
    rampMax = 4095;
    buildRamp(rampMin, rampMax);

    adc1HalfReady=false;
    adc1FullReady=false;
    nAvgSens = 0;

    startAcquisition();
    HAL_GPIO_WritePin(GPIOB, RampMinLed_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, RampMaxLed_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, RampStartLed_Pin, GPIO_PIN_SET);

    while(true) {

        if(adc1HalfReady) {
            adc1HalfReady = false;
            for(int i=0; i<NS; i++) {
                avgRamp[i] += adc1Val[2*i];
                avgSens[i] += adc1Val[2*i+1];
            }
            nAvgSens++;
            if(nAvgSens > maxAvgSens) {
                stopAcquisition();
                pbPressed = true;
            }
        }

        if(adc1FullReady) {
            adc1FullReady = false;
            for(int i=0; i<NS; i++) {
                avgRamp[i] += adc1Val[2*NS+2*i];
                avgSens[i] += adc1Val[2*NS+2*i+1];
            }
            nAvgSens++;
            if(nAvgSens > maxAvgSens) {
                stopAcquisition();
                pbPressed = true;
            }
        }

        if(pbPressed) {
            pbPressed = false;
            execCommand();
        }

        if(bCharPresent) {
            bCharPresent = false;
            command = rxBuffer[0];
            execCommand();
            if(HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuffer, 1) != HAL_OK) {
                errorCode = ERROR_UART_RX;
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
        Error_Handler();
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


static void
MX_DAC_Init(void) {
    DAC_ChannelConfTypeDef sConfig = {0};
    hdac.Instance = DAC1;
    if (HAL_DAC_Init(&hdac) != HAL_OK) {
        errorCode = ERROR_DAC_INIT;
        Error_Handler();
    }
//==========================================================================================
//  Each time the DAC detects a rising edge on the selected timer TRGO output (T2_TRGO), the
//  last data stored into the DAC_DHRx register are transferred into the DAC_DORx register.
//==========================================================================================
    sConfig.DAC_Trigger          = DAC_TRIGGER_T2_TRGO;
    #ifdef DAC_BUFFERED
        sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    #else
        sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    #endif
    #ifdef DAC_CHAN1
        if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
            errorCode = ERROR_DAC_CHANNEL;
            Error_Handler();
        }
    #else
        if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
            errorCode = ERROR_DAC_CHANNEL;
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
        errorCode = ERROR_TIM2_INIT;
        Error_Handler();
    }
    periodValue -= 1;

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler         = prescalerValue;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = periodValue;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        errorCode = ERROR_TIM2_INIT;
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        errorCode = ERROR_TIM2_INIT;
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        errorCode = ERROR_TIM2_INIT;
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
        errorCode = ERROR_UART2_INIT;
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

    /* DMA1_Stream6_IRQn interrupt configuration (USART2_TX)  in conflitto col DAC canale 2 */
    //HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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

    // Ramp Trigger Output Pin (RampTrigger_Pin)
    GPIO_InitStruct.Pin   = RampTrigger_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RampTrigger_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(RampTrigger_GPIO_Port, RampTrigger_Pin, GPIO_PIN_RESET);

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
    while(1) {
        #ifdef DAC_CHAN1
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        #else
                HAL_GPIO_WritePin(RampTrigger_GPIO_Port, RampTrigger_Pin, GPIO_PIN_RESET);
        #endif
        for(int x=0; x<400; x++) {
            for(int j=0; j<50000; j++) {
                asm __volatile__ ("nop");
            }
        }

        // Blink slowly
        for(int i=0; i<2*errorCode; i++) {
            #ifdef DAC_CHAN1
                    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            #else
                    HAL_GPIO_TogglePin(RampTrigger_GPIO_Port, RampTrigger_Pin);
            #endif
            for(int x=0; x<200; x++) {
                for(int j=0; j<50000; j++) {
                    asm __volatile__ ("nop");
                }
            }
        }
        #ifdef DAC_CHAN1
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        #else
                HAL_GPIO_WritePin(RampTrigger_GPIO_Port, RampTrigger_Pin, GPIO_PIN_RESET);
        #endif
        for(int x=0; x<400; x++) {
            for(int j=0; j<50000; j++) {
                asm __volatile__ ("nop");
            }
        }
        // Blink rapidly
        for(int i=0; i<10; i++) {
            #ifdef DAC_CHAN1
                    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            #else
                    HAL_GPIO_TogglePin(RampTrigger_GPIO_Port, RampTrigger_Pin);
            #endif
            for(int x=0; x<600; x++) {
                for(int j=0; j<1500; j++) {
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
    errorCode = ERROR_UART_CB;
    Error_Handler();
}


/// ADC Conversion_Half_Complete callback
void
HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hAdc) {
    if(hAdc == &hadc1) {
        adc1HalfReady = true;
    }
}


/// ADC Conversion_Complete callback
void
HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hAdc) {
    if(hAdc == &hadc1) {
        // HAL_TIM_Base_Stop(&htim2);
        adc1FullReady = true;
    }
}


// Poichè non riesco a generare una secoda rampa sul device
// utilizzato per l'esperimento, introduco un trigger tramite
// una GPIO
#ifdef DAC_CHAN1
    void
    HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
        HAL_GPIO_WritePin(RampTrigger_GPIO_Port, RampTrigger_Pin, GPIO_PIN_SET);
    }


    void
    HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
        HAL_GPIO_WritePin(RampTrigger_GPIO_Port, RampTrigger_Pin, GPIO_PIN_RESET);
    }

#else // DAC_CHAN2

    void
    HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
        HAL_GPIO_WritePin(RampTrigger_GPIO_Port, RampTrigger_Pin, GPIO_PIN_SET);
    }


    void
    HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
        HAL_GPIO_WritePin(RampTrigger_GPIO_Port, RampTrigger_Pin, GPIO_PIN_RESET);
    }
#endif

// In conflitto con il DAC Canale 2
//void
//HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
//}

// End of File