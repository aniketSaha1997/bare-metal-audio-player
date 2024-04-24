/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adpcm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
uint16_t applyMovingAverageFilter(uint16_t newSample) ;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile tTwoByte newSample;
extern AudioElement audioFile;
extern uint8_t audioFileToPlay;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  uint8_t  adpcmSample;
  	static uint16_t pcmSample;
  	static uint8_t nibble = 1;
  	static uint8_t repetition = 0;
  	static uint16_t sample_position = 0;
  	static unsigned char *RawAudio;
  	static uint8_t PrevAudioFileToPlay = 0xFF;

  	if(PrevAudioFileToPlay != audioFileToPlay)
  	{
  		PrevAudioFileToPlay = audioFileToPlay;
  		nibble = 1;
  		repetition = 0;
  		sample_position = 0;
  		RawAudio = (unsigned char *)audioFile.AudioFiles[audioFileToPlay];
  	}

  	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE))
  	{
  		__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

  		if ((sample_position < audioFile.AudioSize[audioFileToPlay]))
  		{  // new sample is generated
  			repetition = 7;	// reinitialize repetition down counter
  			if (nibble)
  			{   // first 4 bits of the ADPCM byte decoded
  				adpcmSample = (uint8_t)(RawAudio[sample_position] >> 4);
  			}
  			else
  			{   // last 4 bits of the ADPCM byte decoded
  				adpcmSample = (uint8_t)(RawAudio[sample_position] & 0x0F);
  				sample_position++ ;
  			}

  			nibble = (uint8_t)(!nibble);/* indicator inverted mean next interrupt will handle
  																					 the second part of the byte.  */
  			pcmSample = ADPCM_Decode(adpcmSample);

  			// update sample
  			newSample.uShort = (uint16_t)32768 + pcmSample;

		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, newSample.uBytes[0]);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, newSample.uBytes[1]);

  #ifdef USE_DAC
  			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (newSample.uShort)>>4);
  #endif
  		}
  		else if (sample_position < audioFile.AudioSize[audioFileToPlay])
  		{  // repetition 7 more times of the PWM period before new sample, total of times the same value is repeated = 8
  			repetition--;

  			// reload Timer with the actual sample value
  			newSample.uShort = (uint16_t)32768 + pcmSample;

  		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, newSample.uBytes[0]);
  		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, newSample.uBytes[1]);

  #ifdef USE_DAC
  			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (newSample.uShort)>>4);
  #endif
  		}
  		else
  		{  // end of the audio clip
  			/* Disable the TIM3 Interrupt */
  			HAL_NVIC_DisableIRQ(TIM3_IRQn);
  			// stop the timer
  			HAL_TIM_Base_Stop(&htim3);
  		}

  	}
  	return;
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(USER_Btn_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
#define FILTER_WINDOW_SIZE 5

static uint16_t filterBuffer[FILTER_WINDOW_SIZE];
static uint8_t filterIndex = 0;

uint16_t applyMovingAverageFilter(uint16_t newSample)
{
    filterBuffer[filterIndex] = newSample;

    filterIndex = (filterIndex + 1) % FILTER_WINDOW_SIZE;

    uint32_t sum = 0;
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        sum += filterBuffer[i];
    }

    return sum / FILTER_WINDOW_SIZE; // Return the filtered result
}

/* USER CODE END 1 */
