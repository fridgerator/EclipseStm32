/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/

#include <main.h>
#include <stm32f303xc.h>
#include <stm32f3xx.h>
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_adc.h>
#include <stm32f3xx_hal_adc_ex.h>
#include <stm32f3xx_hal_cortex.h>
#include <stm32f3xx_hal_def.h>
#include <stm32f3xx_hal_flash.h>
#include <stm32f3xx_hal_gpio.h>
#include <stm32f3xx_hal_rcc.h>
#include <stm32f3xx_hal_rcc_ex.h>
#include <stm32f3xx_hal_tim.h>
#include <stm32f3xx_hal_tim_ex.h>
#include <sys/_stdint.h>
#include <usb_device.h>
#include <usbd_cdc.h>
#include <usbd_def.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

int button1On = 0;
int button2On = 0;
int button2Count = 0;
int button1Count = 0;
int UNBOUNCE_CNT = 10;
int needsReset = 0;

int32_t enc1;
int32_t enc2;
int32_t enc1Old;
int32_t enc2Old;

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

uint8_t printUsb(char* buf) {
	USBD_CDC_HandleTypeDef *hcdc =
			(USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
	uint8_t result = USBD_OK;

	uint16_t Len = strlen(buf);
	/* USER CODE BEGIN 7 */
	char UserTxBufferFS[255];
	memcpy(UserTxBufferFS, buf, sizeof(char) * Len);
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len);

	do {
		result |= USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	} while ((result != USBD_OK) && (result != USBD_FAIL));

	if ((Len % 64 == 0) && (result == USBD_OK)) {
		while (hcdc->TxState)
			;
		USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buf, 0);
		result |= USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	}

	return result;
}

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM8_Init();
	MX_ADC3_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();

	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	uint32_t g_ADCValue1 = 0;
	uint32_t g_ADCValue3 = 0;

	uint32_t GPIO_Pin_Button1 = GPIO_PIN_9;   //PA9
	uint32_t GPIO_Pin_Button2 = GPIO_PIN_10;  //PA10

	uint32_t i = 0;
	HAL_Delay(6000);

	char buffer[255];
	printUsb("VOGA TableLifter Init finished.\n\r");

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		i++;
		if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
			g_ADCValue1 = HAL_ADC_GetValue(&hadc1);
		}
		if (HAL_ADC_PollForConversion(&hadc3, 1000) == HAL_OK) {
			g_ADCValue3 = HAL_ADC_GetValue(&hadc3);
		}

		char buffer[255];
//		sprintf(buffer, "Analog1=%u, Analog3=%u \n\r", g_ADCValue1,
//				g_ADCValue3);
		HAL_Delay(1);
		sprintf(buffer, "tick %u\n\r", i);
		printUsb(buffer);

		button1On = 1;
		button2On = 1;

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin_Button1) == GPIO_PIN_RESET) {
			if (button1Count < UNBOUNCE_CNT) {
				sprintf(buffer, "unbounce %u\n\r", button1Count);
				printUsb(buffer);
				button1Count++;
			} else {
				printUsb("Button UP is set!\n\r");
				button1On = 1;
				button1Count = 0;
				needsReset = 1;
			}
		} else {
			button1Count = 0;
			button1On = 0;
		}

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin_Button2) == GPIO_PIN_RESET) {
			if (button2Count < UNBOUNCE_CNT)
				button2Count++;
			else {
				button2On = 1;
				button2Count = 0;
				needsReset = 1;
				printUsb("Button DOWN is set!\n\r");
			}
		} else {
			button2Count = 0;
			button2On = 0;
		}
		if (button1On == 0 && button2On == 0) {
			//reset stepper drivers BTM7752G
			if (needsReset == 1) {
				//INH2 FB11, INH1 PA4
				// Reset pulse at INH and IN pin (INH, IN1 and IN2 low)
				HAL_GPIO_WritePin(INH1_GPIO_Port, INH1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(INH2_GPIO_Port, INH2_Pin, GPIO_PIN_SET);

				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);

				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

				// t_reset = 8us for BTM7752Gb
				HAL_Delay(5);
				needsReset = 0;
			}
		} else if (button1On == 1) {
			uint8_t pwm = 50;
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);

			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
		} else if (button2On == 1) {
			uint8_t pwm = 50;
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);

			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		}

		// read encoder counter
		// uint32_t enc1 = htim4.Instance->CNT;
		enc1 = __HAL_TIM_GetCounter(&htim3);
		enc2 = __HAL_TIM_GetCounter(&htim4);

		if (enc1Old != enc1) {
			sprintf(buffer, "Encoder1 = %d\n", enc1);
			printUsb(buffer);
			enc1Old = enc1;
		}
		if (enc2Old != enc2) {
			sprintf(buffer, "Encoder2 = %d\n", enc2);
			printUsb(buffer);
			enc2Old = enc2;
		}

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_TIM1
			| RCC_PERIPHCLK_TIM8 | RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC34;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
	PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_MultiModeTypeDef multimode;
	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the ADC multi-mode
	 */
	multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
	multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* ADC3 init function */
static void MX_ADC3_Init(void) {

	ADC_MultiModeTypeDef multimode;
	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the ADC multi-mode
	 */
	multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
	multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_SlaveConfigTypeDef sSlaveConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

}

/* TIM8 init function */
static void MX_TIM8_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 0;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim8);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(INH1_GPIO_Port, INH1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(INH2_GPIO_Port, INH2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : INH1_Pin */
	GPIO_InitStruct.Pin = INH1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(INH1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : INH2_Pin */
	GPIO_InitStruct.Pin = INH2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(INH2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BUTTON1_Pin BUTTON2_Pin */
	GPIO_InitStruct.Pin = BUTTON1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BUTTON1_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = BUTTON2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BUTTON2_GPIO_Port, &GPIO_InitStruct);



}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {

	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
