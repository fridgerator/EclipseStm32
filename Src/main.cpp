/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * Copyright (c) Klemen Živkovič, Župančičeva 34, Grosuplje Slovenia
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
#include <math.h>
#include <stm32_hal_legacy.h>
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
#include <usb_device.h>
#include <usbd_cdc.h>
#include <usbd_def.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>
#include <eeprom_flash.h>

#include "PID_v1.h"

#define SIXSECONDS (6*1000L) // three seconds are 3000 milliseconds
#define THREESECONDS (3*1000L) // three seconds are 3000 milliseconds
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)  256)    /* Size of array containing ADC converted values */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

// for DFU update sample check ( Usb Firmware Update ):
// c:\Users\klemen\STM32Cube\Repository\STM32Cube_FW_F3_V1.7.0\Projects\STM32303C_EVAL\Applications\USB_Device\DFU_Standalone\Src\main.c
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
volatile uint8_t button1On = 0;
volatile uint8_t button2On = 0;

uint32_t button2Count = 0;
uint32_t button1Count = 0;
uint32_t UNBOUNCE_CNT = 3;
uint32_t alreadyResetted = 0;

uint32_t g_ADCValue1 = 0;
uint32_t g_ADCValue3 = 0;

long inPosition1, inPosition2;

float g_ADCValue = 0;

uint32_t enc1;
uint32_t enc2;
uint32_t enc1Old;
uint32_t enc2Old;

uint32_t tick1, tick2;

uint8_t inposition1 = 0;
uint8_t inposition2 = 0;

uint32_t PWM_RES = 1000;
float pwm1 = 0;
float pwm2 = 0;
float dPwm = 0.01;

char buffer[255] = { "" };
char prev_buffer[255] = { "" };

static union { // This Data structure lets
	uint8_t asBytes[24]; // us take the byte array
	float asFloat[6]; // sent from processing and
} // easily convert it to a
floatUnion; // float array

static union { // This Data structure lets
	uint8_t asBytes[2]; // us take the byte array
	uint16_t asInt[1]; // sent from processing and
} // easily convert it to a
byteUnion; // float array

uint8_t received_data[100];
uint32_t received_data_size = 0;
uint32_t receive_total;

bool emergencyStop = false;
uint32_t emergencyStopTimes = 0;
uint32_t last6seconds;
uint32_t last1seconds;
bool prvic = false;
bool slowStopDone = false;
uint8_t previousButton = 0;
bool inPidCorrection = false;

float speedRamp = 0.2; // [increment of PWM / ms]

// 4094 ... 3.3V
// x    ... 1.6V
// x = 1.6*4094/3.3 = 1984

uint16_t g_ADCValue_threshold = 700; // max = 520mV  -> 0.520/3.3*4094=645.5
uint16_t adcMaxMeasureCount = 10;
uint16_t adcMeasureCount = 0;
uint32_t adc3sum, adc1sum;
float adc3Average, adc1Average;

//float aggKp = 110, aggKi = 60, aggKd = 1;
//float aggKp = 70, aggKi = 3, aggKd = 0.2;
//float aggKp = 70, aggKi = 35, aggKd = 1;
//float aggKp = 70, aggKi = 40, aggKd = 1.5;

float aggKp = 70, aggKi = 50, aggKd = 3;

float Setpoint1 = 32768;
float Input1 = 32768;
float Output2;
float Input2 = 32768;
float Output1;

float Output2_adj;
float Output1_adj;
int32_t o2, o1;

PID myPID2 = PID(&Input2, &Output2, &Setpoint1, aggKp, aggKi, aggKd, DIRECT);
PID myPID1 = PID(&Input1, &Output1, &Setpoint1, aggKp, aggKi, aggKd, DIRECT);

int posDelta;

char *ftoa(char *a, double f, int precision);
void buildAndSendBuffer();
void SerialReceive();
void slowStop();
void fastStop();

/* Private variables ---------------------------------------------------------*/

//prescaler 72000000/(PWM frequency*PWM resolution) – 1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*
 Motor Encoders:
 M1 ENCODERS TIM3 PB4 ... ENC3, PB5 ... ENC4
 M2 ENCODERS TIM4 PB7 ... ENC1, PB6 ... ENC2

 Motor Power:
 Motor1 TIM 8 CH2 (PB0), TIM 1 CH2 (PC13)
 Motor2 TIM 8 CH1 (PB14), TIM 1 CH1 (PC13)
 Motor Enable:
 Motor 1 Enable (PB11)
 Motor 2 Enable (PA4)


 */

static int8_t sign(float val1, float val2) {
	if (val1 > val2)
		return 1;
	else if (val1 < val2)
		return -1;

	return 0;
}

// test pwms
void testPWMs() {
	int i = 0;
	int sign = 1;
	while (true) {
		i++;
		if (i % 300 == 0) {
			Output1 = Output1 + sign;
			Output2 = Output2 + sign;
			if (Output1 == 800 || Output1 == -800) {
				sign = -1 * sign;
			}
		}

		// MOTOR 2
		if (Output2 < 0.0) {
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, abs(Output2));
		} else if (Output2 > 0.0) {
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abs(Output2));
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		}

		// MOTOR 1
		if (Output1 < 0.0) {
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, abs(Output1));
		} else if (Output1 > 0.0) {
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, abs(Output1));
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
		}
	}
}

// 310 mV padca na 1kOhm uporu
// I=U/R -> I = 0.310 V/1000 Ohm = 0.000310 A = 0.3 mA
void HAL_Delay(__IO uint32_t Delay) {
	uint32_t tickstart = HAL_GetTick();
	uint32_t now;
	while ((now - tickstart) < Delay) {
		now = HAL_GetTick();
	}
}

uint8_t printUsb(const char* buf) {
	uint16_t Len = strlen(buf);

	if (hUsbDeviceFS.dev_address != 0 && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
		uint8_t UserTxBufferFS[255];
		memcpy(UserTxBufferFS, buf, sizeof(char) * Len);
		USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len);
		return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	}
	return USBD_FAIL;
}
/* USER CODE END 0 */
void myDelay(uint32_t length) {
	for (unsigned long k = 0; k < length; k++) {
	}
}

void resetPwm(uint32_t i) {
//printUsb(buffer);
//INH2 FB11, INH1 PA4
// Reset pulse at INH and IN pin (INH, IN1 and IN2 low)
	HAL_GPIO_WritePin(INH1_GPIO_Port, INH1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INH2_GPIO_Port, INH2_Pin, GPIO_PIN_RESET);
	myDelay(10000);
	//HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
	myDelay(10000);
	//HAL_Delay(100);

	HAL_GPIO_WritePin(INH1_GPIO_Port, INH1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INH2_GPIO_Port, INH2_Pin, GPIO_PIN_SET);

// t_reset = 8us for BTM7752Gb
	sprintf(buffer, "%lu Resetted.\n\r", i);
	printUsb(buffer);
	alreadyResetted = 1;
}

int main(void) {
//	myPID1.SetControllerDirection(REVERSE);
//	myPID2.SetControllerDirection(DIRECT);

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

	/* Initialize interrupts */
	MX_NVIC_Init();

	/* USER CODE BEGIN 2 */
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); //starts PWM on CH1N pin
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); //starts PWM on CH2N pin

	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1); //starts PWM on CH1N pin
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2); //starts PWM on CH2N pin

	HAL_GPIO_WritePin(INH1_GPIO_Port, INH1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INH2_GPIO_Port, INH2_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);

	myPID2.SetMode(AUTOMATIC);
	myPID2.SetOutputLimits(-850.0, 850.0);
	myPID1.SetMode(AUTOMATIC);
	myPID1.SetOutputLimits(-850.0, 850.0);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	writeFlash();
	uint32_t value = readFlash(0);




	uint32_t i = 0;
	uint32_t previous_i = 0;
	uint32_t iOfStandStill = 1;
	int32_t i_emergStop = 0;

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

	/*
	 HAL_ADC_Start_IT(&hadc1);
	 HAL_ADC_Start_IT(&hadc3);
	 */
	/*
	 if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *) aADCxConvertedValues1,
	 ADCCONVERTEDVALUES_BUFFER_SIZE) != HAL_OK) {
	 Error_Handler();
	 }

	 if (HAL_ADCEx_MultiModeStart_DMA(&hadc3, (uint32_t *) aADCxConvertedValues3,
	 ADCCONVERTEDVALUES_BUFFER_SIZE) != HAL_OK) {
	 Error_Handler();
	 }
	 */

	htim3.Instance->CNT = 32768;
	htim4.Instance->CNT = 32768;
	Setpoint1 = 32768;

//printUsb("VOGA TableLifter Init finished.\n\r");

	if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
		printUsb("Error starting encoder 3");
	}
	if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) {
		printUsb("Error starting encoder 4");
	}

	//HAL_Delay(100);
	resetPwm(0);
	//testPWMs();

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		i++;

		g_ADCValue1 = 0;
		g_ADCValue3 = 0;

		bool controlPower1 = true;
		if (controlPower1) {
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 100000) == HAL_OK) {
				if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
					g_ADCValue1 = HAL_ADC_GetValue(&hadc1);
					HAL_ADC_Start(&hadc3);
					if (HAL_ADC_PollForConversion(&hadc3, 100000) == HAL_OK) {
						if ((HAL_ADC_GetState(&hadc3) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
							g_ADCValue3 = HAL_ADC_GetValue(&hadc3);

							/*
							 if (adcMeasureCount < adcMaxMeasureCount) {
							 adc1sum = adc1sum + g_ADCValue1;
							 adc3sum = adc3sum + g_ADCValue3;
							 adcMeasureCount++;
							 } else {
							 adc3Average = adc3sum / adcMeasureCount;
							 adc1Average = adc1sum / adcMeasureCount;
							 adc3sum = 0;
							 adc1sum = 0;
							 adcMeasureCount = 0;
							 */

							g_ADCValue = MAX(g_ADCValue3, g_ADCValue1);
							if (g_ADCValue > g_ADCValue_threshold) {
								emergencyStopTimes++;
								printUsb("Triggered\n");
								char f1[10];
								char f3[10];
								ftoa(f1, g_ADCValue3, 3);
								ftoa(f3, g_ADCValue1, 3);
								sprintf(buffer, "ADC1: %s   ADC2:%s\n", f1, f3);
								printUsb(buffer);

								fastStop();
								resetPwm(i);
								myDelay(5000000);
								Output1_adj = 0;
								Output2_adj = 0;
								if (!emergencyStop) {
									if (Setpoint1 > Input2) {
										Setpoint1 = Input2 - 30;
									} else {
										Setpoint1 = Input2 + 30;
									}
									emergencyStop = true;
									i_emergStop = i;
								}
							}
//							}

						}
					}
				}
			}
		}

		if (HAL_GetTick() - last6seconds >= SIXSECONDS) {
			last6seconds += SIXSECONDS; // remember the time
			if (emergencyStopTimes >= 2) {
				// over current happened 3 times.
				// lets stop
				fastStop();
				myDelay(10000);
				while (true) {
					__WFI();
				}
			}
			emergencyStopTimes = 0;
		}

		if (button1On == 0 && button2On == 0) {
			if (abs(Setpoint1 - Input1) < 2)
				inPosition1++;
			if (abs(Setpoint1 - Input2) < 2)
				inPosition2++;
			if (myPID1.GetMode() == AUTOMATIC && inPosition1 > 100000) {
				myPID1.SetMode(MANUAL);
				Output1 = 0;
			}
			if (myPID2.GetMode() == AUTOMATIC && inPosition2 > 100000) {
				myPID2.SetMode(MANUAL);
				Output2 = 0;
			}

			previousButton = 0;
			prvic = false;
			if (iOfStandStill == 0)
				iOfStandStill = i;
		}

		if (emergencyStop && (i - i_emergStop) > 10000) {
			//reset stepper drivers BTM7752G
			if (abs(Setpoint1 - Input1) < 3 && abs(Setpoint1 - Input2) < 3) {
				if (alreadyResetted != 1) {
					resetPwm(i);
				}
				emergencyStop = false;
				slowStopDone = true;
			}
		}

		Input1 = htim4.Instance->CNT;
		Input2 = htim3.Instance->CNT;

		if (inPidCorrection == false)
			if (Input1 == Setpoint1)
				inPidCorrection = true;

		if (button2On == 1 && !emergencyStop) {
			// UP BUTTON PRESSED
			inPidCorrection = false;
			alreadyResetted = 0;
			if (prvic == false && previousButton != 2) {
				resetPwm(i);
				prvic = true;
				myPID1.SetMode(AUTOMATIC);
				myPID2.SetMode(AUTOMATIC);
				inPosition1 = 0;
				inPosition2 = 0;
			}
			//if (Setpoint1 < Input2 + 40) {
			Setpoint1 = Setpoint1 + 0.008;
			//}
			previousButton = 2;
			iOfStandStill = 0;
		}

		if (button1On == 1 && !emergencyStop) {
			inPidCorrection = false;
			// DOWN BUTTON PRESSED
			alreadyResetted = 0;
			if (prvic == false && previousButton != 1) {
				resetPwm(i);
				prvic = true;
				myPID1.SetMode(AUTOMATIC);
				myPID2.SetMode(AUTOMATIC);
				inPosition1 = 0;
				inPosition2 = 0;
			}
			//if (Setpoint1 > Input2 - 40) {
			Setpoint1 = Setpoint1 - 0.008;
			//}
			previousButton = 1;
			iOfStandStill = 0;
		}
		//HAL_Delay(1);

		myPID1.Compute();
		myPID2.Compute();

		posDelta = static_cast<int>(round(Input2 - Input1));
		if (i % 300 == 0) {  // print reasults over usb every 300ms
			SerialReceive();
			buildAndSendBuffer();
		}

		int8_t sign1 = sign(Output1 + posDelta * 20, Output1_adj);
		Output1_adj = Output1_adj + sign1 * speedRamp;

		int8_t sign2 = sign(Output2 - posDelta * 20, Output2_adj);
		Output2_adj = Output2_adj + sign2 * speedRamp;


		if (Output1 < 0)
			if (Output1_adj < -1000)
				Output1_adj = -1000;
		if (Output1 > 0)
			if (Output1_adj > 1000)
				Output1_adj = 1000;

		if (Output2 < 0)
			if (Output2_adj < -1000)
				Output2_adj = -1000;
		if (Output2 > 0)
			if (Output2_adj > 1000)
				Output2_adj = 1000;

		o2 = (int32_t) Output2_adj;
		o1 = (int32_t) Output1_adj;
		//o2 = (int32_t) Output2;
		//o1 = (int32_t) Output1;

		// output PWM
		// MOTOR 2
		if (o2 < 0.0) {
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, abs(o2));
		} else if (o2 >= 0.0) {
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abs(o2));
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		}

		// MOTOR 1
		if (o1 < 0.0) {
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, abs(o1));
		} else if (o1 >= 0.0) {
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, abs(o1));
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
		}

	}
}

/* USER CODE END 3 */

void slowStop() {
	while (round(abs(o1)) > 2 || round(abs(o2)) > 2) {
		if (o2 < 0.0) {
			o2 = o2 + (round(abs(o2) / 10) + 1);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abs(o2));
		} else if (o2 > 0.0) {
			o2 = o2 - (round(abs(o2) / 10) + 1);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, abs(o2));
		}

		if (o1 < 0.0) {
			o1 = o1 + (round(abs(o1) / 10) + 1);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, abs(o1));
		} else if (o1 > 0.0) {
			o1 = o1 - (round(abs(o1) / 10) + 1);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, abs(o1));
		}
		// pause for 0.1 seconds.
		HAL_Delay(100);
		//myDelay(100000);
	}
	slowStopDone = true;
}

void fastStop() {
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
}

void buildAndSendBuffer() {
	char f1[10];
	char f2[10];
	char f3[10];
	sprintf(buffer, "sp=%s, input=%s, output=%s\n\r", ftoa(f1, Setpoint1, 3), ftoa(f2, Input1, 3), ftoa(f3, Output2, 3));

	strcpy(buffer, "PID ");
	strcat(buffer, ftoa(f1, Setpoint1, 3));
	strcat(buffer, " ");

	strcat(buffer, ftoa(f1, Input2, 3));
	strcat(buffer, " ");

	strcat(buffer, ftoa(f1, Output1, 3));
	strcat(buffer, " ");

	strcat(buffer, ftoa(f1, myPID1.GetKp(), 3));
	strcat(buffer, " ");

	strcat(buffer, ftoa(f1, myPID1.GetKi(), 3));
	strcat(buffer, " ");

	strcat(buffer, ftoa(f1, myPID1.GetKd(), 3));
	strcat(buffer, " ");

	if (myPID1.GetMode() == AUTOMATIC)
		strcat(buffer, "Automatic");
	else
		strcat(buffer, "Manual");

	strcat(buffer, " ");

	if (myPID1.GetDirection() == DIRECT)
		strcat(buffer, "Direct");
	else
		strcat(buffer, "Reverse");

	strcat(buffer, " ");

	char buf1[4] = "";
	itoa(g_ADCValue, buf1, 10);
	strcat(buffer, buf1);

	strcat(buffer, " ");

	char buf2[4] = "";
	itoa(posDelta, buf2, 10);
	strcat(buffer, buf2);

	strcat(buffer, "\n");

	if (strcmp(prev_buffer, buffer) != 0) {
		printUsb(buffer);
		int8_t len = strlen(buffer);
		strncpy(prev_buffer, buffer, len);
	}
}

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive() {
	if (hUsbDeviceFS.dev_address != 0 && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
		uint8_t index = 0;
		uint8_t Auto_Man = -1;
		uint8_t Direct_Reverse = -1;
		if (received_data_size == 28) {
			while (index < received_data_size) {
				if (index == 0)
					Auto_Man = received_data[index];
				else if (index == 1)
					Direct_Reverse = received_data[index];
				else if (index >= 26)
					byteUnion.asBytes[index - 26] = received_data[index];
				else
					floatUnion.asBytes[index - 2] = received_data[index];
				index++;
			}

			if ((Auto_Man == 0 || Auto_Man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1)) {
				Setpoint1 = double(floatUnion.asFloat[0]);

				//Input=double(foo.asFloat[1]);       // * the user has the ability to send the
				//   value of "Input"  in most cases (as
				//   in this one) this is not needed.
				if (Auto_Man == 0)			// * only change the output if we are in
						{ //   manual mode.  otherwise we'll get an
					Output2 = double(floatUnion.asFloat[2]); //   output blip, then the controller will
					Output1 = double(floatUnion.asFloat[2]); //   output blip, then the controller will
				} //   overwrite.
				g_ADCValue_threshold = byteUnion.asInt[0];
				double p, i, d; // * read in and set the controller tunings
				p = double(floatUnion.asFloat[3]); //
				i = double(floatUnion.asFloat[4]); //
				d = double(floatUnion.asFloat[5]); //
				myPID2.SetTunings(p, i, d); //
				myPID1.SetTunings(p, i, d); //

				if (Auto_Man == 0) {
					myPID2.SetMode(MANUAL); // * set the controller mode
					myPID1.SetMode(MANUAL); // * set the controller mode
				} else {
					myPID2.SetMode(AUTOMATIC); //
					myPID1.SetMode(AUTOMATIC); //
				}

				if (Direct_Reverse == 0) {
					myPID2.SetControllerDirection( DIRECT); // * set the controller Direction
					myPID1.SetControllerDirection( DIRECT); // * set the controller Direction
				} else {
					myPID2.SetControllerDirection( REVERSE); //
					myPID1.SetControllerDirection( REVERSE); //
				}
			}
			received_data_size = 0;
		}
	}

}

char *ftoa(char *a, double f, int precision) {
	long p[] = { 0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000 };

	char *ret = a;
	long heiltal = (long) f;
	itoa(heiltal, a, 10);
	while (*a != '\0')
		a++;
	*a++ = '.';
	long desimal = abs((long) ((f - heiltal) * p[precision]));
	itoa(desimal, a, 10);
	return ret;
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
	FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_TIM8 | RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC34;
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
	HAL_SYSTICK_CLKSourceConfig(
	SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
 */
static void MX_NVIC_Init(void) {
	/* EXTI9_5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	/* EXTI15_10_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the ADC multi-mode
	 */
	multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
	multimode.TwoSamplingDelay = ADC_SAMPLETIME_601CYCLES_5;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
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
	hadc3.Init.ContinuousConvMode = ENABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
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
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 10;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC,
	TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC,
	TIM_CHANNEL_2) != HAL_OK) {
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 3;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xFFFF;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
	sConfig.IC1Filter = 5;
	sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
	sConfig.IC2Filter = 5;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 3;
	htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	htim4.Init.Period = 0xFFFF;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
	sConfig.IC1Filter = 5;
	sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
	sConfig.IC2Filter = 5;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* TIM8 init function */
static void MX_TIM8_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim8.Init.Period = 1000;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 10;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC,
	TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC,
	TIM_CHANNEL_2) != HAL_OK) {
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
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
	HAL_GPIO_WritePin(INH1_GPIO_Port, INH1_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(INH2_GPIO_Port, INH2_Pin, GPIO_PIN_SET);

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

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Button1_Pin Button2_Pin */
	GPIO_InitStruct.Pin = Button1_Pin | Button2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
