/*
 * ButtonControl.cpp
 *
 *  Created on: 3 Dec 2017
 *      Author: klemen
 */

#include "ButtonControl.h"

/* Null, because instance will be initialized on demand. */
ButtonControl* ButtonControl::instance = 0;

ButtonControl* ButtonControl::getInstance() {
	if (!instance) {
		instance = new ButtonControl();
	}
	return instance;
}

ButtonControl::ButtonControl() {
	timeCapEnd = 0;
	timeCapStart = 0;
	capMax = 0;
	capMin = ULONG_MAX;
	currentMode = both;
	currentPosition = position::bottom;
	lastSetPosition = position::top;
	botomPosition = 0;
	topPosition = 0;
	dCap_dT_trigger = 20382000;
	X = 0.3;
	v = 0;
	ledBlinkCount = 0;
	currentBlinkNo = 0;
}

ButtonControl::~ButtonControl() {
	// TODO Auto-generated destructor stub
}

void ButtonControl::valueReceived(float dCap_dT) {
	//char buf1[30];

	//char f1[15];
	v = X * dCap_dT + (1 - X) * v;    // low bandpass filter

	//sprintf(f1, "%s\n", ftoa(f1, v, 3));
	//uint8_t ret = printUsb(f1);
	ITM_SendChar(static_cast<int>(dCap_dT));

	// at least 1 second from last event
	if (HAL_GetTick() - timeCapEnd < 2000)
		return;

	if (dCap_dT == 0)
		return;

	if (timeCapStart == 0 && dCap_dT < dCap_dT_trigger) {
		timeCapStart = HAL_GetTick();
		timeCapEnd = 0;
	}

	if (timeCapStart > 0 && dCap_dT > dCap_dT_trigger)
		timeCapEnd = HAL_GetTick();

	uint32_t deltaTime = (timeCapEnd - timeCapStart);
	if (deltaTime == 0)
		return;

	if (deltaTime > 100 && deltaTime < 1000) {
		this->enableLed(100, 2);
		if (currentPosition == position::bottom) {
			//sprintf(buf1, "ButtonControl: goTOP\n");
			//uint8_t ret = printUsb(buf1);
			if (botomPosition > 0) {
				Setpoint1 = botomPosition;
				Setpoint2 = botomPosition;
			}
			currentPosition = position::top;
		} else if (currentPosition == position::top) {
			//sprintf(buf1, "ButtonControl: goBOTTOM\n");
			//uint8_t ret = printUsb(buf1);
			if (topPosition > 0) {
				Setpoint1 = topPosition;
				Setpoint2 = topPosition;
			}
			currentPosition = position::bottom;
		}
		timeCapStart = 0;
	} else if (deltaTime > 2000 && deltaTime < 3000) {
		if (this->currentMode == ButtonControl::mode::both) {
			this->currentMode = ButtonControl::mode::left;
			//sprintf(buf1, "ButtonControl: left\n");
			//printUsb(buf1);
			this->enableLed();
		} else if (this->currentMode == ButtonControl::mode::left) {
			this->currentMode = ButtonControl::mode::right;
			//sprintf(buf1, "ButtonControl: right\n");
			//printUsb(buf1);
			this->enableLed();
		} else if (this->currentMode == ButtonControl::mode::right) {
			this->currentMode = ButtonControl::mode::both;
			//sprintf(buf1, "ButtonControl: both\n");
			//printUsb(buf1);
			this->disableLed();
			htim3.Instance->CNT = Input1;
			htim4.Instance->CNT = Input1;
			Setpoint1 = Input1;
			Setpoint2 = Input1;
		}
		timeCapStart = 0;
	} else if (timeCapStart > 0 && HAL_GetTick() - timeCapStart > 4000) {
		//setting top position
		if (lastSetPosition == position::top) {
			this->botomPosition = Input1;
			lastSetPosition = position::bottom;
			this->enableLed(80,10);
			//sprintf(buf1, "ButtonControl: BOTOM Position set.\n");
			//printUsb(buf1);
		} else {
			this->topPosition = Input1;
			lastSetPosition = position::top;
			this->enableLed(160,10);
			//sprintf(buf1, "ButtonControl: TOP Position set.\n");
			//printUsb(buf1);
		}
		timeCapStart = 0;
	}
}

void ButtonControl::enableLed() {
	htim2.Init.Period = ButtonControl::currentMode * 500;
	HAL_TIM_Base_Init(&htim2);
	if (!(htim2.Instance->CR1 & TIM_CR1_CEN)) {
		HAL_TIM_Base_Start_IT(&htim2);
	}
}

void ButtonControl::enableLed(uint32_t period) {
	htim2.Init.Period = period;
	HAL_TIM_Base_Init(&htim2);
	if (!(htim2.Instance->CR1 & TIM_CR1_CEN)) {
		HAL_TIM_Base_Start_IT(&htim2);
	}
}

void ButtonControl::enableLed(uint32_t period, uint8_t blinkCount) {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	htim2.Init.Period = period;
	ledBlinkCount = blinkCount;
	currentBlinkNo = 0;
	HAL_TIM_Base_Init(&htim2);
	if (!(htim2.Instance->CR1 & TIM_CR1_CEN)) {
		HAL_TIM_Base_Start_IT(&htim2);
	}
}

void ButtonControl::disableLed() {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Stop_IT(&htim2);
}

void ButtonControl::button1() {

}

void ButtonControl::button2() {

}
