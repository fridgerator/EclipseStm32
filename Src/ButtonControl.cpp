/*
 * ButtonControl.cpp
 *
 *  Created on: 3 Dec 2017
 *      Author: klemen
 */

#include "ButtonControl.h"


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern float Input1;
extern float Input2;
extern float Setpoint1;
extern float Setpoint2;
extern FDC2212 capSense;

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
}

ButtonControl::~ButtonControl() {
	// TODO Auto-generated destructor stub
}

void ButtonControl::blinkLed(uint8_t noTimes) {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	for (int i = 0; i < noTimes; i++) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(200);
	}
}

void ButtonControl::valueReceived(float dCap_dT) {
	if (timeCapStart == 0 && dCap_dT < dCap_dT_trigger)
		timeCapStart = HAL_GetTick();

	if (timeCapStart > 0 && dCap_dT > dCap_dT_trigger)
		timeCapEnd = HAL_GetTick();

	if ((timeCapEnd - timeCapStart) < 1000) {
		if (currentPosition == position::bottom && botomPosition > 0) {
			Setpoint1 = botomPosition;
			Setpoint2 = botomPosition;
			currentPosition = position::top;
		} else if (topPosition > 0) {
			Setpoint1 = topPosition;
			Setpoint2 = topPosition;
			currentPosition = position::bottom;
		}
		timeCapStart = 0;
		timeCapEnd = 0;
	} else if ((timeCapEnd - timeCapStart) > 1000 && (timeCapEnd - timeCapStart) < 2000) {
		if (this->currentMode == ButtonControl::mode::both) {
			this->currentMode = ButtonControl::mode::left;
			this->enableLed();
		} else if (this->currentMode == ButtonControl::mode::left) {
			this->currentMode = ButtonControl::mode::right;
			this->enableLed();
		} else if (this->currentMode == ButtonControl::mode::right) {
			this->currentMode = ButtonControl::mode::both;
			this->disableLed();
			htim3.Instance->CNT = Input1;
			htim4.Instance->CNT = Input1;
			Setpoint1 = Input1;
			Setpoint2 = Input1;
		}
		timeCapStart = 0;
		timeCapEnd = 0;
	} else if (HAL_GetTick() - timeCapStart  > 4000) {
		//setting top position
		if (lastSetPosition == position::top) {
			this->botomPosition = Input1;
			lastSetPosition = position::bottom;
			this->blinkLed(2);
		} else {
			this->topPosition = Input1;
			lastSetPosition = position::top;
			this->blinkLed(4);
		}
		timeCapStart = 0;
		timeCapEnd = 0;
	}
}

void ButtonControl::enableLed() {
	htim2.Init.Period = ButtonControl::currentMode * 500;
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
