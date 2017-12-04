/*
 * ButtonControl.cpp
 *
 *  Created on: 3 Dec 2017
 *      Author: klemen
 */

#include "ButtonControl.h"
#include "stm32f3xx_hal_tim.h"
#include "limits.h"

extern TIM_HandleTypeDef htim2;

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
}

ButtonControl::~ButtonControl() {
	// TODO Auto-generated destructor stub
}

void ButtonControl::valueReceived(long value) {
	if (timeCapStart == 0 && value < capLimit)
		timeCapStart = HAL_GetTick();

	if (timeCapStart > 0 && value > capLimit) {
		timeCapEnd = HAL_GetTick();

		if ((timeCapEnd - timeCapStart) > 1000 && (timeCapEnd - timeCapStart) < 2000) {
			this->mode = LEFT;
			htim2.Init.Period = 100;
			HAL_TIM_Base_Init(&htim2);
			if (!(htim2.Instance->CR1 & TIM_CR1_CEN)) {
				HAL_TIM_Base_Start_IT(&htim2);
			}
		} else if ((timeCapEnd - timeCapStart) > 2000 && (timeCapEnd - timeCapStart) < 3000) {
			this->mode = RIGHT;
			htim2.Init.Period = 1500;
			HAL_TIM_Base_Init(&htim2);
			if (!(htim2.Instance->CR1 & TIM_CR1_CEN)) {
				HAL_TIM_Base_Start_IT(&htim2);
			}
		}
		else
		{
			this->mode = BOTH;
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			HAL_TIM_Base_Stop_IT(&htim2);
		}
		timeCapStart = 0;
		timeCapEnd = 0;
	}

	if (value < capMin)
		capMin = value;
	if (value > capMax)
		capMax = value;

}

void ButtonControl::button1() {

}

void ButtonControl::button2() {

}
