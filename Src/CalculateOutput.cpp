/*
 * calculateOutput.cpp
 *
 *  Created on: 22 May 2017
 *      Author: klemen
 */
#include "calculateOutput.h"

CalculateOutput::CalculateOutput(float* i1, float* i2, float* sp) {
	this->Input1 = i1;
	this->Input2 = i2;
	this->Setpoint1 = sp;
	this->Output1 = 0;
	this->Output2 = 0;
}

void CalculateOutput::calculate() {
	float difference1, difference2;

	difference1 = *Setpoint1 - *Input1;
	if (difference1 > rampPosition)
		difference1 = rampPosition;
	else if (difference1 > 0) {
		//difference1 = difference1;
	} else if (difference1 < 0) {
		//difference1 = -rampPosition;
	} else if (difference1 < -rampPosition) {
		difference1 = -rampPosition;
	}
	Output1 = Output1 + upSpeedRamp * difference1 * 1000 / 5;

	difference2 = *Setpoint1 - *Input2;
	if (difference2 > rampPosition)
		difference2 = rampPosition;
	else if (difference2 > 0) {
		//difference1 = difference1;
	} else if (difference2 < -rampPosition) {
		difference1 = -rampPosition;
	} else if (difference2 < 0) {
		//difference1 = -rampPosition;
	}
	Output2 = Output2 + upSpeedRamp * difference2 * 1000 / 5;

}
