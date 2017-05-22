/*
 * calculateOutput.h
 *
 *  Created on: 22 May 2017
 *      Author: klemen
 */

#ifndef CALCULATEOUTPUT_H_
#define CALCULATEOUTPUT_H_

static float upSpeedRamp = 0.001; // [increment of PWM / ms]
static float downSpeedRamp = 0.6; // [increment of PWM / ms]
static int rampPosition = 20;

class CalculateOutput {

private:
	float *Input1, *Input2, *Setpoint1;

public:
	CalculateOutput(float*, float*, float*);
	void calculate();

	float Output1, Output2;
};
#endif /* CALCULATEOUTPUT_H_ */

