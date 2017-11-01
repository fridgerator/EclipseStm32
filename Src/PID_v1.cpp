/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "main.h"
#include "PID_v1.h"
#include "stm32f303xc.h"
#include "stm32f3xx_hal.h"
#include <limits>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID1::PID1(float* Input, float* Output, float* Setpoint, float Kp, float Ki, float Kd, int ControllerDirection) {

	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	inAuto = false;

	SampleTime = 1;											//default Controller Sample Time is 0.1 seconds
	PID1::SetOutputLimits(0, 255);				//default output limit corresponds to pwm limits
	this->SetAccelerationLimits(std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
	lastEncoderPosition = *myInput;
	timeOfPrevEnc = HAL_GetTick();

	PID1::SetControllerDirection(ControllerDirection);
	PID1::SetTunings(Kp, Ki, Kd);

	lastTime = HAL_GetTick() - SampleTime;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID1::Compute() {
	if (!inAuto)
		return false;
	unsigned long now = HAL_GetTick();
	unsigned long timeChange = (now - this->lastTime);
	if (timeChange >= this->SampleTime) {
		/*Compute all the working error variables*/
		//sprintf(buffer, "%lu timeChange: %lu\n\r", now, timeChange);
		//printUsb(buffer);
		float input = *myInput;
		float error = *mySetpoint - input;
		ITerm += (ki * error);
		if (ITerm > outMax)
			ITerm = outMax;
		else if (ITerm < outMin)
			ITerm = outMin;
		float dInput = (input - lastInput);

		/*Compute PID Output*/
		float output = kp * error + ITerm - kd * dInput;


		//lets calculate speed change
		/*
		if (input != lastEncoderPosition) {
			this->speed = (input - lastEncoderPosition) / (now - timeOfPrevEnc);
			this->acceleration = (this->speed - this->prevSpeed) / timeChange;

			if (this->acceleration > this->maxAccel)
				output = prevOutput;
			if (this->acceleration < this->minAccel)
				output = prevOutput;

			if (output > outMax)
				output = outMax;
			else if (output < outMin)
				output = outMin;

			this->lastEncoderPosition = input;
			this->timeOfPrevEnc = now;
			this->prevSpeed = speed;
		} else {

		}
		*/
		*myOutput = output;

		/*Remember some variables for next time*/
		this->lastInput = input;
		this->lastTime = now;
		this->prevOutput = output;

		return true;

	} else
		return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID1::SetTunings(float Kp, float Ki, float Kd) {
	if (Kp < 0 || Ki < 0 || Kd < 0)
		return;

	dispKp = Kp;
	dispKi = Ki;
	dispKd = Kd;

	float SampleTimeInSec = ((float) SampleTime) / 1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;

	if (controllerDirection == REVERSE) {
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID1::SetSampleTime(int NewSampleTime) {
	if (NewSampleTime > 0) {
		float ratio = (float) NewSampleTime / (float) SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned long) NewSampleTime;
	}
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID1::SetOutputLimits(float Min, float Max) {
	if (Min >= Max)
		return;
	outMin = Min;
	outMax = Max;

	if (inAuto) {
		if (*myOutput > outMax)
			*myOutput = outMax;
		else if (*myOutput < outMin)
			*myOutput = outMin;

		if (ITerm > outMax)
			ITerm = outMax;
		else if (ITerm < outMin)
			ITerm = outMin;
	}
}

void PID1::SetAccelerationLimits(float minAccel, float maxAccel) {
	this->minAccel = minAccel;
	this->maxAccel = maxAccel;
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID1::SetMode(int Mode) {
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto == !inAuto) { /*we just went from manual to auto*/
		PID1::Initialize();
	}
	inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID1::Initialize() {
	ITerm = *myOutput;
	lastInput = *myInput;
	if (ITerm > outMax)
		ITerm = outMax;
	else if (ITerm < outMin)
		ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID1::SetControllerDirection(int Direction) {
	if (inAuto && Direction != controllerDirection) {
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
	controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID1::GetKp() {
	return dispKp;
}
float PID1::GetKi() {
	return dispKi;
}
float PID1::GetKd() {
	return dispKd;
}
int PID1::GetMode() {
	return inAuto ? AUTOMATIC : MANUAL;
}
int PID1::GetDirection() {
	return controllerDirection;
}

void DWT_Init(void) {
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
}

uint32_t DWT_Get(void) {
	return DWT->CYCCNT;
}

__inline
uint8_t DWT_Compare(int32_t tp) {
	return (((int32_t) DWT_Get() - tp) < 0);
}

void DWT_Delay(uint32_t us) // microseconds
		{
	int32_t tp = DWT_Get() + us * (SystemCoreClock / 1000000);
	while (DWT_Compare(tp))
		;
}
