/*
 * ButtonControl.h
 *
 *  Created on: 3 Dec 2017
 *      Author: klemen
 */

#ifndef BUTTONCONTROL_H_
#define BUTTONCONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_tim.h"
#include "limits.h"
#include "math.h"
#include "usb_device.h"

#include "FDC2212.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern float Input1;
extern float Input2;
extern float Setpoint1;
extern float Setpoint2;
extern uint8_t printUsb(const char* buf);
extern char *ftoa(char *a, double f, int precision);

class ButtonControl {

public:
	ButtonControl();
	virtual ~ButtonControl();

	static ButtonControl *getInstance();

	void button1();
	void button2();
	void valueReceived(float value);

	enum mode {
		left = 1, right = 2, both = 4
	} currentMode;

	enum position {
		top = 1, bottom = 2
	} currentPosition;

	uint8_t ledBlinkCount;
	uint8_t currentBlinkNo;

private:
	void enableLed();
	void enableLed(uint32_t);
	void enableLed(uint32_t, uint8_t);
	void disableLed();

	ulong capMin;
	ulong capMax;
	float maDiffTrigger = 0;   // touched: 26646406, nottouched: 28121845
	uint32_t timeCapStart;
	uint32_t timeCapEnd;
	position lastSetPosition;
	long botomPosition;
	long topPosition;
	double v;
	double v1;
	double X;
	double X1;

	static ButtonControl* instance;
	uint32_t count;
	int32_t maDiff;

};

#ifdef __cplusplus
}
#endif

#endif /* BUTTONCONTROL_H_ */
