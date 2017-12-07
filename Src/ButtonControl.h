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
#include "MiniPID.h"
#include "FDC2212.h"

class ButtonControl {
public:
	ButtonControl();
	virtual ~ButtonControl();

	static ButtonControl *getInstance();

	void button1();
	void button2();
	void valueReceived(float value);
	void blinkLed(uint8_t);

	enum mode {
		left = 1, right = 2, both = 4
	} currentMode;

	enum position {
		top = 1, bottom = 2
	} currentPosition;

private:
	void enableLed();
	void disableLed();

	ulong capMin;
	ulong capMax;
	float dCap_dT_trigger = 0;   // touched: 26646406, nottouched: 28121845
	long timeCapStart;
	long timeCapEnd;
	position lastSetPosition;
	long botomPosition;
	long topPosition;

	static ButtonControl* instance;
};

#ifdef __cplusplus
}
#endif

#endif /* BUTTONCONTROL_H_ */
