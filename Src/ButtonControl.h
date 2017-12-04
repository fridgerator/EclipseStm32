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

#include "stm32f3xx_hal.h"
#define BOTH 0
#define LEFT 1
#define RIGHT 2

class ButtonControl {

public:
	ButtonControl();
	virtual ~ButtonControl();

	static ButtonControl *getInstance();

	void button1();
	void button2();
	void valueReceived(long value);

private:

	ulong capMin;
	ulong capMax;
	long capLimit = 27384125;   // touched: 26646406, nottouched: 28121845
	long timeCapStart;
	long timeCapEnd;
	uint8_t mode;

	static ButtonControl* instance;

};

#ifdef __cplusplus
}
#endif

#endif /* BUTTONCONTROL_H_ */
