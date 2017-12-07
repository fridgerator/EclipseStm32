	/*
 * FDC2212.h
 *
 *  Created on: 26 Nov 2017
 *      Author: klemen
 */

#ifndef FDC2212_H_
#define FDC2212_H_

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include "limits.h"


#define FDC2214_I2C_ADDRESS   0x2A
// Address is 0x2A (default) or 0x2B (if ADDR is high)

//bitmasks
#define FDC2212_CH0_UNREADCONV 0x08         //denotes unread CH0 reading in STATUS register

#define FDC2212_CH0_DRDY 			0b10000         	 //denotes data ready in STATUS register
#define FDC2212_CH0_AMPL_LOW 	0b10000000         //denotes amplitude low warning in STATUS register
#define FDC2212_CH0_AMPL_HIGH 0b100000000        //denotes amplitude high warning in STATUS register
#define FDC2212_CH0_WCHD_TO   0b1000000000       //denotes watchdog timeout in STATUS register


//registers
#define FDC2212_DEVICE_ID_REGADDR           0x7F
#define FDC2212_MUX_CONFIG_REGADDR          0x1B
#define FDC2212_CONFIG_REGADDR              0x1A
#define FDC2212_SETTLECOUNT_CH0_REGADDR     0x10
#define FDC2212_RCOUNT_CH0_REGADDR          0x08
#define FDC2212_OFFSET_CH0_REGADDR          0x0C
#define FDC2212_CLOCK_DIVIDERS_CH0_REGADDR  0x14
#define FDC2212_STATUS_REGADDR              0x18
#define FDC2212_ERROR_CONFIG              	0x19
#define FDC2212_DATA_CH0_REGADDR            0x00
#define FDC2212_DATA_LSB_CH0_REGADDR        0x01
#define FDC2212_DRIVE_CH0_REGADDR           0x1E
#define FDC2212_RESET_DEV           				0x1C

extern uint8_t printUsb(const char* buf);

class FDC2212 {
public:
		FDC2212();
		FDC2212(I2C_HandleTypeDef i2cHandle);
    bool begin(void);
    ulong getReading();
    bool initialized;
    void shouldRead();
    bool shouldread;
    unsigned long capMax;
    unsigned long capMin;
    float dCap_dT;
    static FDC2212 *getInstance();

private:
    void loadSettings(void);
    void setGain(void);
    double calculateCapacitance(long long fsensor);
    long long calculateFsensor(unsigned long reading);
    void write8FDC(uint16_t address, uint8_t data);
    void write16FDC(uint16_t address, uint16_t data);
    uint16_t read16FDC(uint16_t address);
    uint8_t read8FDC(uint16_t address);
    uint8_t _i2caddr;
    I2C_HandleTypeDef _i2cHandle;
    unsigned long lastReading;
    long lastReadingTick;
    double angle;

    bool isReading;

    static FDC2212* instance;
};

#endif /* FDC2212_H_ */
