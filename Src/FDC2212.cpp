/*
 * FDC2212.cpp
 *
 *  Created on: 26 Nov 2017
 *      Author: klemen
 */

#include "FDC2212.h"

FDC2212::FDC2212() {
	_i2caddr = (FDC2214_I2C_ADDRESS << 1);
}

FDC2212::FDC2212(I2C_HandleTypeDef i2cHandle) {
	_i2caddr = (FDC2214_I2C_ADDRESS << 1);
	_i2cHandle = i2cHandle;
}

/**************************************************************************/
/*!
 @brief  Setups the HW
 */
/**************************************************************************/
bool FDC2212::begin(void) {
	//Wire.begin();

	uint8_t aRxBuffer[2];
	uint16_t REG_CHIP_MEM_ADDR = 0x7e;
	if (HAL_I2C_Mem_Read(&_i2cHandle, _i2caddr, REG_CHIP_MEM_ADDR, I2C_MEMADD_SIZE_8BIT, aRxBuffer, 2, 10000) != HAL_OK) {
		if (HAL_I2C_GetError(&_i2cHandle) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}
	uint16_t manufacturer = aRxBuffer[0] << 8;
	manufacturer |= aRxBuffer[1];   // returns: manufacturer = 0x5449;

	int devId = read16FDC(FDC2212_DEVICE_ID_REGADDR);
	if (devId != 0x3055) {
		//two valid device ids for FDC2214 0x3054 and 0x3055
		return false;
	}

	loadSettings();
	setGain();

	return true;
}

/**************************************************************************/
/*!
 @brief  Initalizes FDC2214
 */
/**************************************************************************/
void FDC2212::loadSettings(void) {

	//reset device
	write16FDC(FDC2212_RESET_DEV, 0b1000000000000000);  //reset device

	//0b00  00 0001                      RESERVED
	//0b0   0 00 0001                    Normal current drive (auto scan is enabled)
	//0b0   0 0 00 0001                  INTB_DIS - Do NOT disable interrupt pin
	//0b0   0 0 0 00 0001                RESERVED
	//0b0   0 0 0 0 00 0001              Use internal oscilator
	//0b1   1 0 0 0 0 00 0001            RESERVED
	//0b0   0 1 0 0 0 0 00 0001          full current mode
	//0b1   1 0 1 0 0 0 0 00 0001        RESERVED
	//0b0   0 1 0 1 0 0 0 0 00 0001      device is active - no sleep
	//0b00 00 0 1 0 1 0 0 0 0 00 0001    Contineous reads on CH0 CONFIG.ACTIVE_CHAN

	//FDC2212_CONFIG_REGADDR              0x1A
	write16FDC(FDC2212_CONFIG_REGADDR, 0b0001010000000001);  //set config
	//write16FDC(FDC2212_CONFIG_REGADDR, 0x1E81);  //set config
//    write16FDC(FDC2214_CONFIG_REGADDR, 0x201);  //set config

	//settle count maximized, slow application
	write16FDC(FDC2212_SETTLECOUNT_CH0_REGADDR, 0x64);

	//rcount maximized for highest accuracy
	write16FDC(FDC2212_RCOUNT_CH0_REGADDR, 0xFFFF);

	//no offset
	write16FDC(FDC2212_OFFSET_CH0_REGADDR, 0x0000);

	//set clock divider
	//0b00 0000 0001                                    Set clock div to 1
	//0b00 [00 0000 0001]                               RESERVED
	//0b01 [10] [00 0000 0001]                          divide by 2
	//0b01 0b100000000001
	//write16FDC(FDC2212_CLOCK_DIVIDERS_CH0_REGADDR, 0x1001);
	write16FDC(FDC2212_CLOCK_DIVIDERS_CH0_REGADDR, 0b100000000001);

	//set drive register
	//write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0xF800);
	//write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0b0111100000000000);
	//write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0b0010100000000000);
	write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0b0111100000000000);

	// 0b 										101 					Deglitch 10MHz, oscilation is ~4MHz by default
	// 0b 		 	 	 0001000001 101           RESERVED
	// 0b00 		00 0001000001 101        		Ch0, Ch1 Sequence unused only read on CH0 using CONFIG.ACTIVE_CHAN
	// 0b0    0 00 0001000001 101        		Continuous conversion on the single channel selected by CONFIG.ACTIVE_CHAN register field.

	write16FDC(FDC2212_MUX_CONFIG_REGADDR, 0b0000001000001101);  //set mux config for channels

	// set warnings
	// 0b1    											1										report data ready flag
	// 0b0000 								 0000 1										RESERVED
	// 0b1 									 1 0000 1										watchdog timeout error enable
	// 0b00000 				 00000 1 0000 1										RESERVED
	// 0b1 					 1 00000 1 0000 1										enable amplitude low warning
	// 0b1 				 1 1 00000 1 0000 1										enable amplitude high warning
	// 0b0 			 0 1 1 00000 1 0000 1										disable watchdog timeout error
	// 0b00 	00 0 1 1 00000 1 0000 1										RESERVED

	write16FDC(FDC2212_ERROR_CONFIG, 0b0001100000100001);  //set error config

}

///**************************************************************************/
///*!
//    @brief  Given a reading calculates the sensor frequency
//*/
///**************************************************************************/
//long long FDC2212::calculateFsensor(unsigned long reading){
////    Serial.println("reading: "+ String(reading));
//    //fsensor = (CH_FIN_SEL * fref * data) / 2 ^ 28
//    //should be mega hz so can truncate to long long
//    Serial.println("FDC reading: " + String(reading));
//    unsigned long long temp;
//    temp = 1 * 40000000 * reading;
//    temp = temp / (2^28);
////    Serial.println("frequency: " + String((long)temp));
//    return temp;
//}

///**************************************************************************/
///*!
//    @brief  Given sensor frequency calculates capacitance
//*/
///**************************************************************************/
//double FDC2212::calculateCapacitance(long long fsensor){
//    //differential configuration
//    //c sensor = 1                            - (Cboard + Cparacitic)
//    //             / (L * (2*pi * fsensor)^2)
//
//    double pi = 3.14159265359;
//    double L = 18; //uH
//    double Cboard = 33; //pf
//    double Cparacitic = 3; //pf
//
//    double temp = 2 * pi * fsensor;
//    temp = temp * temp;
//
//    temp = temp / 1000000; //uH
//    temp *= L;
//
////    Serial.println("capacitance: " + String(temp));
//    return temp;
//
//}

/**************************************************************************/
/*!
 @brief  Takes a reading and calculates capacitance from i
 */
/**************************************************************************/
unsigned long FDC2212::getReading() {
	int timeout = 100;
	unsigned long reading = 0;
	long long fsensor = 0;
	int status = read16FDC(FDC2212_STATUS_REGADDR);

	if (status & FDC2212_CH0_DRDY)
		asm("nop");
	if (status & FDC2212_CH0_AMPL_LOW)
		asm("nop");
	if (status & FDC2212_CH0_AMPL_HIGH)
		asm("nop");
	if (status & FDC2212_CH0_WCHD_TO)
		asm("nop");

	while (timeout && !(status & FDC2212_CH0_UNREADCONV)) {
//        Serial.println("status: " + String(status));
		status = read16FDC(FDC2212_STATUS_REGADDR);



		timeout--;
	}
	if (timeout == 100) {
		//could be stale grab another
		//read the 28 bit result
		reading = read16FDC(FDC2212_DATA_CH0_REGADDR) << 16;
		reading |= read16FDC(FDC2212_DATA_LSB_CH0_REGADDR);
		while (timeout && !(status & FDC2212_CH0_UNREADCONV)) {
//        Serial.println("status: " + String(status));
			status = read16FDC(FDC2212_STATUS_REGADDR);
			timeout--;
		}
	}
	if (timeout) {
		//read the 28 bit result
		reading = read16FDC(FDC2212_DATA_CH0_REGADDR) << 16;
		reading |= read16FDC(FDC2212_DATA_LSB_CH0_REGADDR);
		return reading;
	} else {
		//error not reading
		//printUsb("error reading fdc");
		return 0;
	}
}

///**************************************************************************/
///*!
//    @brief  Takes a reading and calculates capacitance from it
//*/
///**************************************************************************/
//double FDC2212::readCapacitance() {
//    int timeout = 100;
//    unsigned long reading = 0;
//    long long fsensor = 0;
//    int status = read16FDC(FDC2214_STATUS_REGADDR);
//    while (timeout && !(status & FDC2214_CH0_UNREADCONV)) {
////        Serial.println("status: " + String(status));
//        status = read16FDC(FDC2214_STATUS_REGADDR);
//        timeout--;
//    }
//    if (timeout) {
//        //read the 28 bit result
//        reading = read16FDC(FDC2214_DATA_CH0_REGADDR) << 16;
//        reading |= read16FDC(FDC2214_DATA_LSB_CH0_REGADDR);
//        fsensor = calculateFsensor(reading);
//        return calculateCapacitance(fsensor);
//    } else {
//        //error not reading
//        Serial.println("error reading fdc");
//        return 0;
//    }
//}

/**************************************************************************/
/*!
 @brief  Scans various gain settings until the amplitude flag is cleared.
 WARNING: Changing the gain setting will generally have an impact on the
 reading.
 */
/**************************************************************************/
void FDC2212::setGain(void) {
	//todo
}
/**************************************************************************/
/*!
 @brief  I2C low level interfacing
 */
/**************************************************************************/

// Read 1 byte from the VL6180X at 'address'
uint8_t FDC2212::read8FDC(uint16_t address) {
	uint8_t data;

	/*
	 Wire.beginTransmission(_i2caddr);
	 Wire.write(address >> 8);
	 Wire.write(address);

	 Wire.endTransmission();

	 Wire.requestFrom(_i2caddr, (uint8_t) 1);
	 uint8_t r = Wire.read();
	 */
	uint8_t aRxBuffer[2];
	if (HAL_I2C_Mem_Read(&_i2cHandle, _i2caddr, address, I2C_MEMADD_SIZE_8BIT, aRxBuffer, 1, 100) != HAL_OK) {
		if (HAL_I2C_GetError(&_i2cHandle) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}
	data = aRxBuffer[0];

#if defined(I2C_DEBUG)
	Serial.print("\t$"); Serial.print(address, HEX); Serial.print(": 0x"); Serial.println(r, HEX);
#endif

	return data;
}

// Read 2 byte from the VL6180X at 'address'
uint16_t FDC2212::read16FDC(uint16_t address) {
	uint16_t data;
	/*
	 Wire.beginTransmission(_i2caddr);
	 //    Wire.write(address >> 8);
	 Wire.write(address);
	 Wire.endTransmission();

	 Wire.requestFrom(_i2caddr, (uint8_t) 2);
	 while (!Wire.available());
	 data = Wire.read();
	 data <<= 8;
	 while (!Wire.available());
	 data |= Wire.read();
	 return data;
	 */
	uint8_t aRxBuffer[2];
	if (HAL_I2C_Mem_Read(&_i2cHandle, _i2caddr, address, I2C_MEMADD_SIZE_8BIT, aRxBuffer, 2, 100) != HAL_OK) {
		if (HAL_I2C_GetError(&_i2cHandle) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}
	data = aRxBuffer[0] << 8;
	data |= aRxBuffer[1];

	return data;
}

// write 1 byte
void FDC2212::write8FDC(uint16_t address, uint8_t data) {
	/*
	 Wire.beginTransmission(_i2caddr);
	 Wire.write(address >> 8);
	 Wire.write(address);
	 Wire.write(data);
	 Wire.endTransmission();
	 */

	if (HAL_I2C_Mem_Write(&_i2cHandle, _i2caddr, address, sizeof(uint8_t), &data, 1, 100) != HAL_OK) {
		if (HAL_I2C_GetError(&_i2cHandle) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}

#if defined(I2C_DEBUG)
	Serial.print("\t$"); Serial.print(address, HEX); Serial.print(" = 0x"); Serial.println(data, HEX);
#endif
}

//// write 2 bytes
//void FDC2212::write16FDC(uint16_t address, uint16_t data) {
//    Wire.beginTransmission(_i2caddr);
////    Wire.write(address >> 8);
//    Wire.write(address & 0xFF);
//    Wire.write(data >> 8);
//    Wire.write(data);
//    Wire.endTransmission();
//}
void FDC2212::write16FDC(uint16_t REG_CHIP_MEM_ADDR, uint16_t value) {
	uint8_t aTxBuffer[2] = { (value >> 8), (value & 0xFF), };
	if (HAL_I2C_Mem_Write(&_i2cHandle, _i2caddr, REG_CHIP_MEM_ADDR, I2C_MEMADD_SIZE_8BIT, aTxBuffer, 2, 10000) != HAL_OK) {
		if (HAL_I2C_GetError(&_i2cHandle) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}

}
