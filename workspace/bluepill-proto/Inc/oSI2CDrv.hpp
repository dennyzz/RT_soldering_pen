/*
 * oSI2CDrv.hpp
 *
 *  Created on: 14Apr.,2018
 *      Author: Ralim
 */

#ifndef _OSI2CDRV_HPP_
#define _OSI2CDRV_HPP_
#include "hardware.h"
#include "cmsis_os.h"
#include "semphr.h"

class oSI2CDrv {
public:

	static void init(I2C_HandleTypeDef *i2chandle)
	{
		i2c=i2chandle;
		I2CSem=NULL;
	}

	static void oSInit() {
		if(I2CSem == NULL)
		{
			I2CSem = xSemaphoreCreateBinary();
			xSemaphoreGive(I2CSem);
		}
	}

	static void CpltCallback(); //Normal Tx Callback

	static void Mem_Read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
			uint8_t *pData, uint16_t Size);
	static void Mem_Write(uint16_t DevAddress, uint16_t MemAddress,
			uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

	static void Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
	static void I2C_RegWrite(uint8_t address, uint8_t reg, uint8_t data);
	static uint8_t I2C_RegRead(uint8_t address, uint8_t reg);

private:
	static I2C_HandleTypeDef *i2c;
	static void I2C1_ClearBusyFlagErratum();
	static SemaphoreHandle_t I2CSem;
};

#endif /* FRTOSI2C_HPP_ */
