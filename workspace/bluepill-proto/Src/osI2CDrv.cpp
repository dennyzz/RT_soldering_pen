/*
 * oSI2CDrv.cpp
 *
 *  Created on: 14Apr.,2018
 *      Author: Ralim
 */
#include "oSI2CDrv.hpp"
#include "hardware.h"
#include "semphr.h"

#define I2CUSESDMA

I2C_HandleTypeDef* oSI2CDrv::i2c;
SemaphoreHandle_t oSI2CDrv::I2CSem;
void oSI2CDrv::CpltCallback() {
	i2c->State = HAL_I2C_STATE_READY;  // Force state reset (even if tx error)
	if (I2CSem) {
		xSemaphoreGiveFromISR(I2CSem, NULL);
	}
}

void oSI2CDrv::Mem_Read(uint16_t DevAddress, uint16_t MemAddress,
		uint16_t MemAddSize, uint8_t* pData, uint16_t Size) {

	if (I2CSem == NULL) {
		// no RToS, run blocking code
		HAL_I2C_Mem_Read(i2c, DevAddress, MemAddress, MemAddSize, pData, Size,
				5000);
	} else {
		// RToS is active, run threading
		// Get the mutex so we can use the I2C port
		// Wait up to 1 second for the mutex
		if (xSemaphoreTake(I2CSem, (TickType_t)50) == pdTRUE) {
#ifdef I2CUSESDMA
			if (HAL_I2C_Mem_Read(i2c, DevAddress, MemAddress, MemAddSize,
					pData, Size,500) != HAL_OK) {

				I2C1_ClearBusyFlagErratum();
				xSemaphoreGive(I2CSem);
			}
			xSemaphoreGive(I2CSem);
#else

			HAL_I2C_Mem_Read(i2c, DevAddress, MemAddress, MemAddSize, pData, Size,
					5000);
			xSemaphoreGive(I2CSem);
#endif
		} else {
		}
	}

}
void oSI2CDrv::I2C_RegWrite(uint8_t address, uint8_t reg, uint8_t data) {
	Mem_Write(address, reg, I2C_MEMADD_SIZE_8BIT, &data, 1);
}

uint8_t oSI2CDrv::I2C_RegRead(uint8_t add, uint8_t reg) {
	uint8_t tx_data[1];
	Mem_Read(add, reg, I2C_MEMADD_SIZE_8BIT, tx_data, 1);
	return tx_data[0];
}
void oSI2CDrv::Mem_Write(uint16_t DevAddress, uint16_t MemAddress,
		uint16_t MemAddSize, uint8_t* pData, uint16_t Size) {

	if (I2CSem == NULL) {
		// no RToS, run blocking code
		HAL_I2C_Mem_Write(i2c, DevAddress, MemAddress, MemAddSize, pData, Size,
				5000);
	} else {
		// RToS is active, run threading
		// Get the mutex so we can use the I2C port
		// Wait up to 1 second for the mutex
		if (xSemaphoreTake(I2CSem, (TickType_t)50) == pdTRUE) {
#ifdef I2CUSESDMA
			if (HAL_I2C_Mem_Write(i2c, DevAddress, MemAddress, MemAddSize,
					pData, Size,500) != HAL_OK) {

				I2C1_ClearBusyFlagErratum();
				xSemaphoreGive(I2CSem);
			}
			xSemaphoreGive(I2CSem);
#else
			if (HAL_I2C_Mem_Write(i2c, DevAddress, MemAddress, MemAddSize, pData,
							Size, 5000) != HAL_OK) {
			}
			xSemaphoreGive(I2CSem);
#endif
		} else {
		}
	}

}

void oSI2CDrv::Transmit(uint16_t DevAddress, uint8_t* pData, uint16_t Size) {
	if (I2CSem == NULL) {
		// no RToS, run blocking code
		HAL_I2C_Master_Transmit(i2c, DevAddress, pData, Size, 5000);
	} else {
		// RToS is active, run threading
		// Get the mutex so we can use the I2C port
		// Wait up to 1 second for the mutex
		if (xSemaphoreTake(I2CSem, (TickType_t)portTICK_PERIOD_MS) == pdTRUE) {
#ifdef I2CUSESDMA

			if (HAL_I2C_Master_Transmit_DMA(i2c, DevAddress, pData, Size)
					!= HAL_OK) {
				printf("i2c tx ERROR\n");
				I2C1_ClearBusyFlagErratum();
				xSemaphoreGive(I2CSem);

			}
#else
			HAL_I2C_Master_Transmit(i2c, DevAddress, pData, Size, 5000);
			xSemaphoreGive(I2CSem);
#endif

		} else {
		}
	}

}

void oSI2CDrv::I2C1_ClearBusyFlagErratum() {
	printf("ran Clear Busy Flag?\n");
	GPIO_InitTypeDef GPIO_InitStruct;
	int timeout = 100;
	int timeout_cnt = 0;

	// 1. Clear PE bit.
	i2c->Instance->CR1 &= ~(0x0001);
	/**I2C1 GPIO Configuration
	 PB6     ------> I2C1_SCL
	 PB7     ------> I2C1_SDA
	 */
	//  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStruct.Pin = SCL_Pin;
	HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = SDA_Pin;
	HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET);

	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin)) {
		//Move clock to release I2C
		HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET);
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET);

		timeout_cnt++;
		if (timeout_cnt > timeout)
			return;
	}

	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStruct.Pin = SCL_Pin;
	HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SDA_Pin;
	HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET);

	// 13. Set SWRST bit in I2Cx_CR1 register.
	i2c->Instance->CR1 |= 0x8000;

	asm("nop");

	// 14. Clear SWRST bit in I2Cx_CR1 register.
	i2c->Instance->CR1 &= ~0x8000;

	asm("nop");

	// 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
	i2c->Instance->CR1 |= 0x0001;

	// Call initialization function.
	HAL_I2C_Init(i2c);
}

/**
  * @brief This function handles I2C transfer interrupts.
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    oSI2CDrv::CpltCallback();
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    oSI2CDrv::CpltCallback();
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    oSI2CDrv::CpltCallback();
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    oSI2CDrv::CpltCallback();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    oSI2CDrv::CpltCallback();
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    oSI2CDrv::CpltCallback();
}
