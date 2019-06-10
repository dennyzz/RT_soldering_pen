/*
 * Setup.h
 *
 *  Created on: 29Aug.,2017
 *      Author: Ben V. Brown
 */

#ifndef __SETUP_H_
#define __SETUP_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "stm32f0xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;

extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern I2C_HandleTypeDef hi2c1;

extern IWDG_HandleTypeDef hiwdg;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart2;

void Setup_HAL();

uint16_t getADC(uint8_t channel);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);    //Since the hal header file does not define this one

void SystemClock_Config(void);

void MX_ADC_Init(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_I2C1_Init(void);
void MX_IWDG_Init(void);
void MX_SPI1_Init(void);
void MX_TIM3_Init(void);
void MX_USART2_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SETUP_H_ */
