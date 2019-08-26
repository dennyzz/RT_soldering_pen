/*
 * setup.h
 *
 */

#ifndef __SETUP_H_
#define __SETUP_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "hardware.h"
#include "stm32f0xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern ADC_HandleTypeDef hadc;

extern IWDG_HandleTypeDef hiwdg;

void Setup_HAL();
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
// uint16_t getADC(uint8_t channel);

// void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);    //Since the hal header file does not define this one

#ifdef __cplusplus
}
#endif

#endif /* __SETUP_H_ */
