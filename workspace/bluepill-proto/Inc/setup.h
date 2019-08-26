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

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;
extern PCD_HandleTypeDef hpcd_USB_FS;
extern WWDG_HandleTypeDef hwwdg;

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_DMA_Init(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_IWDG_Init(void);
void MX_USB_PCD_Init(void);
void MX_WWDG_Init(void);

void SystemClock_Config(void);

void Setup_HAL();
/* Exported functions prototypes ---------------------------------------------*/
// uint16_t getADC(uint8_t channel);

// void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);    //Since the hal header file does not define this one

#ifdef __cplusplus
}
#endif


#endif /* __SETUP_H_ */


