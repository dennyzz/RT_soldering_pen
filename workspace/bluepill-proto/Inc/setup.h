/*
 * setup.h
 *
 */

#ifndef __SETUP_H_
#define __SETUP_H_

#include "hardware.h"

#ifdef __cplusplus
extern "C" {
#endif
	
void Setup_HAL();
/* Exported functions prototypes ---------------------------------------------*/
// uint16_t getADC(uint8_t channel);

// void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);    //Since the hal header file does not define this one

#ifdef __cplusplus
}
#endif


#endif /* __SETUP_H_ */


