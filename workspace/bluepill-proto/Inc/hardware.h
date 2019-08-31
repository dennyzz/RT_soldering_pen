/*
 * hardware.h
 *
 */
#ifndef HARDWARE_H_
#define HARDWARE_H_

#include "setup.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;
extern PCD_HandleTypeDef hpcd_USB_FS;
extern WWDG_HandleTypeDef hwwdg;

void Error_Handler(void);

#define MODEL_BLUEPILL_PROTO
#ifdef MODEL_BLUEPILL_PROTO

// #define VCP_TX_Pin GPIO_PIN_2
// #define VCP_TX_GPIO_Port GPIOA
// #define SWDIO_Pin GPIO_PIN_13
// #define SWDIO_GPIO_Port GPIOA
// #define SWCLK_Pin GPIO_PIN_14
// #define SWCLK_GPIO_Port GPIOA
// #define VCP_RX_Pin GPIO_PIN_15
// #define VCP_RX_GPIO_Port GPIOA
// #define LD3_Pin GPIO_PIN_3
// #define LD3_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOC
	

#else
	#warning "no hardware defined"
#endif




#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_H_ */
