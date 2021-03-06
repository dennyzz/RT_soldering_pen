/*
 * hardware.h
 *
 */
#ifndef HARDWARE_H_
#define HARDWARE_H_

#include "setup.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

typedef struct Heater{
  uint16_t vin;
  uint16_t vint;
  uint16_t i1;
  uint16_t i2;
  int16_t ttip;
  int16_t tamb;
  int16_t tint;
} Heater_struct;

#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;
extern PCD_HandleTypeDef hpcd_USB_FS;
extern WWDG_HandleTypeDef hwwdg;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void Error_Handler(void);

#define MODEL_HW1_PROTO
#ifdef MODEL_BLUEPILL_PROTO

// #define VCP_TX_Pin GPIO_PIN_2
// #define VCP_TX_GPIO_Port GPIOA
// #define SWDIO_Pin GPIO_PIN_13
// #define SWDIO_GPIO_Port GPIOA
// #define SWCLK_Pin GPIO_PIN_14
// #define SWCLK_GPIO_Port GPIOA
// #define VCP_RX_Pin GPIO_PIN_15
// #define VCP_RX_GPIO_Port GPIOA

#define BTN_UP_Pin GPIO_PIN_13
#define BTN_UP_GPIO_Port GPIOB

#define BTN_DWN_Pin GPIO_PIN_12
#define BTN_DWN_GPIO_Port GPIOB

#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOC
#define VSENSE_CHAN ADC_CHANNEL_0
#define ISENSE1_CHAN ADC_CHANNEL_1
#define ISENSE2_CHAN ADC_CHANNEL_2
#define TEMP_AMB_CHAN ADC_CHANNEL_3
#define TEMP_TIP_CHAN ADC_CHANNEL_4

#elif defined(MODEL_HW1_PROTO)

// #define VCP_TX_Pin GPIO_PIN_2
// #define VCP_TX_GPIO_Port GPIOA
// #define SWDIO_Pin GPIO_PIN_13
// #define SWDIO_GPIO_Port GPIOA
// #define SWCLK_Pin GPIO_PIN_14
// #define SWCLK_GPIO_Port GPIOA
// #define VCP_RX_Pin GPIO_PIN_15
// #define VCP_RX_GPIO_Port GPIOA

#define BTN_UP_Pin GPIO_PIN_9
#define BTN_UP_GPIO_Port GPIOA

#define BTN_DWN_Pin GPIO_PIN_15
#define BTN_DWN_GPIO_Port GPIOC

#define GATE_Pin GPIO_PIN_13
#define GATE_GPIO_Port GPIOC

#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOC

#define ISENSE1_CHAN ADC_CHANNEL_0
#define ISENSE2_CHAN ADC_CHANNEL_1
#define TEMP_AMB_CHAN ADC_CHANNEL_2
#define TEMP_TIP_CHAN ADC_CHANNEL_3
#define VSENSE_CHAN ADC_CHANNEL_4

#else
	#warning "no hardware defined"
#endif




#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_H_ */
