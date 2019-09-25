/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
// #include "main.hpp"
#include "cmsis_os.h"
#include "setup.h"
#include "hardware.h"
#include "oSI2CDrv.hpp"
#include "display.hpp"
#include "screen/screen.hpp"
#include "screen/GUI.hpp"
#include "font.hpp"

/* Private includes ----------------------------------------------------------*/
#define DISP_WIDTH  (128)
#define DISP_HEIGHT (32)
typedef uint32_t HEIGHT_TYPE;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId GUITaskHandle;
osThreadId PIDTaskHandle;
osThreadId IMUTaskHandle;

Heater_struct heater;

/* Private function prototypes -----------------------------------------------*/
void StartDefaultTask(void const * argument);
void startGUITask(void const *argument);
void startPIDTask(void const *argument);
void startIMUTask(void const *argument);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  /* Initialize all configured peripherals */
  Setup_HAL();

  printf("hello world!\n");

  /* add mutexes, ... */

  /* add semaphores, ... */

  /* start timers, add new ones, ... */

  /* add queues, ... */

  /* Create the thread(s) */
  // osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 64 / 4);
  // defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(GUITask, startGUITask, osPriorityNormal, 0, 2048 / 4);
  GUITaskHandle = osThreadCreate(osThread(GUITask), NULL);

  osThreadDef(PIDTask, startPIDTask, osPriorityRealtime, 0, 1024 / 4);
  PIDTaskHandle = osThreadCreate(osThread(PIDTask), NULL);

  osThreadDef(IMUTask, startIMUTask, osPriorityNormal, 0, 512 / 4);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  volatile uint32_t size = xPortGetFreeHeapSize();
  printf("Heap size:%d bytes\n", (int)size);
  // breakpoint if task fails to create
  if (IMUTaskHandle == 0)
    asm("bkpt");

  /* Start scheduler */
  printf("starting scheduler!\n");
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  while (1)
  {

  }
}

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
void StartDefaultTask(void const * argument)
{
  printf("Default Task Started\n");
  for(;;)
  {
    osDelay(100);
  }
}
/**
  * @brief  Function implementing the startGUITask thread.
  * @param  argument: Not used 
  * @retval None
  */

void startGUITask(void const *argument)
{
  printf("GUI Task Initializing\n");
  screen::GUI gui(heater);
  osDelay(500); // delay for hw startup fix with reset pin later
  gui.init();
  printf("GUI Task Started\n");
  uint32_t last_tick = osKernelSysTick();
  uint32_t task_delay;
  for(;;)
  {
    osDelayUntil (&last_tick, 25);
    task_delay = osKernelSysTick() - last_tick;
    last_tick = osKernelSysTick();
    gui.process(task_delay);
    gui.draw();
  }
}
/**
  * @brief  Function implementing the startPIDTask thread.
  * @param  argument: Not used 
  * @retval None
  */
SemaphoreHandle_t ADCSem;
uint16_t adcbuf[96];
void startPIDTask(void const *argument)
{
  uint32_t starttick;
  uint32_t endtick;
  ADCSem = xSemaphoreCreateBinary();
  printf("PID Task Initializing\n");
  HAL_ADCEx_Calibration_Start(&hadc1);
  printf("finished ADC CAL\n");
  printf("PID Task Started\n");
  for(;;)
  {
    osDelay(100);
    starttick = osKernelSysTick();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcbuf, 96);
    if (xSemaphoreTake(ADCSem, (TickType_t)1000) == pdTRUE) 
    {
      for(int i = 1; i < 16; i++)
      {
        adcbuf[0] += adcbuf[i*6+0];
        adcbuf[1] += adcbuf[i*6+1];
        adcbuf[2] += adcbuf[i*6+2];
        adcbuf[3] += adcbuf[i*6+3];
        adcbuf[4] += adcbuf[i*6+4];
        adcbuf[5] += adcbuf[i*6+5];
      }
      heater.ch0 = adcbuf[0];
      heater.ch1 = adcbuf[1];
      heater.ch2 = adcbuf[2];
      heater.ch3 = adcbuf[3];
      heater.ch4 = adcbuf[4];
      heater.ch5 = adcbuf[5];
      endtick = osKernelSysTick();
      printf("dma took %d ticks\n", (int)(endtick - starttick));
      for (int i = 0; i < 6; i++)
      {
        printf("0x%04x ", adcbuf[i]);
      }
      printf("\n");
      // heater.vin = adcbuf[0]; // and
      // heater.i1 = adcbuf[1];
      // heater.vin = adcbuf[2]; // and
      // heater.i2 = adcbuf[3];
      // heater.ttip = adcbuf[4];
      // heater.tamb = adcbuf[5];
      // heater.tint = adcbuf[6];
      // heater.tamb = adcbuf[7];
    }
  }
}

  // channels[0] = ADC_CHANNEL_1;      channels[0] = ADC_CHANNEL_0;
  // channels[1] = ADC_CHANNEL_3;      channels[1] = ADC_CHANNEL_2;
  // channels[2] = ADC_CHANNEL_TEMPSENSOR;      channels[2] = ADC_CHANNEL_4;
  // channels[3] = ADC_CHANNEL_1;      channels[3] = ADC_CHANNEL_0;

/**
  * @brief  Function implementing the startIMUTask thread.
  * @param  argument: Not used 
  * @retval None
  */
void startIMUTask(void const *argument)
{
  printf("IMU Task Started\n");
  for(;;)
  {
    osDelay(100);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    if (ADCSem) {
      xSemaphoreGiveFromISR(ADCSem, NULL);
    }
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  printf("Assert: file %s on line %d\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
