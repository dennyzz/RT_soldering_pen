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
#include "main.hpp"
#include "cmsis_os.h"
#include "setup.h"
#include "hardware.h"
#include "oSI2CDrv.hpp"
#include "display.hpp"
#include "screen.hpp"
#include "screens/debug_screen.hpp"
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

  osThreadDef(PIDTask, startPIDTask, osPriorityRealtime, 0, 256 / 4);
  PIDTaskHandle = osThreadCreate(osThread(PIDTask), NULL);

  osThreadDef(IMUTask, startIMUTask, osPriorityNormal, 0, 256 / 4);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  volatile uint32_t size = xPortGetFreeHeapSize();
  printf("Heap size:%d bytes\n", size);
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
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */

void startGUITask(void const *argument)
{
  printf("GUI Task Initializing\n");
  display.init();
  Display::FrameBuff &fb = display.get_fb();
  char buffer[10];
  uint16_t x = 0;
  uint16_t y = 0;
  // screen::Screen* list[screen::ScreenId::MAX];
  // screen::GUIManager guiMan(list);
  // screen::Debug debug(guiMan);
  // list[screen::ScreenId::DEBUG] = &debug;
  printf("GUI Task Started\n");
  for(;;)
  {
    // osDelay(50);
    for(int i = 0; i < 1000; i++)
    {
      sprintf(buffer, "%3d", i);
      x = fb.draw_text(50, 10, buffer, Font::num22);
      fb.draw_text(x, 10, "\260F", Font::num7);
      display.redraw();
      printf("called redraw\n");
      osDelay(50);
      fb.clear();
    }

    // guiMan.get()->tick();
    // guiMan.get()->draw();
  }
}
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
void startPIDTask(void const *argument)
{
  printf("PID Task Started\n");
  for(;;)
  {
    osDelay(100);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  }
}
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
void startIMUTask(void const *argument)
{
  printf("IMU Task Started\n");
  for(;;)
  {
    osDelay(100);
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
