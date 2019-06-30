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
#include "hardware.h"
#include "oSI2CDrv.hpp"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

// FreeRTOS variables
osThreadId defaultTaskHandle;
osThreadId GUITaskHandle;
osThreadId PIDTaskHandle;
osThreadId IMUTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void StartDefaultTask(void const * argument);
void startGUITask(void const *argument);
void startPIDTask(void const *argument);
void startIMUTask(void const *argument);
/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  Setup_HAL();

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128 / 4);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(GUITask, startGUITask, osPriorityBelowNormal, 0, 2 * 128 / 4);
  GUITaskHandle = osThreadCreate(osThread(GUITask), NULL);

  osThreadDef(PIDTask, startPIDTask, osPriorityRealtime, 0, 1 * 128 / 4);
  PIDTaskHandle = osThreadCreate(osThread(PIDTask), NULL);

  osThreadDef(IMUTask, startIMUTask, osPriorityNormal, 0, 1 * 128 / 4);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  //Test that there was enough ram in the FreeRToS pool to allocate all the tasks
  if (IMUTaskHandle == 0)
    asm("bkpt");

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */
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
  for(;;)
  {
    osDelay(100);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  }
}
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
#define DEVICEADDR_OLED   (0x3c<<1)
void startGUITask(void const *argument)
{
  oSI2CDrv::init(&hi2c1);
  oSI2CDrv::oSInit();
  const uint8_t REFRESH_COMMANDS[17] = {0x80, 0xAF, 0x80, 0x21, 0x80, 0x20,
                                      0x80, 0x7F, 0x80, 0xC0, 0x80, 0x22,
                                      0x80, 0x00, 0x80, 0x01, 0x40};
  for(;;)
  {
    osDelay(1000);
    oSI2CDrv::Transmit(DEVICEADDR_OLED, (uint8_t*)REFRESH_COMMANDS, sizeof(REFRESH_COMMANDS));
  }
}
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
void startPIDTask(void const *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
void startIMUTask(void const *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}

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
    // asm("bkpt");
    oSI2CDrv::CpltCallback();
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    oSI2CDrv::CpltCallback();
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */









/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/