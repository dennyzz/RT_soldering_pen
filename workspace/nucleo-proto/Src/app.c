/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "ssd1306.h"

osThreadId defaultTaskHandle;
osThreadId blinkTaskHandle;
osThreadId displayTaskHandle;

void StartDefaultTask(void const * argument);
void BlinkerTask(void const * argument);
void DisplayTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  osThreadDef(blinkTask, BlinkerTask, osPriorityNormal, 0, 128);
  blinkTaskHandle = osThreadCreate(osThread(blinkTask), NULL);
  osThreadDef(dispTask, BlinkerTask, osPriorityNormal, 0, 128);
  displayTaskHandle = osThreadCreate(osThread(dispTask), NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

void BlinkerTask(void const * argument)
{
  const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
  for(;;)
  {
    vTaskDelay(xDelay);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    vTaskDelay(xDelay);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    vTaskDelay(xDelay);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    vTaskDelay(xDelay);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    vTaskDelay(xDelay);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    vTaskDelay(xDelay);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    vTaskDelay(xDelay);
    vTaskDelay(xDelay);
    vTaskDelay(xDelay);
    vTaskDelay(xDelay);
    vTaskDelay(xDelay);
    printf("HELLO\n");
  }
}

void DisplayTask(void const * argument)
{
  initOLED();
  for(;;)
  {

  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
