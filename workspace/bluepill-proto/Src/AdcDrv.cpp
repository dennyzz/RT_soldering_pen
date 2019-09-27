/*
 * AdcDrv.cpp
 *
 *  Created on: Sept 26 2019
 *      Author: Denny
 */

#include "AdcDrv.hpp"
#include "hardware.h"
#include "semphr.h"

SemaphoreHandle_t AdcDrv::ADCSem;
ADC_HandleTypeDef* AdcDrv::hadc;
uint16_t AdcDrv::adcBuf[112];
uint32_t AdcDrv::starttick;
uint32_t AdcDrv::endtick;
uint8_t AdcDrv::numChannels;

void AdcDrv::init(ADC_HandleTypeDef *adcHandle, uint8_t numChan) {
  hadc = adcHandle;
  numChannels = numChan;
  ADCSem = xSemaphoreCreateBinary();
  HAL_ADCEx_Calibration_Start(hadc);
  printf("finished ADC CAL\n");
}

void AdcDrv::CpltCallback(ADC_HandleTypeDef* hadc) {
    HAL_ADC_Stop_DMA(hadc);
    if (ADCSem) 
      xSemaphoreGiveFromISR(ADCSem, NULL);
  }

void AdcDrv::measure(void) {
  starttick = osKernelSysTick();
  HAL_ADC_Start_DMA(hadc, (uint32_t*)adcBuf, 112);
  if (xSemaphoreTake(ADCSem, (TickType_t)1000) == pdTRUE) 
  {
    for(uint8_t i = 1; i < 16; i++) //oversample 16x for 16-bit measurements
    {
      for(uint8_t j = 0; j < numChannels; j++)
      {
        adcBuf[j] += adcBuf[i*numChannels+j];
      }
    }
    endtick = osKernelSysTick();
    printf("dma took %d ticks\n", (int)(endtick - starttick));
    for (int i = 0; i < 6; i++)
    {
      printf("0x%04x ", adcBuf[i]);
    }
    printf("\n");
  }
}

void AdcDrv::getRawValues(Heater_struct &heater)
{
  heater.vin = adcBuf[0];
  heater.i1 = adcBuf[1];
  heater.i2 = adcBuf[2];
  heater.tamb = adcBuf[3];
  heater.ttip = adcBuf[4];
  heater.tint = adcBuf[5];
  heater.vint = adcBuf[6];
}

void AdcDrv::getValues(Heater_struct &heater)
{
  uint32_t tmp;
  int32_t stmp;
  uint16_t cpu_mvolts;
  uint16_t intmax = 0xFFFF;
  uint16_t intvref_cal = 1200; // cursory mV internal ref
  // internal voltage in mV
  tmp = intvref_cal * 65536;
  tmp /= adcBuf[6];
  cpu_mvolts = tmp;
  heater.vint = tmp;

  // vin in mVolts with 68k/10k divider
  tmp = adcBuf[0];
  tmp *= cpu_mvolts * 78;
  tmp /= 10 * intmax;
  heater.vin = tmp;

  // Current in mA
  tmp = adcBuf[1];
  tmp *= cpu_mvolts * 100;
  tmp /= intmax * 100;
  heater.i1 = tmp;
  // Current in mA
  tmp = adcBuf[2];
  tmp *= cpu_mvolts * 100;
  tmp /= intmax * 50;
  heater.i2 = tmp;


  stmp = adcBuf[3];
  stmp *= cpu_mvolts * 10;
  stmp /= intmax;
  stmp -= 5000;
  stmp *= 10;
  heater.tamb = stmp;

  stmp = adcBuf[5];
  stmp *= cpu_mvolts * 10; // in 1mV's
  stmp /= intmax;
  stmp -= 13220; // (1430 - 4.3*25)
  stmp *= 43;
  heater.tint = stmp;

  stmp = adcBuf[4];
  stmp *= cpu_mvolts * 100;
  stmp /= intmax;
  stmp /= 15;
  heater.ttip = stmp;

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    AdcDrv::CpltCallback(hadc);
  }
}


