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
uint16_t AdcDrv::referenceVolts;
const uint16_t AdcDrv::u16intmax;

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
  // update internal Vref measurement
  referenceVolts = updateIntVRef(adcBuf[6]);
  heater.vint = referenceVolts;
  heater.vin = convertVoltageSense(adcBuf[0]);
  heater.i1 = convertCurrentSense(adcBuf[1], 100);
  heater.i2 = convertCurrentSense(adcBuf[2], 50);
  heater.tamb = convertExtTemp(adcBuf[3]);
  heater.ttip = convertTipTemp(adcBuf[4]);
  heater.tint = convertIntTemp(adcBuf[5]);
  printf("int:%d\n", adcBuf[5]);
}

uint16_t AdcDrv::updateIntVRef(uint16_t rawData)
{
  // TODO: put this into calibration settings
  uint16_t intvref_cal = 1200; // 1.20V 
  uint32_t tmp = intvref_cal * u16intmax;
  tmp /= rawData;
  return (uint16_t)tmp;
}

int16_t AdcDrv::convertExtTemp(uint16_t rawData)
{
  // in 1/10degC in int16
  int32_t stmp = rawData;
  // using MCP9700A (0C == 500mV) (10mV/C)
  stmp *= referenceVolts * 10;
  stmp -= 5000 << 16; // offset is 5000 100uV's 
  stmp /= 10; // 10mV/C 
  stmp /= u16intmax;
  return (int16_t)stmp;
}

int16_t AdcDrv::convertIntTemp(uint16_t rawData)
{
  // in 1/10degC in int16
  int32_t stmp = rawData;
  // from spec 1.43V = 25C approx 1.322V@0C
  // 4.3mV/C
  stmp *= referenceVolts * 10; // in 100uV's
  stmp -= 13225 << 16; // (1430 - 4.3*25) offset from 0C in 100uV's
  stmp /= 430;
  stmp *= 100;
  stmp /= u16intmax;
  return (int16_t)stmp;
}

int16_t AdcDrv::convertTipTemp(uint16_t rawData)
{
  // in 1/10degC in int16
  int32_t stmp = rawData;
  stmp *= referenceVolts * 10;
  // TODO: change this to a calibration value
  // TODO: double check these numbers for thermocouple?
  // TODO: the math used to calc gain looks wrong?!?!
  stmp /= 152; // 371x gain 41uV/C = 15.2mV/C
  stmp *= 100;
  stmp /= u16intmax;
  // returned is still degrees above reference junction!
  return (int16_t)stmp;
}

uint16_t AdcDrv::convertVoltageSense(uint16_t rawData)
{
  // vin in mVolts with 68k/10k divider
  uint32_t tmp = rawData;
  // TODO: put this in calibration settings?
  tmp *= referenceVolts;
  tmp /= 10;
  tmp *= 78;
  tmp /= u16intmax;
  return (uint16_t)tmp;
}

uint16_t AdcDrv::convertCurrentSense(uint16_t rawData, uint16_t gain)
{
  // Current in mA for INA180A2/3 (A2 == 50V/V) (A3 == 100V/V) 
  uint32_t tmp = rawData;
  tmp *= referenceVolts;
  tmp /= gain; // gain V/V
  tmp *= 100; // 10mOhm sense resistor mV/(10/1000)
  tmp /= u16intmax;
  return (uint16_t)tmp;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    AdcDrv::CpltCallback(hadc);
  }
}


