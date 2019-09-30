#ifndef _ADCDRV_HPP_
#define _ADCDRV_HPP_

#include "hardware.h"
#include "semphr.h"

// way to simultaneously measure a bunch of things 

class AdcDrv {

public:
  enum class State {
    DONE, 
    MEASURE_IDLE, 
    MEASURE_HEAT,
  } _measure_state = State::DONE;

private: 
  static uint16_t adcBuf[112];
  static SemaphoreHandle_t ADCSem;
  static ADC_HandleTypeDef *hadc;
  static uint32_t starttick;
  static uint32_t endtick;
  static uint8_t numChannels;
  static uint16_t referenceVolts;
  static const uint16_t u16intmax = 0xFFFF;
public:
  static void init(ADC_HandleTypeDef *adcHandle, uint8_t numChan);

  // conversions
  static uint16_t updateIntVRef(uint16_t rawData);
  static int16_t convertExtTemp(uint16_t rawData);
  static int16_t convertIntTemp(uint16_t rawData);
  static int16_t convertTipTemp(uint16_t rawData);
  static uint16_t convertVoltageSense(uint16_t rawData);
  static uint16_t convertCurrentSense(uint16_t rawData, uint16_t gain);

  static void measure(void);
  static void getRawValues(Heater_struct &heater);
  static void getValues(Heater_struct &heater);
  static void CpltCallback(ADC_HandleTypeDef* hadc);

};

#endif //_ADCDRV_HPP_
