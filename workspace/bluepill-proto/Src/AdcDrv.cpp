/*
 * AdcDrv.cpp
 *
 *  Created on: Sept 26 2019
 *      Author: Denny
 */

#include "AdcDrv.hpp"
#include "hardware.h"
#include "semphr.h"


void AdcDrv::init(ADC_HandleTypeDef *adcHandle, State s) {
    hadc = adcHandle;
    set_channel_group(s);
    ADCSem = xSemaphoreCreateBinary();
    HAL_ADCEx_Calibration_Start(hadc);
    printf("finished ADC CAL\n");
}

void AdcDrv::CpltCallback(ADC_HandleTypeDef* hadc) {
    HAL_ADC_Stop_DMA(hadc);
    if (ADCSem) {
        xSemaphoreGiveFromISR(ADCSem, NULL);
    }
}

int AdcDrv::measure(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_ADC_Start_DMA(hadc, (uint32_t*)adcBuf, numChannels*OVERSAMPLE);
    if (xSemaphoreTake(ADCSem, (TickType_t)1000) == pdTRUE) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        for(uint8_t i = 1; i < OVERSAMPLE; i++) 
        {
            for(uint8_t j = 0; j < numChannels; j++)
            {
                adcBuf[j] += adcBuf[i*numChannels+j];
            }
        }
    }
}

void AdcDrv::config_channels(ADC_HandleTypeDef* hadc, uint32_t *channels, uint32_t sampleTime, uint32_t numChan)
{
    assert_param(numChan <= ADC_REGULAR_RANK_16);
    ADC_ChannelConfTypeDef sConfig = {0};
    for (int i = 0; i < numChan; i++)
    {
        sConfig.Channel = channels[i];
        sConfig.Rank = i+1;
        sConfig.SamplingTime = sampleTime;
        if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }
    return;
}

void AdcDrv::set_channel_group(State s)
{
    uint32_t channels[MAX_NUM_CHANNELS];

    if (s == State::MEASURE_TEMP) {
        _measure_state = s;
        numChannels = 4;
        hadc1.Init.NbrOfConversion = numChannels;
        channels[0] = TEMP_AMB_CHAN;
        channels[1] = TEMP_TIP_CHAN;
        channels[2] = ADC_CHANNEL_TEMPSENSOR;
        channels[3] = ADC_CHANNEL_VREFINT;
    }
    else if (s == State::MEASURE_POWER) {
        _measure_state = s;
        numChannels = 4;
        hadc1.Init.NbrOfConversion = numChannels;
        channels[0] = VSENSE_CHAN;
        channels[1] = ISENSE1_CHAN;
        channels[2] = ISENSE2_CHAN;
        channels[3] = ADC_CHANNEL_VREFINT;
    }
    else if (s == State::MEASURE_IDLE) {
        _measure_state = s;
        numChannels = 7;
        hadc1.Init.NbrOfConversion = numChannels;
        channels[0] = VSENSE_CHAN;
        channels[1] = ISENSE1_CHAN;
        channels[2] = ISENSE2_CHAN;
        channels[3] = TEMP_AMB_CHAN;
        channels[4] = TEMP_TIP_CHAN;
        channels[5] = ADC_CHANNEL_TEMPSENSOR;
        channels[6] = ADC_CHANNEL_VREFINT;
    }
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    config_channels(&hadc1, channels, ADC_SAMPLETIME_239CYCLES_5, numChannels);

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
    if (_measure_state == State::MEASURE_POWER) {
        referenceVolts = updateIntVRef(adcBuf[3]);
        heater.vint = referenceVolts;
        heater.vin = convertVoltageSense(adcBuf[0]);
        heater.i1 = convertCurrentSense(adcBuf[1], 50);
        heater.i2 = convertCurrentSense(adcBuf[2], 100);
    }
    else if (_measure_state == State::MEASURE_TEMP) {
        referenceVolts = updateIntVRef(adcBuf[3]);
        heater.vint = referenceVolts;
        heater.tamb = convertExtTemp(adcBuf[0]);
        heater.ttip = convertTipTemp(adcBuf[1]);
        heater.tint = convertIntTemp(adcBuf[2]);
    }
    else if (_measure_state == State::MEASURE_IDLE) {
        referenceVolts = updateIntVRef(adcBuf[6]);
        heater.vint = referenceVolts;
        heater.vin = convertVoltageSense(adcBuf[0]);
        heater.i1 = convertCurrentSense(adcBuf[1], 50);
        heater.i2 = convertCurrentSense(adcBuf[2], 100);
        heater.tamb = convertExtTemp(adcBuf[3]);
        heater.ttip = convertTipTemp(adcBuf[4]);
        heater.tint = convertIntTemp(adcBuf[5]);
    }
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
    stmp *= 10;
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
        AdcDrv::get_instance().CpltCallback(hadc);
    }
}


