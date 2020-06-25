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
        MEASURE_TEMP, 
        MEASURE_POWER,
    };
    State _measure_state = State::DONE;

private: 
    static const uint16_t u16intmax = 0xFFFF;
    static const int MAX_NUM_CHANNELS = 7;
    static const int OVERSAMPLE = 2 ^ 4; //oversample 16x for 16-bit measurements (12-bit adc << 4)
    uint16_t adcBuf[MAX_NUM_CHANNELS * OVERSAMPLE];
    SemaphoreHandle_t ADCSem;
    ADC_HandleTypeDef *hadc;
    // uint32_t starttick;
    // uint32_t endtick;
    uint8_t numChannels;
    uint16_t referenceVolts;
public:
    void init(ADC_HandleTypeDef *adcHandle, State s);

    // conversions
    uint16_t updateIntVRef(uint16_t rawData);
    int16_t convertExtTemp(uint16_t rawData);
    int16_t convertIntTemp(uint16_t rawData);
    int16_t convertTipTemp(uint16_t rawData);
    uint16_t convertVoltageSense(uint16_t rawData);
    uint16_t convertCurrentSense(uint16_t rawData, uint16_t gain);

    int measure(void);
    void config_channels(ADC_HandleTypeDef* hadc, uint32_t *channels, uint32_t sampleTime, uint32_t numChan);
    void set_channel_group(State s);
    void getRawValues(Heater_struct &heater);
    void getValues(Heater_struct &heater);
    void CpltCallback(ADC_HandleTypeDef* hadc);

    static AdcDrv &get_instance() {
        static AdcDrv instance;
        return instance;
    }
};

#endif //_ADCDRV_HPP_
