#ifndef _ADC_HPP_
#define _ADC_HPP_

// way to simultaneously measure a bunch of things 
Vin - Iin1
Vin - Iin2

Ttip - Tamb

class Adc {

public:
	enum class State {
		DONE, 
		MEASURE_IDLE, 
		MEASURE_HEAT,
	} _measure_state = State::DONE






}



// #endif _ADC_HPP_