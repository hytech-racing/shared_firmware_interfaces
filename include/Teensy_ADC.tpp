#include "Teensy_ADC.h"

template <int TEENSY_ADC_NUM_CHANNELS>
Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::Teensy_ADC(const int* pins)
{
    analogReadResolution(TEENSY_ADC_DEFAULT_RESOLUTION);
    for (int i = 0; i < TEENSY_ADC_NUM_CHANNELS; i++)
    {
        _pins[i] = pins[i];
        Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::channels_[i] = AnalogChannel();
        Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::filter_channels_ = Filter_IIR(0, analogRead(_pins[i]));
    }
    
}

template <int TEENSY_ADC_NUM_CHANNELS>
void Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::init()
{
    for(int i = 0; i < TEENSY_ADC_NUM_CHANNELS; i++)
    {
        pinMode(_pins[i], INPUT);
    }
    
}
template <int TEENSY_ADC_NUM_CHANNELS>
void Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::init()
{
    Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::sample();
    Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::convert();
    
}
template <int TEENSY_ADC_NUM_CHANNELS>
void Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::sample()
{
    for (int i = 0; i < TEENSY_ADC_NUM_CHANNELS; i++) {
        uint16_t val = analogRead(_pins[i]);
        Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::channels_[i].lastSample = (Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::filter_channels_[i].filteredResult(val) & (~(1 << TEENSY_ADC_DEFAULT_RESOLUTION)));
    }
    
}

template <int TEENSY_ADC_NUM_CHANNELS>
void Teensy_ADC<int TEENSY_ADC_NUM_CHANNELS>::setAlphas(int pin, float alpha) override {
    for(int i = 0; i < TEENSY_ADC_NUM_CHANNELS; i++) {
        if(pin == _pins[i]) {
            FilterIIRMulti<TEENSY_ADC_NUM_CHANNELS>::setAlphas(i, alpha);
            break;
        }
    }
}



