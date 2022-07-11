#ifndef __ADCCHANNEL_H__
#define __ADCCHANNEL_H__

#include "driver/gpio.h"
#include "driver/adc.h"

#ifdef __cplusplus
extern "C" {
#endif


class ADCChannel{
    adc_channel_t   _channel;
    adc_unit_t      _unit;
public:    
    ADCChannel(uint8_t gpio);
    adc_channel_t channel(){return _channel;}
    adc_unit_t unit(){return _unit;}
};

#ifdef __cplusplus
}
#endif

#endif /* __ADCCHANNEL_H__ */