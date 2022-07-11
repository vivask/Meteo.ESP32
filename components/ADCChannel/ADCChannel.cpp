#include <stdlib.h>
 
#include "ADCChannel.h"

ADCChannel::ADCChannel(uint8_t gpio)
{
    switch (gpio)
    {
    case 0:
        _unit = ADC_UNIT_2;
        _channel = ADC_CHANNEL_1;
        break;
    case 2:
        _unit = ADC_UNIT_2;
        _channel = ADC_CHANNEL_2;
        break;
    case 4:
        _unit = ADC_UNIT_2;
        _channel = ADC_CHANNEL_0;
        break;
    case 12:
        _unit = ADC_UNIT_2;
        _channel = ADC_CHANNEL_5;
        break;
    case 13:
        _unit = ADC_UNIT_2;
        _channel = ADC_CHANNEL_4;
        break;
    case 14:
        _unit = ADC_UNIT_2;
        _channel = ADC_CHANNEL_6;
        break;
    case 15:
        _unit = ADC_UNIT_2;
        _channel = ADC_CHANNEL_3;
        break;
    case 25:
        _unit = ADC_UNIT_2;
        _channel = ADC_CHANNEL_8;
        break;
    case 26:
        _unit = ADC_UNIT_2;
        _channel = ADC_CHANNEL_9;
        break;
    case 27:
        _unit = ADC_UNIT_2;
        _channel = ADC_CHANNEL_7;
        break;
    case 32:
        _unit = ADC_UNIT_1;
        _channel = ADC_CHANNEL_4;
        break;
    case 33:
        _unit = ADC_UNIT_1;
        _channel = ADC_CHANNEL_5;
        break;
    case 34:
        _unit = ADC_UNIT_1;
        _channel = ADC_CHANNEL_6;
        break;
    case 35:
        _unit = ADC_UNIT_1;
        _channel = ADC_CHANNEL_7;
        break;
    case 36:
        _unit = ADC_UNIT_1;
        _channel = ADC_CHANNEL_0;
        break;
    case 39:
        _unit = ADC_UNIT_1;
        _channel = ADC_CHANNEL_3;
        break;
    default:
        _unit = ADC_UNIT_MAX;
        break;
    }
}
