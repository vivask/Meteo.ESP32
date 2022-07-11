#ifndef __MICS6814_H__
#define __MICS6814_H__

#include "ADCChannel.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	CH_CO,
	CH_NO2,
	CH_NH3
} channel_t;

typedef enum
{
	CO,
	NO2,
	NH3
} gas_t;


class mics6814
{
public:
	mics6814();
    ~mics6814();

	esp_err_t mics6814_init(const uint8_t pinNO2, const uint8_t pinNH3, const uint8_t pinCO);

	bool calibrate();
	void loadCalibrationData(
		uint16_t base_NH3,
		uint16_t base_RED,
		uint16_t base_OX);

	float measure(gas_t gas);

	uint16_t getResistance    (channel_t channel);
	uint16_t getBaseResistance(channel_t channel);
	float    getCurrentRatio  (channel_t channel);
private:
	ADCChannel* _adcCO;
	ADCChannel* _adcNO2;
	ADCChannel* _adcNH3;

	uint16_t _baseNH3;
	uint16_t _baseCO;
	uint16_t _baseNO2;

	const adc_bits_width_t 	_width;
	const adc_atten_t 		_atten;

private:
	esp_err_t read(const adc_channel_t channel, const adc_unit_t unit, uint16_t& raw_data);
	esp_err_t read_1(const adc1_channel_t channel, uint16_t& raw_data);
	esp_err_t read_2(const adc2_channel_t channel, uint16_t& raw_data);
	esp_err_t readNH3(uint16_t& raw_data) { return read(_adcNH3->channel(), _adcNH3->unit(), raw_data); }
	esp_err_t readCO(uint16_t& raw_data) { return read(_adcCO->channel(), _adcCO->unit(), raw_data); }
	esp_err_t readNO2(uint16_t& raw_data) { return read(_adcNO2->channel(), _adcNO2->unit(), raw_data); }
	esp_err_t read_1_raw(const adc1_channel_t channel, uint16_t& raw_data, uint32_t& voltage);
public:
	esp_err_t readRawNH3(uint16_t& raw_data, uint32_t& voltage) { return read_1_raw((adc1_channel_t)_adcNH3->channel(), raw_data, voltage); }
	esp_err_t readRawNO2(uint16_t& raw_data, uint32_t& voltage) { return read_1_raw((adc1_channel_t)_adcNO2->channel(), raw_data, voltage); }
	esp_err_t readRawCO(uint16_t& raw_data, uint32_t& voltage) { return read_1_raw((adc1_channel_t)_adcCO->channel(), raw_data, voltage); }
};

#ifdef __cplusplus
}
#endif

#endif /* __MICS6814_H__ */