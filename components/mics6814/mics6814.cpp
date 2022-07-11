#include <math.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"

#include "mics6814.h"

static const char *TAG = "MICS6814";

#define NO_OF_SAMPLES CONFIG_NO_OF_SAMPLES
#define CALIBRATION_ATTEMPS_MAX CONFIG_CALIBRATION_ATTEMPS_MAX
#define CALIBRATION_ATTEMPS_MAX 99999

#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_0

mics6814::mics6814() : 
    _width(ADC_WIDTH),
    _atten(ADC_ATTEN)
{

    _adcCO = NULL;
    _adcNO2 = NULL;
    _adcNH3 = NULL;
}

mics6814::~mics6814(){

    if(_adcCO) delete _adcCO;
    if(_adcNO2) delete _adcNO2;
    if(_adcNH3) delete _adcNH3;
}


esp_err_t mics6814::mics6814_init(const uint8_t pinNO2, const uint8_t pinNH3, const uint8_t pinCO){
    _adcCO = new ADCChannel(pinCO);
    _adcNO2 = new ADCChannel(pinNO2);
    _adcNH3 = new ADCChannel(pinNH3);

	uint16_t raw;
    readNO2(raw);
	vTaskDelay(50 / portTICK_RATE_MS);
	ESP_LOGW(TAG, "NO2: %d", raw);
    readCO(raw);
	ESP_LOGW(TAG, "CO: %d", raw);
	vTaskDelay(50 / portTICK_RATE_MS);
    readNH3(raw);
	ESP_LOGW(TAG, "NH3: %d", raw);

    return ESP_OK;
}

esp_err_t mics6814::read_1(const adc1_channel_t channel, uint16_t& raw_data){

	unsigned long rs = 0;
    esp_err_t esp_err = adc1_config_width(_width);
	if(esp_err != ESP_OK){
		ESP_LOGE(TAG, "Config width error");
		return esp_err;
	}

    esp_err = adc1_config_channel_atten(channel, _atten);
	if(esp_err != ESP_OK){
		ESP_LOGE(TAG, "Config ADC1 channel %d attenuation error", channel);
		return esp_err;
	}

    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        rs += adc1_get_raw(channel);
    }
	raw_data = rs / NO_OF_SAMPLES;
	return ESP_OK;
}

esp_err_t mics6814::read_2(const adc2_channel_t channel, uint16_t& raw_data){

	unsigned long rs = 0;
	esp_err_t esp_err = adc2_config_channel_atten(channel, _atten);
	if(esp_err != ESP_OK){
		ESP_LOGE(TAG, "Config ADC2 channel %d attenuation error", channel);
		return esp_err;
	}

    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        int raw;
        adc2_get_raw(channel, _width, &raw);
        rs += raw;
    }   
	raw_data = rs / NO_OF_SAMPLES;
	return ESP_OK;
}

esp_err_t mics6814::read(const adc_channel_t channel, const adc_unit_t unit, uint16_t& raw_data){
    
    
    raw_data = 0;
    if(!(unit == ADC_UNIT_1 || unit == ADC_UNIT_2)){
        ESP_LOGE(TAG, "Incorrect ADC unit for chanel: %d", channel);
        return ESP_FAIL;
    }

	return (unit == ADC_UNIT_1) ? read_1((adc1_channel_t)channel, raw_data) : read_2((adc2_channel_t)channel, raw_data);
}

/**
 * Калибрует MICS-6814 перед использованием
 *
 * Алгоритм работы:
 *
 * Непрерывно измеряет сопротивление,
 *   с сохранением последних N измерений в буфере.
 * Вычисляет плавающее среднее за последние секунды.
 * Если текущее измерение близко к среднему,
 *   то считаем, что каллибровка прошла успешно.
 */
bool mics6814::calibrate()
{
	// Кол-во секунд, которые должны пройти прежде,
	//   чем мы будем считать, что каллибровка завершена
	// (Меньше 64 секунд, чтобы избежать переполнения)
	uint8_t seconds = 10;

	// Допустимое отклонение для среднего от текущего значения
	uint8_t delta = 2;

	// Буферы измерений
	uint16_t bufferNH3[seconds];
	uint16_t bufferCO[seconds];
	uint16_t bufferNO2[seconds];

	// Указатели для следующего элемента в буфере
	uint8_t pntrNH3 = 0;
	uint8_t pntrCO = 0;
	uint8_t pntrNO2 = 0;

	// Текущая плавающая сумма в буфере
	uint16_t fltSumNH3 = 0;
	uint16_t fltSumCO = 0;
	uint16_t fltSumNO2 = 0;

	// Текущее измерение
	uint16_t curNH3;
	uint16_t curCO;
	uint16_t curNO2;

	// Флаг стабильности показаний
	bool isStableNH3 = false;
	bool isStableCO  = false;
	bool isStableNO2 = false;

	// Забиваем буфер нулями
	for (int i = 0; i < seconds; ++i)
	{
		bufferNH3[i] = 0;
		bufferCO[i]  = 0;
		bufferNO2[i] = 0;
	}

	// Калибруем
	ESP_LOGI(TAG, "Calibration start...");
	uint16_t count = 0;
	do
	{
		vTaskDelay(1000 / portTICK_RATE_MS);

        readNO2(curNO2);
		vTaskDelay(50 / portTICK_RATE_MS);
        readCO(curCO);
		vTaskDelay(50 / portTICK_RATE_MS);
        readNH3(curNH3);

		// Обновите плавающую сумму, вычитая значение, 
		//   которое будет перезаписано, и добавив новое значение.
		fltSumNH3 = fltSumNH3 + curNH3 - bufferNH3[pntrNH3];
		fltSumCO  = fltSumCO  + curCO  - bufferCO[pntrCO];
		fltSumNO2 = fltSumNO2 + curNO2 - bufferNO2[pntrNO2];

		// Сохраняем d буфере новые значения
		bufferNH3[pntrNH3] = curNH3;
		bufferCO[pntrCO]   = curCO;
		bufferNO2[pntrNO2] = curNO2; 

		// Определение состояний флагов
		isStableNH3 = abs(fltSumNH3 / seconds - curNH3) < delta;
		isStableCO  = abs(fltSumCO  / seconds - curCO)  < delta;
		isStableNO2 = abs(fltSumNO2 / seconds - curNO2) < delta;

		ESP_LOGI(TAG, "Deviation NH3: %d", abs(fltSumNH3 / seconds - curNH3));
		ESP_LOGI(TAG, "Deviation CO: %d", abs(fltSumCO  / seconds - curCO));
		ESP_LOGI(TAG, "Deviation NO2: %d", abs(fltSumNO2 / seconds - curNO2));


		// Указатель на буфер
		pntrNH3 = (pntrNH3 + 1) % seconds;
		pntrCO  = (pntrCO  + 1) % seconds;
		pntrNO2 = (pntrNO2 + 1) % seconds;

	} while (!(isStableNH3 || isStableCO || isStableNO2) && count++ < CALIBRATION_ATTEMPS_MAX);

	_baseNH3 = fltSumNH3 / seconds;
	_baseCO  = fltSumCO  / seconds;
	_baseNO2 = fltSumNO2 / seconds;

	bool ret = (count < CALIBRATION_ATTEMPS_MAX);

	if(ret){
		ESP_LOGI(TAG, "Calibration success per %d cicles", count);
	}else{
		ESP_LOGW(TAG, "Calibration fail");
	}
	
	return ret;
}

void mics6814::loadCalibrationData(
	uint16_t baseNH3,
	uint16_t baseCO,
	uint16_t baseNO2)
{
	_baseNH3 = baseNH3;
	_baseCO  = baseCO;
	_baseNO2 = baseNO2;
}

uint16_t mics6814::getBaseResistance(channel_t channel)
{
	switch (channel)
	{
	case CH_NH3:
		return _baseNH3;
	case CH_CO:
		return _baseCO;
	case CH_NO2:
		return _baseNO2;
	}

	return 0;
}

/**
 * Запрашивает текущее сопротивление для данного канала от датчика. 
 * Значение - это значение АЦП в диапазоне от 0 до 1024.
 * 
 * @param channel
 * Канал для считывания базового сопротивления.
 *
 * @return
 * Беззнаковое 16-битное базовое сопротивление выбранного канала.
 */
uint16_t mics6814::getResistance(channel_t channel)
{
    uint16_t raw;
	unsigned long rs = 0;
	int counter = 0;

	switch (channel)
	{
	case CH_CO:
		for (int i = 0; i < 100; i++)
		{
            readCO(raw);
            rs += raw;            
			counter++;
			vTaskDelay(2 / portTICK_RATE_MS);
		}
        break;
	case CH_NO2:
		for (int i = 0; i < 100; i++)
		{
            readNO2(raw);
            rs += raw;            
			counter++;
			vTaskDelay(2 / portTICK_RATE_MS);
		}
        break;
	case CH_NH3:
		for (int i = 0; i < 100; i++)
		{
            readNH3(raw);
            rs += raw;            
			counter++;
			vTaskDelay(2 / portTICK_RATE_MS);
		}
	}

	return (counter != 0) ? rs / counter : 0;
}

/**
 * Вычисляет коэффициент текущего сопротивления для данного канала.
 * 
 * @param channel
 * Канал для запроса значений сопротивления.
 *
 * @return
 * Коэффициент сопротивления с плавающей запятой для данного датчика.
 */
float mics6814::getCurrentRatio(channel_t channel)
{
	float baseResistance = (float)getBaseResistance(channel);
	float resistance = (float)getResistance(channel);

	return resistance / baseResistance * (1023.0 - baseResistance) / (1023.0 - resistance);

	return -1.0;
}

/**
 * Измеряет концентрацию газа в промилле для указанного газа.
 *
 * @param gas
 * Газ для расчета концентрации.
 *
 * @return
 * Текущая концентрация газа в частях на миллион (ppm).
 */
float mics6814::measure(gas_t gas)
{
	float ratio;
	float c = 0;

	switch (gas)
	{
	case CO:
		ratio = getCurrentRatio(CH_CO);
		c = pow(ratio, -1.179) * 4.385;
		break;
	case NO2:
		ratio = getCurrentRatio(CH_NO2);
		c = pow(ratio, 1.007) / 6.855;
		break;
	case NH3:
		ratio = getCurrentRatio(CH_NH3);
		c = pow(ratio, -1.67) / 1.47;
		break;
	}

	return isnan(c) ? -1 : c;
}


esp_err_t mics6814::read_1_raw(const adc1_channel_t channel, uint16_t& raw_data, uint32_t& voltage){

	unsigned long rs = 0;

    //Characterize ADC
    esp_adc_cal_characteristics_t *adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
//    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
//    print_char_val_type(val_type);

    esp_err_t esp_err = adc1_config_width(_width);
	if(esp_err != ESP_OK){
		ESP_LOGE(TAG, "Config width error");
		return esp_err;
	}

    esp_err = adc1_config_channel_atten(channel, _atten);
	if(esp_err != ESP_OK){
		ESP_LOGE(TAG, "Config ADC1 channel %d attenuation error", channel);
		return esp_err;
	}

    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        rs += adc1_get_raw(channel);
    }
	raw_data = rs / NO_OF_SAMPLES;
	voltage = esp_adc_cal_raw_to_voltage(raw_data, adc_chars);
	free(adc_chars);
	return ESP_OK;
}
