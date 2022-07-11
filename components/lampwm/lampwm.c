/**
 * @file ledpwm.c
 *
 * ESP-IDF driver for control LED lamp
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <string.h>
#include <math.h>
#include <esp_log.h>
#include <driver/ledc.h>
#include "lampwm.h"

static const char *TAG = "lampwm";

#define LEDC_TIMER			LEDC_TIMER_0
#define LEDC_MODE			LEDC_LOW_SPEED_MODE
//#define LEDC_OUTPUT_IO	(5) // Define the output GPIO
//#define LEDC_OUTPUT_IO		CONFIG_BLINK_GPIO // Define the output GPIO
#define LEDC_CHANNEL		LEDC_CHANNEL_0
#define LEDC_DUTY_RES		LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY			(4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY		(5000) // Frequency in Hertz. Set frequency at 5 kHz


esp_err_t lampwm_init_desc(gpio_num_t LEDC_OUTPUT_IO){

	// Prepare and then apply the LEDC PWM timer configuration
	ledc_timer_config_t ledc_timer = {
		.speed_mode			= LEDC_MODE,
		.timer_num			= LEDC_TIMER,
		.duty_resolution	= LEDC_DUTY_RES,
		.freq_hz			= LEDC_FREQUENCY,  // Set output frequency at 5 kHz
		.clk_cfg			= LEDC_AUTO_CLK
	};
	esp_err_t err = ledc_timer_config(&ledc_timer);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "LEDC timer config fail");
        return err;
    }

	// Prepare and then apply the LEDC PWM channel configuration
	ledc_channel_config_t ledc_channel = {
		.speed_mode			= LEDC_MODE,
		.channel			= LEDC_CHANNEL,
		.timer_sel			= LEDC_TIMER,
		.intr_type			= LEDC_INTR_DISABLE,
		.gpio_num			= LEDC_OUTPUT_IO,
		.duty				= 0, // Set duty to 0%
		.hpoint				= 0
	};
	err = ledc_channel_config(&ledc_channel);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "LEDC channel config fail");
    }
    return err;
}

esp_err_t lampwm_set_duty(const float percent){
	double maxduty = pow(2, 13) - 1;
	float _percent = percent / 100;
	uint32_t duty = maxduty * _percent;
	ESP_LOGI(TAG, "persent=%0.1f", percent);
	esp_err_t err = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "LEDC set duty fail");
        return err;
    }
	err = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "LEDC update duty fail");
    }
    return err;
}
