/**
 * @file mics6814.h
 * @defgroup mics6814 mics6814
 * @{
 *
 * ESP-IDF driver for mics6814 based arduino spi
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */
#pragma once

#include <stdint.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_NOT_USED    (-1)

typedef struct 
{
	float		  co;
	float		  nh3;
	float		  no2;
	int8_t 		status;
} mics6814_data_t;

esp_err_t mics6814_init(gpio_num_t miso, gpio_num_t sclk, gpio_num_t cs);
esp_err_t get_mics6814_data(void* data);

#ifdef __cplusplus
}
#endif

/**@}*/
