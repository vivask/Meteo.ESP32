/**
 * @file ze08ch2o.h
 * @defgroup ze08ch2o ze08ch2o
 * @{
 *
 * ESP-IDF driver for ze08ch2o digital phenols sensor
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __ZE08CH2O_H__
#define __ZE08CH2O_H__

#include <stdint.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZE08CH2O_READ_ERROR_CODE                        (0xFFFF)

typedef struct {
	uint8_t gasName;
	uint8_t gasUnit;
	uint8_t noDecimalByte;
	uint8_t concentrationHighByte, concentrationLowByte;
	uint8_t fullRangeHighByte, fullRangeLowByte;
	uint8_t checkSum;
} ze08ch2o_data_t;

esp_err_t ze08ch2o_init(gpio_num_t rxd_gpio);
uint16_t ze08ch2o_read();

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __ZE08CH2O_H__
