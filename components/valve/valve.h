/**
 * @file valve.h
 * @defgroup valve valve
 * @{
 *
* ESP-IDF driver for salenoid Valve control
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __VALVE_H__
#define __VALVE_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t valve_init_desc(gpio_num_t gpio);
esp_err_t valve_close();
esp_err_t valve_open();

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __VALVE_H__
