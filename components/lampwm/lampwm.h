/**
 * @file ledpwm.h
 * @defgroup ledpwm ledpwm
 * @{
 *
 * ESP-IDF driver for control LED lamp
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __LEDPWM_H__
#define __LEDPWM_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lampwm_init_desc(gpio_num_t LEDC_OUTPUT_IO);
esp_err_t lampwm_set_duty(const float percent);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __LEDPWM_H__
