/**
 * @file ino_uart.h
 * @defgroup ino_uart ino_uart
 * @{
 *
 * ESP-IDF driver for arduino uart read
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __INO_UART_H__
#define __INO_UART_H__

#include <stdint.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
	CO,
	NO2,
	NH3
} gas_t;

typedef struct{
  uint16_t resistance;
  uint16_t base_reistance;
  float current_ratio;
  float meashure;
}meashure_6814_t;

esp_err_t init_arduino_uart(gpio_num_t txd_pin, gpio_num_t rxd_pin);
void get_6814_meashure(meashure_6814_t* nh3, meashure_6814_t* co, meashure_6814_t* no2);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __INO_UART_H__
