/**
 * @file radsens.h
 * @defgroup radsens radsens
 * @{
 *
 * ESP-IDF driver for RadSens digital radiation sensor
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __RADSENS_H__
#define __RADSENS_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RADSENS_I2C_ADDRESS  0x66

/**
 * Sensitivity of Radsens module operation.
 */
typedef enum {
    RADSENS_SENSITIVITY_MIN = 0,  
    RADSENS_SENSITIVITY_NORMAL = 105, 
    RADSENS_SENSITIVITY_MAX = 255  
} RADSENS_Sensitivity;

/**
 * HV Generator of Radsens module operation.
 */
typedef enum {
    RADSENS_HV_GENERATOR_OFF = false,  
    RADSENS_HV_GENERATOR_ON = true 
} RADSENS_HV_Generator;

/**
 * Configuration parameters for Radsens module.
 * Use function ::radsens_init_default_params() to use default configuration.
 */
typedef struct {
    RADSENS_Sensitivity     sensitivity;
    RADSENS_HV_Generator    hv_generator; 
} radsens_params_t;

typedef struct {
    uint8_t         buffer[19];
    uint32_t        pulse_cnt;
    float           intensy_static;
    float           intensy_dynamic;
} radsens_data_t;


esp_err_t radsens_init_default_params(radsens_params_t *params);
esp_err_t radsens_init_desc(i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t radsens_init(radsens_params_t *params);
esp_err_t radsens_read_data(uint32_t* pulse, float* intensy_static, float* intensy_dynamic);
esp_err_t radsens_get_hv_generator_state(bool* state);
esp_err_t radsens_get_sensitivity(uint8_t* sensitivity);
esp_err_t radsens_set_hv_generator(bool state);
esp_err_t radsens_set_sensitivity(uint8_t sensitivity);
bool radsens_initialized();
uint8_t radsens_get_current_sensitivity();

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __RADSENS_H__
