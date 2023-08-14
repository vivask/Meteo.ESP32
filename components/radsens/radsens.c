/**
 * @file radsens.c
 *
 * ESP-IDF driver for RadSens digital radiation sensor
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <string.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "radsens.h"

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf

static const char *TAG = "radsens";

/**\name RADSENS chip identifier */
#define RADSENS_CHIP_ID                 0x7D
/**\name RADSENS firmware version */
#define RADSENS_VERSION                 0x02

/**
 * RADSENS registers
 */
#define RADSENS_CHIP_ID_REG             0x00
#define RADSENS_FIRMWARE_REG            0x01
#define RADSENS_DYNAMIC_REG     		0x03
#define RADSENS_STATIC_REG      		0x06
#define RADSENS_PULSE_COUNTER_REG       0x09
#define RADSENS_ADDRESS_REG             0x10
#define RADSENS_HV_GENERATOR_REG        0x11
#define RADSENS_SENSITYVE_REG           0x12

#define RADSEN_REGISTER_COUNT			(19)

static i2c_dev_t i2c_dev;

static uint8_t current_sensitivity = RADSENS_SENSITIVITY_NORMAL;

static uint32_t pulse_count = 0;


inline static esp_err_t write_register8(i2c_dev_t *dev, uint8_t addr, uint8_t value)
{
    return i2c_dev_write_reg(dev, addr, &value, 1);
}


esp_err_t radsens_init_desc(i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio){
 
    i2c_dev.port = port;
    i2c_dev.addr = RADSENS_I2C_ADDRESS;
    i2c_dev.cfg.sda_io_num = sda_gpio;
    i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&i2c_dev);
}


esp_err_t radsens_init(){
    
    esp_err_t err;
    uint8_t chip_id;

   
    I2C_DEV_TAKE_MUTEX(&i2c_dev);

    err = i2c_dev_read_reg(&i2c_dev, RADSENS_CHIP_ID_REG, &chip_id, 1);
    if (err != ESP_OK) {
        return err;
    }

    if(chip_id != RADSENS_CHIP_ID){
        err = ESP_ERR_INVALID_VERSION;
    }   

    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    return err; 
}

esp_err_t radsens_read_data(radsens_data *comp_data) {
    
    esp_err_t err = ESP_OK;
    uint8_t	buffer[RADSEN_REGISTER_COUNT];

	if (comp_data == NULL) {
		err = ESP_FAIL;
        ESP_LOGI(TAG, "Invalid argument");
	}else {
        I2C_DEV_TAKE_MUTEX(&i2c_dev);
        esp_err_t err = i2c_dev_read(&i2c_dev, NULL, 0, buffer, RADSEN_REGISTER_COUNT);
        if(err != ESP_OK){
            ESP_LOGE(TAG, "%s: No ack, radsens not connected...skip...", esp_err_to_name(err));
        }else{
            pulse_count += (buffer[RADSENS_PULSE_COUNTER_REG] << 8) | buffer[RADSENS_ADDRESS_REG];
            comp_data->pulse_cnt = pulse_count;

            comp_data->i_static = (((uint32_t)buffer[RADSENS_STATIC_REG] << 16) |
                                ((uint16_t)buffer[RADSENS_STATIC_REG+1] << 8) |
                                buffer[RADSENS_STATIC_REG+2]) / 10.0;

            comp_data->i_dynamic = (((uint32_t)buffer[RADSENS_DYNAMIC_REG] << 16) |
                                ((uint16_t)buffer[RADSENS_DYNAMIC_REG+1] << 8) |
                                buffer[RADSENS_DYNAMIC_REG+2]) / 10.0;

            comp_data->sens = buffer[RADSENS_SENSITYVE_REG];

            comp_data->hv_state = buffer[RADSENS_HV_GENERATOR_REG];
        }

        I2C_DEV_GIVE_MUTEX(&i2c_dev);    
    }
    return err;
}

esp_err_t radsens_get_hv_generator_state(bool* state){
   
    I2C_DEV_TAKE_MUTEX(&i2c_dev);

    esp_err_t err = i2c_dev_read_reg(&i2c_dev, RADSENS_HV_GENERATOR_REG, state, 1);
    if (err != ESP_OK) {
        return err;
    }

    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    return err;
}

esp_err_t radsens_get_sensitivity(uint8_t* sensitivity){
   
    I2C_DEV_TAKE_MUTEX(&i2c_dev);

    esp_err_t err = i2c_dev_read_reg(&i2c_dev, RADSENS_SENSITYVE_REG, sensitivity, 1);
    if (err != ESP_OK) {
        return err;
    }

    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    return err;
}

esp_err_t radsens_set_hv_generator(bool state){
   
    uint8_t data = (state) ? 1 : 0;    
    esp_err_t err = write_register8(&i2c_dev, RADSENS_HV_GENERATOR_REG, data);
    if(err == ESP_OK) pulse_count = 0;
    return err;
}

esp_err_t radsens_set_sensitivity(uint8_t sensitivity){   
   
    esp_err_t err = write_register8(&i2c_dev, RADSENS_SENSITYVE_REG, sensitivity);
    if(err == ESP_OK) current_sensitivity = sensitivity;
    return err;
}
