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

/**
 * RADSENS registers
 */
#define RADSENS_REG_ID                  0x00
#define RADSENS_ID                      0x7D
#define RADSENS_VERSION                 0x02
#define RADSENS_FIRMWARE_REG            0x01
#define RADSENS_INTENSY_DYNAMIC_REG     0x03
#define RADSENS_INTENSY_STATIC_REG      0x06
#define RADSENS_PULSE_COUNTER_REG       0x09
#define RADSENS_ADDRESS_REG             0x10
#define RADSENS_HV_GENERATOR_REG        0x11
#define RADSENS_SENSITYVE_REG           0x12

static i2c_dev_t i2c_dev;
static radsens_data_t *radsens_data = NULL;
//static bool initialized = false;
static uint8_t current_sensitivity = RADSENS_SENSITIVITY_NORMAL;

static float prvious_static = 0.0;
static float prvious_dynamic = 0.0;
static const int deviation = 10;

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            I2C_DEV_GIVE_MUTEX(&i2c_dev); \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

inline static esp_err_t write_register8(i2c_dev_t *dev, uint8_t addr, uint8_t value)
{
    return i2c_dev_write_reg(dev, addr, &value, 1);
}

esp_err_t radsens_init_default_params(radsens_params_t *params){

    CHECK_ARG(params);

    params->sensitivity = RADSENS_SENSITIVITY_NORMAL;
    params->hv_generator = RADSENS_HV_GENERATOR_ON;

    return ESP_OK;
}

esp_err_t radsens_init_desc(i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio){
 
    i2c_dev.port = port;
    i2c_dev.addr = RADSENS_I2C_ADDRESS;
    i2c_dev.cfg.sda_io_num = sda_gpio;
    i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    size_t sz = sizeof(radsens_data_t);
    radsens_data = (radsens_data_t*)malloc(sz);
    memset(radsens_data, 0x00, sz);

    return i2c_dev_create_mutex(&i2c_dev);
}

esp_err_t radsens_free_desc()
{
    free(radsens_data);
    radsens_data = NULL;
    return i2c_dev_delete_mutex(&i2c_dev);
}

esp_err_t radsens_init(radsens_params_t *params){
    
    esp_err_t err;
    uint8_t register_id;

    CHECK_ARG(params);
    
    I2C_DEV_TAKE_MUTEX(&i2c_dev);

    err = i2c_dev_read_reg(&i2c_dev, RADSENS_REG_ID, &register_id, 1);
    CHECK_LOGE(err, "Sensor not found");

    if(register_id == RADSENS_ID){
        err = write_register8(&i2c_dev, RADSENS_SENSITYVE_REG, params->sensitivity);
        CHECK_LOGE(err, "Error write RADSENS_SENSITYVE_REG");
    }else{
        err = ESP_ERR_INVALID_VERSION;
        ESP_LOGE(TAG, "Wrong sensor ID: %02x\n", register_id);
    }   

    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    if(err == ESP_OK){
        
        bool hvState;
        
        vTaskDelay(100 / portTICK_RATE_MS);

        CHECK_LOGE(radsens_get_sensitivity(&current_sensitivity), "Error read sensitivity");
        CHECK_LOGE(radsens_get_hv_generator_state(&hvState), "Error read HV generator state");
        if(params->hv_generator != hvState){
            CHECK_LOGE(radsens_set_hv_generator(params->hv_generator), "Error set HV generator state");
        }
    }

    return err; 
}

static esp_err_t radsens_read(radsens_data_t* data){
    
    I2C_DEV_TAKE_MUTEX(&i2c_dev);
    //ESP_LOGI(TAG, "Take mutex...");
    esp_err_t err = i2c_dev_read(&i2c_dev, NULL, 0, data->buffer, sizeof(data->buffer));
    if(err != ESP_OK){
        ESP_LOGE(TAG, "%s: No ack, radsens not connected...skip...", esp_err_to_name(err));
    }else{
        if(data->buffer[RADSENS_REG_ID] == RADSENS_ID){
            data->pulse_cnt += (data->buffer[9] << 8) | data->buffer[10];
            data->intensy_static = (((uint32_t)data->buffer[RADSENS_INTENSY_STATIC_REG] << 16) | 
                         ((uint16_t)data->buffer[RADSENS_INTENSY_STATIC_REG+1] << 8) | 
                         data->buffer[RADSENS_INTENSY_STATIC_REG+2]) / 10.0;
            data->intensy_dynamic = (((uint32_t)data->buffer[RADSENS_INTENSY_DYNAMIC_REG] << 16) | 
                          ((uint16_t)data->buffer[RADSENS_INTENSY_DYNAMIC_REG+1] << 8) | 
                          data->buffer[RADSENS_INTENSY_DYNAMIC_REG+2]) / 10.0;
        }else{
            err = ESP_ERR_INVALID_VERSION; 
            ESP_LOGE(TAG, "Wrong sensor ID: %02x\n", data->buffer[RADSENS_REG_ID]);
        }
    }

    I2C_DEV_GIVE_MUTEX(&i2c_dev);
    //ESP_LOGI(TAG, "Give mutex");

    return err;
}

esp_err_t radsens_read_data(uint32_t* pulse, float* intensy_static, float* intensy_dynamic){
    
    esp_err_t err = radsens_read(radsens_data);
    if(err == ESP_OK){
        if(radsens_data->buffer[0] == 0x7D && radsens_data->buffer[1] == 0x02){
            if(radsens_data->intensy_static == 0.0 || radsens_data->intensy_dynamic == 0.0) {
               return ESP_FAIL;
            }
            if(abs(radsens_data->intensy_static - prvious_static) > deviation || 
               abs(radsens_data->intensy_dynamic - prvious_dynamic) > deviation) {
               prvious_static = radsens_data->intensy_static;
               prvious_dynamic = radsens_data->intensy_dynamic;
               return ESP_FAIL;
            }
            prvious_static = radsens_data->intensy_static;
            prvious_dynamic = radsens_data->intensy_dynamic;
            *intensy_static = radsens_data->intensy_static;
            *intensy_dynamic = radsens_data->intensy_dynamic;
            *pulse = radsens_data->pulse_cnt;
        }else{
            return ESP_FAIL;
        }
    }
    return err;
}

esp_err_t radsens_get_hv_generator_state(bool* state){
   
    esp_err_t err = radsens_read(radsens_data);
    if(err == ESP_OK){
        *state = (radsens_data->buffer[RADSENS_HV_GENERATOR_REG] == 1);
    }
    return err;
}

esp_err_t radsens_get_sensitivity(uint8_t* sensitivity){
   
    esp_err_t err = radsens_read(radsens_data);
    if(err == ESP_OK){
        *sensitivity = radsens_data->buffer[RADSENS_SENSITYVE_REG];
    }
    return err;
}

esp_err_t radsens_set_hv_generator(bool state){
   
    uint8_t data = (state) ? 1 : 0;    
    esp_err_t err = write_register8(&i2c_dev, RADSENS_HV_GENERATOR_REG, data);
    if(err == ESP_OK) radsens_data->pulse_cnt = 0;
    return err;
}

esp_err_t radsens_set_sensitivity(uint8_t sensitivity){   
   
    esp_err_t err = write_register8(&i2c_dev, RADSENS_SENSITYVE_REG, sensitivity);
    if(err == ESP_OK) current_sensitivity = sensitivity;
    return err;
}

bool radsens_initialized(){
    return (radsens_data && radsens_data->buffer[RADSENS_REG_ID] == RADSENS_ID);
}

uint8_t radsens_get_current_sensitivity(){
    return current_sensitivity;
}