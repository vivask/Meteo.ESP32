/**
 * @file valve.c
 *
 * ESP-IDF driver for salenoid Valve control
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <string.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "valve.h"

static const char *TAG = "valve";

static gpio_config_t* valve_config = NULL;
static gpio_num_t valve_gpio;

static int32_t get_gpip_sel(gpio_num_t gpio){

    int32_t sel = -1;

    switch (gpio)
    {
    case GPIO_NUM_0:
        sel = GPIO_SEL_0;
        break;    
    case GPIO_NUM_1:
        sel = GPIO_SEL_1;
        break;    
    case GPIO_NUM_2:
        sel = GPIO_SEL_2;
        break;    
    case GPIO_NUM_3:
        sel = GPIO_SEL_3;
        break;    
    case GPIO_NUM_4:
        sel = GPIO_SEL_4;
        break;    
    case GPIO_NUM_5:
        sel = GPIO_SEL_5;
        break;    
    case GPIO_NUM_6:
        sel = GPIO_SEL_6;
        break;    
    case GPIO_NUM_7:
        sel = GPIO_SEL_7;
        break;    
    case GPIO_NUM_8:
        sel = GPIO_SEL_8;
        break;    
    case GPIO_NUM_9:
        sel = GPIO_SEL_9;
        break;    
    case GPIO_NUM_10:
        sel = GPIO_SEL_10;
        break;    
    case GPIO_NUM_11:
        sel = GPIO_SEL_11;
        break;    
    case GPIO_NUM_12:
        sel = GPIO_SEL_12;
        break;    
    case GPIO_NUM_13:
        sel = GPIO_SEL_13;
        break;    
    case GPIO_NUM_14:
        sel = GPIO_SEL_14;
        break;    
    case GPIO_NUM_15:
        sel = GPIO_SEL_15;
        break;    
    case GPIO_NUM_16:
        sel = GPIO_SEL_16;
        break;    
    case GPIO_NUM_17:
        sel = GPIO_SEL_17;
        break;    
    case GPIO_NUM_18:
        sel = GPIO_SEL_18;
        break;    
    case GPIO_NUM_19:
        sel = GPIO_SEL_19;
        break;    
    case GPIO_NUM_20:
        sel = GPIO_SEL_20;
        break;    
    case GPIO_NUM_21:
        sel = GPIO_SEL_21;
        break;    
    case GPIO_NUM_22:
        sel = GPIO_SEL_22;
        break;    
    case GPIO_NUM_23:
        sel = GPIO_SEL_23;
        break;    
    case GPIO_NUM_25:
        sel = GPIO_SEL_25;
        break;    
    case GPIO_NUM_26:
        sel = GPIO_SEL_26;
        break;    
    case GPIO_NUM_27:
        sel = GPIO_SEL_27;
        break;    
    case GPIO_NUM_28:
        sel = GPIO_SEL_28;
        break;    
    case GPIO_NUM_29:
        sel = GPIO_SEL_29;
        break;    
    case GPIO_NUM_30:
        sel = GPIO_SEL_30;
        break;    
    case GPIO_NUM_31:
        sel = GPIO_SEL_31;
        break;    
    case GPIO_NUM_32:
        sel = GPIO_SEL_32;
        break;    
    case GPIO_NUM_33:
        sel = GPIO_SEL_33;
        break;    
    case GPIO_NUM_34:
        sel = GPIO_SEL_34;
        break;    
    case GPIO_NUM_35:
        sel = GPIO_SEL_35;
        break;    
    case GPIO_NUM_36:
        sel = GPIO_SEL_36;
        break;    
    default:
        break;
    }

    return sel;
}


esp_err_t valve_init_desc(gpio_num_t gpio){

    valve_config = (gpio_config_t*)malloc(sizeof(gpio_config_t));
    memset(valve_config, 0x00, sizeof(gpio_config_t));

    valve_gpio = gpio;    
    valve_config->intr_type = GPIO_INTR_DISABLE;
    valve_config->mode = GPIO_MODE_OUTPUT;
    valve_config->pin_bit_mask = get_gpip_sel(gpio);
    return gpio_config(valve_config);
}

void valve_free_desc()
{
    free(valve_config);
    valve_config = NULL;
}

esp_err_t valve_close(){
    return gpio_set_level(valve_gpio, 1);
}

esp_err_t valve_open(){
    return gpio_set_level(valve_gpio, 0);
}

