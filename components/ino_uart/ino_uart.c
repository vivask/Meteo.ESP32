#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "ino_uart.h"

#define SERIAL_SIZE_RX 130

static const char *TAG = "INO";

static meashure_6814_t meashure_result[3];

static SemaphoreHandle_t _lock_data_ = NULL;

#define SEMAPHORE_TAKE(a, b) do { \
    if (!xSemaphoreTake(a, b /*portMAX_DELAY*/)) \
    { \
         ESP_LOGE(TAG, "Could not take mutex"); \
         return ESP_ERR_TIMEOUT; \
    } \
}while (0)

#define SEMAPHORE_GIVE(a) do { \
    if (!xSemaphoreGive(a)) \
    { \
         ESP_LOGE(TAG, "Could not give mutex"); \
         return ESP_FAIL; \
    } \
}while (0)


static void rx_arduino_task()
{
    uint8_t* data = (uint8_t*) malloc(SERIAL_SIZE_RX+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, SERIAL_SIZE_RX, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            if(rxBytes == sizeof(meashure_result)){
                SEMAPHORE_TAKE(_lock_data_, portMAX_DELAY);
                memcpy(meashure_result, data, sizeof(meashure_result));
                //ESP_LOG_BUFFER_HEXDUMP(TAG, meashure_result, sizeof(meashure_result), ESP_LOG_INFO);
                ESP_LOGI(TAG, "CO: %f, NH3: %f, NO2: %f", 
                    meashure_result[CO].meashure, 
                    meashure_result[NH3].meashure, 
                    meashure_result[NO2].meashure);
                SEMAPHORE_GIVE(_lock_data_);
            }
        }
    }
    free(data);
}

esp_err_t init_arduino_uart(gpio_num_t txd_pin, gpio_num_t rxd_pin) 
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    esp_err_t err = uart_param_config(UART_NUM_2, &uart_config);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Uart config error: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_set_pin(UART_NUM_2, txd_pin, rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Uart set pin error: %s", esp_err_to_name(err));
        return err;
    }
    // We won't use a buffer for sending data.
    err = uart_driver_install(UART_NUM_2, SERIAL_SIZE_RX, 0, 0, NULL, 0);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Uart driver install error: %s", esp_err_to_name(err));
        return err;
    }

    _lock_data_ = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(rx_arduino_task, "rx_arduino_task", configMINIMAL_STACK_SIZE*8, NULL, 10, NULL, 0);

    return ESP_OK;
}

void get_6814_meashure(meashure_6814_t* nh3, meashure_6814_t* co, meashure_6814_t* no2){
    SEMAPHORE_TAKE(_lock_data_, portMAX_DELAY);   
    memcpy(co, &meashure_result[CO], sizeof(meashure_6814_t));
    memcpy(nh3, &meashure_result[NH3], sizeof(meashure_6814_t));
    memcpy(no2, &meashure_result[NO2], sizeof(meashure_6814_t));
    SEMAPHORE_GIVE(_lock_data_); 
}