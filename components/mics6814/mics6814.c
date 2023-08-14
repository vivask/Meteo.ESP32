#include <stdio.h>
// #include <stdint.h>
// #include <stddef.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/spi_slave.h>
#include <driver/gpio.h>

#include "mics6814.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST    HSPI_HOST
#else
#define RCV_HOST    SPI2_HOST
#endif

#define QUEUE_SIZE              (8)

#define DELAY_SPI_REQUEST_MS    500

static const char *TAG = "mics6814";

static mics6814_data_t data = { 0 };

static gpio_num_t _gpio_cs = -1;

static void pool_spi_task(void* pvParameters) {

    WORD_ALIGNED_ATTR char recvbuf[sizeof(mics6814_data_t)] = "";

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = sizeof(mics6814_data_t)*QUEUE_SIZE;
    t.tx_buffer= NULL;
    t.rx_buffer= recvbuf;

    for (;;) {
        if (gpio_get_level(_gpio_cs) == 0) {
            esp_err_t ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
            if (ret == ESP_OK){
                memcpy(&data, (void*)recvbuf, sizeof(mics6814_data_t));
                // ESP_LOGW(TAG, "[MICS6814] CO: %f, NO2: %f, NH3: %f, STATUS: %d", data.co, data.no2, data.nh3, data.status);
            }else {
                ESP_LOGE(TAG, "SPI read error: %s", esp_err_to_name(ret));
            }
        }
        vTaskDelay(DELAY_SPI_REQUEST_MS / portTICK_RATE_MS);
    }

	ESP_LOGI(TAG, "POOL SPI TASK STOPPED");
  
	vTaskDelete( NULL );        
}

esp_err_t mics6814_init(gpio_num_t miso, gpio_num_t sclk, gpio_num_t cs) {
    
    //Configuration for the SPI bus
    spi_bus_config_t bus_cfg={
        .mosi_io_num=miso,
        .sclk_io_num=sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 50000,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slv_cfg={
        .mode=0,
        .spics_io_num=cs,
        .queue_size=QUEUE_SIZE,
        .flags=0,
    };

    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(miso, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(sclk, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(cs, GPIO_PULLUP_ONLY);

    //Check master status
    _gpio_cs = cs;
    if (gpio_get_level(_gpio_cs) != 0) {
        return ESP_FAIL;
    }

    esp_err_t err = spi_slave_initialize(RCV_HOST, &bus_cfg, &slv_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        return err;
    }

    xTaskCreate(&pool_spi_task, "pool_spi_task", 0x800, NULL, CONFIG_WIFI_MANAGER_TASK_PRIORITY-2, NULL);

    return ESP_OK;
}

esp_err_t get_mics6814_data(void* comp_data) {
    memcpy(comp_data, &data, sizeof(mics6814_data_t));
    return ESP_OK;
}