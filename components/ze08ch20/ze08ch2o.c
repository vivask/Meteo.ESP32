/**
 * @file ze08ch2o.c
 *
 * ESP-IDF driver for ZE08CH2O digital phenols sensor
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <string.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <driver/uart.h>
#include "ze08ch2o.h"

static const char *TAG = "ze08ch2o";

#define UART_TIMEOUT                                    (50)
#define UART_SPEED                                      (9600)
#define UART_NUM                                        (UART_NUM_1)
#define ZE08CH2O_UART_DEFAULT_READ_TIMEOUT              (2500UL)
#define ZE08CH2O_UART_START_BYTE                        (0xFF)
#define ZE08CH2O_UART_GAS_NAME                          (0x17)
#define ZE08CH2O_UART_GAS_UNIT                          (0x04)
#define RX_BUF_SIZE                                     (0x400)

#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)


esp_err_t ze08ch2o_init(gpio_num_t rxd_gpio) 
{
    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config)); 
    uart_config.baud_rate = UART_SPEED;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    CHECK_LOGE(uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0), "Error driver install");
    CHECK_LOGE(uart_param_config(UART_NUM, &uart_config), "Configuration fail");
    CHECK_LOGE(uart_set_pin(UART_NUM, rxd_gpio, rxd_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), "Set pins fail");

    vTaskDelay(pdMS_TO_TICKS(1000));

    size_t length = 0;
    CHECK_LOGE(uart_get_buffered_data_len(UART_NUM, (size_t*)&length), "Data length read fail");
    esp_err_t err = (length == 0) ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
    if(err != ESP_OK){
        ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(err));
    }
    return err;
}

inline static uint32_t mills()
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static bool available()
{
    size_t length = 0;
    CHECK_LOGE(uart_get_buffered_data_len(UART_NUM, (size_t*)&length), "Data length read fail");
    return (length != 0);
}

static uint8_t ze08ch2o_get_data()
{
    uint8_t data = 0;
    uart_read_bytes(UART_NUM, &data, 1, UART_TIMEOUT / portTICK_RATE_MS);
    return data;
}


uint16_t ze08ch2o_read() {

    uint8_t headerDetected = false;
    uint8_t writePos = 0x00;
    uint8_t checkSum = 0x00;
    uint16_t rc = ZE08CH2O_READ_ERROR_CODE;
    uint32_t readStartTime;

    ze08ch2o_data_t ze08ch2o_data = {0};
    uint8_t* ptrRawBuffer = (uint8_t*) &ze08ch2o_data;
	 
    readStartTime =  mills();
    ESP_LOGD(TAG, "Data size: %d", sizeof(ze08ch2o_data));
    while ((sizeof(ze08ch2o_data) > writePos) && (ZE08CH2O_UART_DEFAULT_READ_TIMEOUT >  mills() - readStartTime)) {
        if (!available()) {
          continue;
        }
        ptrRawBuffer[writePos] = ze08ch2o_get_data();
        if (!headerDetected) {
            headerDetected = (ZE08CH2O_UART_START_BYTE == ptrRawBuffer[0x00]);
        } else {
            writePos++;
        }   
    }
	// Reading is not finished sucessfully: not all bytes recieved or wrong Gas ID / Unit ID contained in the packet
    if (writePos < sizeof(ze08ch2o_data) ||
        ZE08CH2O_UART_GAS_NAME != ze08ch2o_data.gasName ||
        ZE08CH2O_UART_GAS_UNIT != ze08ch2o_data.gasUnit) {
        goto finish;
    }
    // Calculate checksum.
    //Start byte & recieved checksum is not taken in account. The first one is dropped in the read procedure and the second one just will skipped in calculation
    for (uint8_t i = 0x00; i < sizeof(ze08ch2o_data) - 1; i++) {
        checkSum += ptrRawBuffer[i];
        //printf(" %0X", ptrRawBuffer[i]);
    }
    //printf("\n");
    checkSum = (~checkSum) + 1;
    ESP_LOGD(TAG, "Recieved checksum: %0X", ze08ch2o_data.checkSum);
    ESP_LOGD(TAG, "Calculated checksum: %0X", checkSum);
    if (checkSum == ze08ch2o_data.checkSum) {
        ESP_LOGD(TAG, "Full range (ppb): %d", (ze08ch2o_data.fullRangeHighByte << 0x08) + ze08ch2o_data.fullRangeLowByte);
        rc = (ze08ch2o_data.concentrationHighByte << 0x08) + ze08ch2o_data.concentrationLowByte;
    }
	 
finish:
    return rc;
}