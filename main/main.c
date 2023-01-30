#include <string.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

#include "httpclient.h"
#include "i2cdev.h"
#include "radsens.h"
#include "bmp280.h"
#include "ds18x20.h"
#include "ze08ch2o.h"
#include "ino_uart.h"
#include "button.h"

#include "wifi_manager.h"

static const char *TAG = "MAIN";

#define DELAY_TIME_BETWEEN_ITEMS_S 5
#define TWDT_TIMEOUT_S 60

#define I2C_SCL_IO              GPIO_NUM_22               
#define I2C_SDA_IO              GPIO_NUM_23               
#define DS18B20_GPIO            GPIO_NUM_21
#define ZE08CH2O_GPIO           GPIO_NUM_19
#define INO_RXD_PIN             GPIO_NUM_18
#define INO_TXD_PIN             GPIO_NUM_5
#define BUTTON_PIN              GPIO_NUM_12

peripheral_state_t _peripheral;

static bmp280_t bmp280_dev;
static ds18x20_addr_t ds18x20_addrs;
static char DS18B20_ADDRESS[32] = {0};

static TaskHandle_t xTaskPool = NULL;
static bool drop = false;

static bool _wifi_connected = false;
static bool _ap_started = false;
static bool _scan_started = false;

static void pool_radsens() {

    uint32_t pulse;
    float intensy_static, intensy_dyanmic;
        
    esp_err_t ret = radsens_read_data(&pulse, &intensy_static, &intensy_dyanmic);
    //device_set_state(&_peripheral.radsens, ret);
    if(ret == ESP_ERR_TIMEOUT){
        //FLASH_LOGD("RadSens I2C Timeout");
    } else if(ret == ESP_OK) {
        https_radsens_send_data(intensy_dyanmic, intensy_static, pulse);
    } else {
        //FLASH_LOGE("%s: No ack, radsens not connected...skip...", esp_err_to_name(ret));    
    }
}

static void pool_bmp280() {

    float pressure, temperature, humidity;
    esp_err_t err = bmp280_read_float(&bmp280_dev, &temperature, &pressure, &humidity);
    //device_set_state(&_peripheral.bmx280, err);        
    if (err != ESP_OK){
        FLASH_LOGE("%s: %s: No ack, bme280 not connected...skip...",
        (bmp280_dev.id == BME280_CHIP_ID) ? "BME280" : "BMP280", esp_err_to_name(err));
    }else {
        ESP_LOGD(TAG, "Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f ", pressure, temperature, humidity);
        https_bme280_send_data(temperature, humidity, pressure);
    }
}

static void pool_ds18b20() {
    
    float temperature;
    esp_err_t err = ds18x20_measure_and_read_multi(DS18B20_GPIO, &ds18x20_addrs, 1, &temperature);
    //device_set_state(&_peripheral.ds18b20, err);        
    if(err != ESP_OK){
        //FLASH_LOGD("DS18B20: %s: No ack, sensor not connected...skip...", esp_err_to_name(err));
    }else{
        ESP_LOGD(TAG, "Temperature: %.2f C", temperature);
        https_ds18b20_send_data(DS18B20_ADDRESS, temperature);
    }
}

static void pool_ze08ch2o() {
    
    uint16_t ppm = ze08ch2o_read();
    if(ppm == ZE08CH2O_READ_ERROR_CODE){
        FLASH_LOGE(TAG, "ZE08CH2O reading failed");    
    }else{
        ESP_LOGD(TAG, "CH2O: %d", ppm);
        https_ze08_send_data(ppm);
    }
}

static void pool_mics6814() {
    
    meashure_6814_t nh3, co, no2;
    get_6814_meashure(&nh3, &co, &no2);
    if( nh3.meashure == 0.0 && co.meashure == 0.0 && no2.meashure == 0.0 ){
        return;
    }
    https_6814_send_data(nh3.meashure, co.meashure, no2.meashure);
    //https_logi("CO: %f, NO2: %f, NH3: %f", co.meashure, no2.meashure, nh3.meashure);
}

static void task_pool(void* arg){

    if(!isSntpOk()){
        https_loge("SNTP initialisation fail, peripheral operation is not possible");    
        https_logi("Rebooting ......");
        esp_restart();
    }

    if(esp_task_wdt_add(NULL) != ESP_OK && esp_task_wdt_status(NULL) != ESP_OK){
        ESP_LOGE(TAG, "Task pool erorr add wdt");
        vTaskDelete(NULL);
        return;
    }

    while(!drop){
        if(_peripheral.radsens == ON && !drop){
            pool_radsens();
        }
        if(_peripheral.bmx280 == ON && !drop){
            pool_bmp280();
        }
        if(_peripheral.ds18b20 == ON && !drop){
            pool_ds18b20();
        }
        if(_peripheral.ze08ch2o == ON && !drop){
            pool_ze08ch2o();
        }
        if(_peripheral.mics6814 == ON && !drop){
            pool_mics6814();
        }
        if(esp_task_wdt_reset() != ESP_OK){
            ESP_LOGE(TAG, "Task pool erorr wdt reset");
            break;
        }
        vTaskDelay(1000*DELAY_TIME_BETWEEN_ITEMS_S / portTICK_RATE_MS);
    }
    ESP_LOGW(TAG, "Peripheral task droped!");
    vTaskDelete(NULL);
}

static void peripheral_initialization(){
    esp_err_t ret;
    memset(&_peripheral, 0x00, sizeof(peripheral_state_t));

    size_t ds18x20_sensor_count = 0;
    gpio_set_pull_mode(DS18B20_GPIO, GPIO_PULLUP_ONLY);
    ret = ds18x20_scan_devices(DS18B20_GPIO, &ds18x20_addrs, 1, &ds18x20_sensor_count);

    ret = (ret != ESP_OK || ds18x20_addrs == 0) ? ESP_FAIL : ret;
    device_set_state(&_peripheral.ds18b20, ret);

    if(ret != ESP_OK){
        https_logw("%s: DS18B20 initialisation fail", esp_err_to_name(ret));
    }else{
        sprintf(DS18B20_ADDRESS, "%08X%08X", (uint32_t)(ds18x20_addrs >> 32), (uint32_t)ds18x20_addrs);
        https_logi("DS18B20 : %s initialisation success", DS18B20_ADDRESS);
    }

    ret = i2cdev_init();
    device_set_state(&_peripheral.i2c, ret);

    radsens_params_t radsens_params;
    radsens_init_default_params(&radsens_params);

    ret = (radsens_init_desc(0, I2C_SDA_IO, I2C_SCL_IO) == ESP_OK && 
           radsens_init(&radsens_params) == ESP_OK) ? ESP_OK : ESP_FAIL;

    device_set_state(&_peripheral.radsens, ret);
    if(ret != ESP_OK){
        https_logw("%s: RadSens initialisation fail", esp_err_to_name(ret));
    }else{
        https_logi("Radsens initialisation success");
    }

    bmp280_params_t bmp20_params;
    bmp280_init_default_params(&bmp20_params);
    memset(&bmp280_dev, 0, sizeof(bmp280_t));

    ret = (bmp280_init_desc(&bmp280_dev, BMP280_I2C_ADDRESS_0, 0, I2C_SDA_IO, I2C_SCL_IO) == ESP_OK &&
           bmp280_init(&bmp280_dev, &bmp20_params) == ESP_OK) ? ESP_OK : ESP_FAIL;

    bool bme280p = bmp280_dev.id == BME280_CHIP_ID;
    device_set_state(&_peripheral.bmx280, ret);
    if(ret != ESP_OK){
        https_logw("%s: %s initialisation fail", esp_err_to_name(ret), bme280p ? "BME280" : "BMP280");
    }else{
        https_logi("%s initialisation success", bme280p ? "BME280" : "BMP280");
    }

    ret = ze08ch2o_init(ZE08CH2O_GPIO);
    device_set_state(&_peripheral.ze08ch2o, ret);
    if(ret != ESP_OK){
        https_logw("%s: ZE08CH2O initialisation fail", esp_err_to_name(ret));
    }else{
        https_logi("ZE08CH2O initialisation success");
    }

    ret = init_arduino_uart(INO_TXD_PIN, INO_RXD_PIN);
    device_set_state(&_peripheral.mics6814, ret);
    if (ret != ESP_OK) {
        https_logw("%s: MICS6814 initialisation fail", esp_err_to_name(ret));
    }else{
        https_logi("MICS6814 initialisation success");
    }
}


static void peripheral_start(){
    xTaskCreatePinnedToCore(task_pool, "task_pool", configMINIMAL_STACK_SIZE*8, NULL, 10, &xTaskPool, 0);
    //xTaskCreate(&task_pool, "task_pool", configMINIMAL_STACK_SIZE*8, NULL, 10, &xTaskPool);
}

static void peripheral_stop(){
    if(xTaskPool){
        vTaskDelete( xTaskPool );
        if(esp_task_wdt_delete(xTaskPool) != ESP_OK && esp_task_wdt_status(xTaskPool) != ESP_ERR_NOT_FOUND){
            FLASH_LOGE("Wdt pool task delete error");
        }
        if(esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0)) != ESP_OK && esp_task_wdt_status(xTaskGetIdleTaskHandleForCPU(0)) != ESP_ERR_NOT_FOUND){
            FLASH_LOGE("Wdt pool task delete error");
        }
        xTaskPool = NULL;
    }
}

static void cb_https_client_start(void *pvParameter){
    ESP_LOGI(TAG, "HTTP Client Started");
    peripheral_initialization();
    //peripheral_calibration();
    peripheral_start();    
}

static void cb_https_client_stop(void *pvParameter){
    ESP_LOGW(TAG, "HTTP Client Stoped");
    peripheral_stop();
}

void cb_disconnection(void *pvParameter){
    _wifi_connected = false;
	ESP_LOGW(TAG, "WIFI Connection lost");
    https_client_stop();
}

void cb_connection_ok(void *pvParameter){
    _wifi_connected = true;
    if(_ap_started == false && _scan_started == false){
        https_client_init(wifi_manager_get_httpd_config()->address, wifi_manager_get_httpd_config()->port);
        https_client_start();
    }
}

void cb_stop_ap(void *pvParameter){
    _ap_started = false;  
    if(_wifi_connected == true && _scan_started == false){
        https_client_init(wifi_manager_get_httpd_config()->address, wifi_manager_get_httpd_config()->port);
        https_client_start();
    }
}

void cb_start_ap(void *pvParameter){
    _ap_started = true;    
}

void cb_scan_stop(void *pvParameter){
    _scan_started = false;    
    if(_wifi_connected == true && _ap_started == false){
        https_client_init(wifi_manager_get_httpd_config()->address, wifi_manager_get_httpd_config()->port);
        https_client_start();
    }
}

void cb_scan_start(void *pvParameter){
    _scan_started = true;    
}

void cb_start_peripheral(void *pvParameter){
    ESP_LOGW(TAG, "PERIPHERAL START");
    peripheral_start();
}

void cb_stop_peripheral(void *pvParameter){
    ESP_LOGW(TAG, "PERIPHERAL STOP");
    peripheral_stop();
}

void cb_drop_peripheral(void *pvParameter){
    drop = true;
    if(esp_task_wdt_delete(xTaskPool) != ESP_OK && esp_task_wdt_status(xTaskPool) != ESP_ERR_NOT_FOUND){
        FLASH_LOGE("Wdt pool task delete error");
    }
    if(esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0)) != ESP_OK && esp_task_wdt_status(xTaskGetIdleTaskHandleForCPU(0)) != ESP_ERR_NOT_FOUND){
        FLASH_LOGE("Wdt pool task delete error");
    }
}

void cb_push_button(int count) {
    https_logw("Button pressed %d times", count);

    if (count == 2) {
        https_logi("Rebooting...");
        esp_restart();
    }

    if (count == 5) {
        https_logi("Setup mode runing..."); 
        wifi_manager_start_setup_mode();
    }
}

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());   
        ret = nvs_flash_init(); 
    }
    ESP_ERROR_CHECK(ret);

    if(esp_task_wdt_init(TWDT_TIMEOUT_S, false) != ESP_OK){
        ESP_LOGE(TAG, "Wdt init false");
        return;
    }

#ifndef CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
    esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0));
#endif
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 && !CONFIG_FREERTOS_UNICORE
    esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(1));
#endif

    wifi_manager_start(false);

    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_connection_ok);
    wifi_manager_set_callback(WM_EVENT_STA_DISCONNECTED, &cb_disconnection);
    wifi_manager_set_callback(WM_ORDER_STOP_AP, &cb_stop_ap);
    wifi_manager_set_callback(WM_ORDER_START_AP, &cb_start_ap);
    wifi_manager_set_callback(WM_EVENT_SCAN_DONE, &cb_scan_stop);
    wifi_manager_set_callback(WM_ORDER_START_WIFI_SCAN, &cb_scan_start);

    https_client_set_start_callback(&cb_https_client_start);
    https_client_set_stop_callback(&cb_https_client_stop);
    https_client_set_stop_peripheral(&cb_stop_peripheral);
    https_client_set_start_peripheral(&cb_stop_peripheral);
    https_client_set_drop_peripheral(&cb_drop_peripheral);

    button_init(BUTTON_PIN, cb_push_button);

}