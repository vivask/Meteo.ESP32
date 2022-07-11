#include <string.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "httpclient.h"
#include "i2cdev.h"
#include "radsens.h"
#include "bmp280.h"
#include "ds18x20.h"
#include "ze08ch2o.h"
//#include "mics6814.h"

#include "wifi_manager.h"

static const char *TAG = "MAIN";

#define DELAY_TIME_BETWEEN_ITEMS_MS 5000

#define I2C_SCL_IO              GPIO_NUM_22               
#define I2C_SDA_IO              GPIO_NUM_23               
#define DS18B20_GPIO            GPIO_NUM_17
#define ZE08CH2O_GPIO           GPIO_NUM_0

#define MICS6814_GPIO_NO2       32 
#define MICS6814_GPIO_NH3       35
#define MICS6814_GPIO_CO        34

peripheral_state_t _peripheral;

//mics6814 m6814;
static bmp280_t bmp280_dev;
static ds18x20_addr_t ds18x20_addrs;
static char DS18B20_ADDRESS[32] = {0};

static TaskHandle_t xTaskPool = NULL;

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
        FLASH_LOGE("%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));    
    }
}

static void pool_bmp280() {

    float pressure, temperature, humidity;
    esp_err_t err = bmp280_read_float(&bmp280_dev, &temperature, &pressure, &humidity);
    //device_set_state(&_peripheral.bmx280, err);        
    if (err != ESP_OK){
        FLASH_LOGE("%s: %s: No ack, sensor not connected...skip...",
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
/*
static void pool_mics6814(void* arg) {
    
    uint32_t task_idx= (uint32_t)arg;  

    int cnt =0;
    while(1){
        ESP_LOGD(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);

        xSemaphoreTake(mics6814_mux, portMAX_DELAY);
        uint16_t raw;
        uint32_t voltage;
        //ESP_LOGI(TAG, "NO2: %d/%d = %f => %fppm", m6814.getResistance(CH_NO2), m6814.getBaseResistance(CH_NO2), m6814.getCurrentRatio(CH_NO2), m6814.measure(NO2));
        //ESP_LOGI(TAG, "CO: %d/%d = %f => %fppm", m6814.getResistance(CH_CO), m6814.getBaseResistance(CH_CO), m6814.getCurrentRatio(CH_CO), m6814.measure(CO));
        //ESP_LOGI(TAG, "NH3: %d/%d = %f => %fppm", m6814.getResistance(CH_NH3), m6814.getBaseResistance(CH_NH3), m6814.getCurrentRatio(CH_NH3), m6814.measure(NH3));
        m6814.readRawNH3(raw, voltage);
        ESP_LOGI(TAG, "NH3 raw: %d, voltage = %d", raw, voltage);
        m6814.readRawNO2(raw, voltage);
        ESP_LOGI(TAG, "NO2 raw: %d, voltage = %d", raw, voltage);
        m6814.readRawCO(raw, voltage);
        ESP_LOGI(TAG, "CO raw: %d, voltage = %d", raw, voltage);
        xSemaphoreGive(mics6814_mux);

        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
    }
    vSemaphoreDelete(mics6814_mux);
    vTaskDelete(NULL);
}
*/

static void task_pool(void* arg){
    while(1){
        if(_peripheral.radsens == ON){
            pool_radsens();
        }
        if(_peripheral.bmx280 == ON){
            pool_bmp280();
        }
        if(_peripheral.ds18b20 == ON){
            pool_ds18b20();
        }
        if(_peripheral.ze08ch2o == ON){
            pool_ze08ch2o();
        }

        vTaskDelay(DELAY_TIME_BETWEEN_ITEMS_MS / portTICK_RATE_MS);
    }
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
/*
    ret = m6814.mics6814_init(MICS6814_GPIO_NO2, MICS6814_GPIO_NH3, MICS6814_GPIO_CO);
    device_set_state(&_peripheral.mics6814, ret);
    if(ret != ESP_OK){
        https_logw("%s: MICS6814 initialisation fail", esp_err_to_name(ret));
    }else{
        https_logi("MICS6814 initialisation success");
    }
*/
}

/*
static void peripheral_calibration(){
    if(_peripheral.mics6814 == ON){
        if(m6814.calibrate()){
            _peripheral.mics6814 = CALIBRATED;    
        }
    }
}
*/

/*
static void mics6814_start(){
    if(_peripheral.mics6814 == CALIBRATED){
        xTaskCreate(&pool_mics6814, "pool_mics6814", configMINIMAL_STACK_SIZE*8, NULL, 10, &xMICS6814);
    }
}

static void mics6814_stop(){
    if(xMICS6814){
        vTaskDelete( xMICS6814 );
        xMICS6814 = NULL;
    }
}
*/

static void peripheral_start(){
    xTaskCreate(&task_pool, "task_pool", configMINIMAL_STACK_SIZE*8, NULL, 10, &xTaskPool);
}

static void peripheral_stop(){
    if(xTaskPool){
        vTaskDelete( xTaskPool );
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

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());   
        ret = nvs_flash_init(); 
    }
    ESP_ERROR_CHECK(ret);

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
}