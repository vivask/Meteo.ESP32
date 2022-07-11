#include <string.h>
#include <stdarg.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <esp_https_ota.h>
#include <esp_ota_ops.h>
#include <esp_http_client.h>
#include <cJSON.h>

#include "wifi_manager.h"
#include "httpclient.h"
#include "radsens.h"

#define ROOT_DIR                            CONFIG_ROOT_DIR
#define LOAD_SETTING_SCRIPTNAME             CONFIG_LOAD_SETTING_SCRIPTNAME
#define COMMAND_SCRIPTNAME                  CONFIG_COMMAND_SCRIPTNAME
#define COMMAND_UPGRADE_SUCCESS             CONFIG_COMMAND_UPGRADE_SUCCESS
#define COMMAND_UPGRADE_FAIL                CONFIG_COMMAND_UPGRADE_FAIL
#define FIRMWARE_DIR                        CONFIG_FIRMWARE_DIR
#define FIRMWARE_EMPTY                      CONFIG_FIRMWARE_EMPTY

#define BASE_URL_MAX_SIZE 64
#define URL_MAX_SIZE 256
#define MESSAGE_MAX_LEN 128

#define DELAY_TIME_BETWEEN_ITEMS_MS  5000
#define OTA_RECV_TIMEOUT 5000

#define WATCHDOG_TIMER 1000
#define WATCHDOG_RECIVE_RETRY 50
#define WATCHDOG_SEND_RETRY 50
#define RECONNECT_RETRY 5

#define ON  1
#define OFF 2

typedef struct {
    uint8_t _valve_state;
    uint8_t _ccs811_baseline;
    float _min_temp;
    float _max_temp;
    uint8_t _valve_disable;
    uint8_t _setup_mode;
    uint8_t _restart;
    uint8_t _radsens_hv_mode;
    uint8_t _radsens_hv_state;
    uint8_t _radsens_sensitivity;
    char _firmware[64];
    uint8_t _clear_journal;
} Peripheral;

const static char* TAG = "HTTPC";

static const uint16_t FLASH_LOG_READ_PERIOD = 1; // 1 = 1 minute
static char _root_url[BASE_URL_MAX_SIZE] = {0};
static char _firmware_url[BASE_URL_MAX_SIZE] = {0};
static char _mac_address[18] = {0};
static Peripheral peripheral = {0};
static bool _upgrade_during = false;
static bool _http_post_stoped = false;
static TaskHandle_t xHttpsClientActive = NULL;
static TaskHandle_t xFlashLogActive = NULL;
static TaskHandle_t xUpgradeFirmware = NULL;
static bool _ap_started = false;
static bool _https_client_active = false;
static bool _httpd_not_responding = false;

static uint32_t _watchdog_receive_counter = 0;
static uint32_t _watchdog_send_counter = 0;
static uint32_t _watchdog_reconnect_counter = 0;
static TaskHandle_t xWatchdog = NULL;

static SemaphoreHandle_t _lock_http_socket = NULL;
static esp_http_client_handle_t _client = NULL;

void (*cb_ptr_start)(void*) = NULL;
void (*cb_ptr_stop)(void*) = NULL;
void (*cb_ptr_stop_peripheral)(void*) = NULL;
void (*cb_ptr_start_peripheral)(void*) = NULL;

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

void https_client_set_start_callback(void (*func_ptr)(void*) ){
    cb_ptr_start = func_ptr;    
}

void https_client_set_stop_callback(void (*func_ptr)(void*) ){
    cb_ptr_stop = func_ptr;    
}

void https_client_set_stop_peripheral(void (*func_ptr)(void*) ){
    cb_ptr_stop_peripheral = func_ptr;    
}

void https_client_set_start_peripheral(void (*func_ptr)(void*) ){
    cb_ptr_start_peripheral = func_ptr;    
}

static void date_time_format(char* date_time, time_t t){
    time_t now = (t==0) ? get_local_datetime() : t;
	struct tm ti = {0};	
    localtime_r(&now, &ti);
    sprintf(date_time, "20%02d-%02d-%02d %02d:%02d:%02d", ti.tm_year - 100, ti.tm_mon + 1,
           ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec);
}

static void _watchdog(void* arg){

    while(1){
        _watchdog_receive_counter++;
        _watchdog_send_counter++;
        if(_watchdog_receive_counter > WATCHDOG_RECIVE_RETRY){
            FLASH_LOGW("Receive watchdog restarting...");
            esp_restart();
        }
        if(_watchdog_send_counter > WATCHDOG_SEND_RETRY){
            FLASH_LOGW("Send watchdog restarting...");
            esp_restart();
        }
        if(_watchdog_reconnect_counter > RECONNECT_RETRY){
            FLASH_LOGW("Reconnect watchdog restarting...");
            esp_restart();
        }
        vTaskDelay(WATCHDOG_TIMER / portTICK_RATE_MS);
    }
}

void watchdog_start(){

    xTaskCreate(&_watchdog, "_watchdog", configMINIMAL_STACK_SIZE*4, NULL, 10, &xWatchdog);
}


void watchdog_stop(){

    if(xWatchdog){
        vTaskDelete( xWatchdog );
        xWatchdog = NULL;
    }
}

void peripheral_start(){
    if(cb_ptr_start_peripheral) (*cb_ptr_start_peripheral)(NULL);
}

void peripheral_stop(){
    if(cb_ptr_stop_peripheral) (*cb_ptr_stop_peripheral)(NULL);
}

bool https_client_active(){
    return _https_client_active;
}

static esp_err_t _http_get_config_eh(esp_http_client_event_t *evt) {

    if(evt->event_id == HTTP_EVENT_ON_DATA){
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        int http_code = esp_http_client_get_status_code(evt->client);
        if (http_code > 0 && (http_code == HttpStatus_Ok || http_code == HttpStatus_MovedPermanently)) {
            char* buffer = (char*)malloc(evt->data_len+1);
            memcpy(buffer, evt->data, evt->data_len);
            buffer[evt->data_len] = '\0';
            ESP_LOGD(TAG, "Request: %s", buffer);
            cJSON *json = cJSON_Parse(buffer);
            if (cJSON_HasObjectItem(json, "valve_state")){
                peripheral._valve_state = cJSON_GetObjectItem(json,"valve_state")->valueint;
                peripheral._ccs811_baseline  = cJSON_GetObjectItem(json,"ccs811_baseline")->valueint;
                peripheral._min_temp  = cJSON_GetObjectItem(json,"min_temp")->valuedouble;
                peripheral._max_temp  = cJSON_GetObjectItem(json,"max_temp")->valuedouble;
                peripheral._valve_disable  = cJSON_GetObjectItem(json,"valve_disable")->valueint;
                sprintf(peripheral._firmware, "%s", cJSON_GetObjectItem(json,"firmware")->valuestring);
                peripheral._setup_mode  = cJSON_GetObjectItem(json,"setup_mode")->valueint;
                peripheral._restart  = cJSON_GetObjectItem(json,"reboot")->valueint;
                peripheral._radsens_hv_mode  = cJSON_GetObjectItem(json,"radsens_hv_mode")->valueint;
                peripheral._radsens_hv_state  = cJSON_GetObjectItem(json,"radsens_hv_state")->valueint;
                peripheral._radsens_sensitivity  = cJSON_GetObjectItem(json,"radsens_sensitivity")->valueint;
                peripheral._clear_journal  = cJSON_GetObjectItem(json,"clear_journal_esp32")->valueint;
                cJSON_Delete(json);
                _watchdog_receive_counter=0;

                ESP_LOGD(TAG, "Valve state: %d", peripheral._valve_state);
                ESP_LOGD(TAG, "CCS811 baseline:: %d", peripheral._ccs811_baseline);
                ESP_LOGD(TAG, "Min temperature:: %f", peripheral._min_temp);
                ESP_LOGD(TAG, "Max temperature:: %f", peripheral._max_temp);
                ESP_LOGD(TAG, "Valve disable: %d", peripheral._valve_disable);
                ESP_LOGD(TAG, "Firmware file: %s", peripheral._firmware);
                ESP_LOGD(TAG, "Setup mode: %d", peripheral._setup_mode);
                ESP_LOGD(TAG, "Restart mode: %d", peripheral._restart);
                ESP_LOGD(TAG, "Radsens mode: %d", peripheral._radsens_hv_mode);
                ESP_LOGD(TAG, "Radsens state: %d", peripheral._radsens_hv_state);
                ESP_LOGD(TAG, "Radsens sensitivity: %d", peripheral._radsens_sensitivity);
                ESP_LOGD(TAG, "Clear journal: %d", peripheral._clear_journal);
            }
            free(buffer);
        }
    }

    return ESP_OK;
}

void https_disconnect(){
    if(_client){
        esp_http_client_cleanup(_client);
        _client = NULL;
    }
}

void https_connect(){
    https_disconnect();
	esp_http_client_config_t config = {
        .url = _root_url,
        .method = HTTP_METHOD_POST,
        .cert_pem = (char*)wifi_manager_get_httpd_config()->ca, 
        //.client_cert_pem = (char*)wifi_manager_get_httpd_config()->client_crt, 
        //.client_key_pem = (char*)wifi_manager_get_httpd_config()->client_key, 
        .username = (char*)wifi_manager_get_httpd_config()->username,
        .password = (char*)wifi_manager_get_httpd_config()->password,
        .event_handler = _http_get_config_eh
    };
    _client = esp_http_client_init(&config);
    esp_http_client_set_header(_client, "Content-Type", "application/json");
}

esp_err_t _https_post_json(const char* post_data, bool force){
    
    if(!https_client_active() && !force)
    {
        FLASH_LOGW("WiFi Connection lost");
        return ESP_FAIL;    
    }
    if(_upgrade_during){
        _http_post_stoped = true;
        return ESP_OK;
    }

    SEMAPHORE_TAKE(_lock_http_socket, portMAX_DELAY);
    ESP_LOGD(TAG, "TAKE Semaphore");
    if(!_client) https_connect();
    esp_http_client_set_post_field(_client, post_data, strlen(post_data));
    esp_err_t err = esp_http_client_perform(_client);
    if (err != ESP_OK) {
        if(_httpd_not_responding == false) {
            ESP_LOGW(TAG, "Lost coonnection");
            ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
            FLASH_LOGE("HTTP POST request failed: %s", esp_err_to_name(err));
            _httpd_not_responding = true;
            https_disconnect();
            _watchdog_reconnect_counter++;
        }
    }else{
        if(_httpd_not_responding == true){
            ESP_LOGW(TAG, "Restored coonnection");
            FLASH_LOGI("HTTP POST request restored");
            _httpd_not_responding = false;
        }
        _watchdog_send_counter = 0;
        ESP_LOGD(TAG, "Data send ok to: %s", _root_url);
        ESP_LOGD(TAG, "Data: %s", post_data);
    }

    SEMAPHORE_GIVE(_lock_http_socket);
    ESP_LOGD(TAG, "GIVE Semaphore");

    return err;
}

esp_err_t https_post_json(const char* post_data){
    return _https_post_json(post_data, false);
}

static void get_device_mac_address(char* mac)
{
    uint8_t base_mac_addr[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(base_mac_addr, ESP_MAC_WIFI_STA));
    sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X", 
            base_mac_addr[0], base_mac_addr[1], base_mac_addr[2], 
            base_mac_addr[3], base_mac_addr[4], base_mac_addr[5]);
}

static void _do_firmware_upgrade(void* arg){

    char url[URL_MAX_SIZE];

    while(1){
        if(peripheral._firmware[0] != 0 && 
            strcmp(peripheral._firmware, FIRMWARE_EMPTY) != 0){
            
            _upgrade_during = true;
            https_disconnect();

            if(_http_post_stoped){
                sprintf(url, "%s/%s", _firmware_url, peripheral._firmware);
                esp_http_client_config_t config = {
                    .url = url,
                    .cert_pem = (char*)wifi_manager_get_httpd_config()->ca,
                    .username = (char*)wifi_manager_get_httpd_config()->username,
                    .password = (char*)wifi_manager_get_httpd_config()->password,
                    .timeout_ms = OTA_RECV_TIMEOUT,
                    //.event_handler = _http_load_file_eh,
                    .keep_alive_enable = true
                };

                SEMAPHORE_TAKE(_lock_http_socket, portMAX_DELAY);

                ESP_LOGW(TAG, "%s", url);
                esp_err_t err = esp_https_ota(&config);

                SEMAPHORE_GIVE(_lock_http_socket);
                _upgrade_during = false;
                _http_post_stoped = false;

                if(err == ESP_OK){   
                    https_logi("ESP_HTTPS_OTA upgrade successful. Rebooting ...");
                    sprintf(url, "{\"ORDER\":\"UPGRADE_SUCCESS\",\"DEVICE\":\"%s\",\"FNAME\":\"%s\"}", 
                        _mac_address, peripheral._firmware);
                    https_post_json(url);
                    esp_restart();
                }

                https_loge("ESP_HTTPS_OTA upgrade fail.");
                sprintf(url, "{\"ORDER\":\"UPGRADE_FAIL\",\"DEVICE\":\"%s\",\"FNAME\":\"%s\"}",  
                    _mac_address, peripheral._firmware);
                https_post_json(url);
            }            
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

static void task_firmware_start(){
    xTaskCreate(&_do_firmware_upgrade, "_do_firmware_upgrade", configMINIMAL_STACK_SIZE*8, NULL, 10, &xUpgradeFirmware);
}

static void task_firmware_stop(){
    if(xUpgradeFirmware){
        vTaskDelete( xUpgradeFirmware );
        xUpgradeFirmware = NULL;
    }    
}

static void _do_setup_mode(){
    wifi_manager_start_setup_mode();
}

static void _send_flash_log(const time_t dt, const char type, const char* message){

    char post_data[URL_MAX_SIZE];
    char date_time[32];

    time_t sntp_time = get_sntp_time_init();
    time_t sntp_tiks = get_sntp_tiks_init();
    time_t datetime = (dt < sntp_tiks) ? (sntp_time - (sntp_tiks - dt)) : dt;    
    date_time_format(date_time, datetime);

    switch(type){
        case 'I':
            sprintf(post_data, "{\"ORDER\":\"LOGING\",\"DEVICE\":\"%s\",\"TYPE\":\"I\",\"DATE_TIME\":\"%s\",\"MESSAGE\":\"%s\"}", 
                    _mac_address, date_time, message);
            break;
        case 'W':
            sprintf(post_data, "{\"ORDER\":\"LOGING\",\"DEVICE\":\"%s\",\"TYPE\":\"W\",\"DATE_TIME\":\"%s\",\"MESSAGE\":\"%s\"}", 
                    _mac_address, date_time, message);
            break;
        case 'E':
            sprintf(post_data, "{\"ORDER\":\"LOGING\",\"DEVICE\":\"%s\",\"TYPE\":\"E\",\"DATE_TIME\":\"%s\",\"MESSAGE\":\"%s\"}", 
                    _mac_address, date_time, message);
            break;
        default:
            sprintf(post_data, "{\"ORDER\":\"LOGING\",\"DEVICE\":\"%s\",\"TYPE\":\"E\",\"DATE_TIME\":\"%s\",\"MESSAGE\":\"Unknown error type: %c\"}", 
                    _mac_address, date_time, type);
    }    

    https_post_json(post_data);
   	ESP_LOGD(TAG, "%s", post_data);
}

static void _task_read_flash(void* arg){

    while(1){

        if(https_client_active() && _httpd_not_responding == false){

            //ESP_LOGW(TAG, "Start flash log read");

            read_flash_log(_send_flash_log);

            //ESP_LOGW(TAG, "Stop flash log read");
            //vTaskDelay(2000*FLASH_LOG_READ_PERIOD / portTICK_RATE_MS);
            vTaskDelay(1000*60*FLASH_LOG_READ_PERIOD / portTICK_RATE_MS);
        }
    }

    vTaskDelete(NULL);
}


void flash_read_start(){
    xTaskCreate(&_task_read_flash, "_task_read_flash", configMINIMAL_STACK_SIZE*8, NULL, 10, &xFlashLogActive);
}

void flash_read_stop(){
    if(xFlashLogActive != NULL)
    {
        vTaskDelete( xFlashLogActive );
        xFlashLogActive = NULL;
    }    
}

static void _task_load_configure(void* arg) {
    
    char url[URL_MAX_SIZE];
    while(1){
        if(https_client_active())
        {

            char date_time[32];
            date_time_format(date_time, 0);
            sprintf(url, "{\"ORDER\":\"GET_SETTINGS\",\"DEVICE\":\"%s\",\"DATE_TIME\":\"%s\"}",  _mac_address, date_time);
            https_post_json(url);

            // Setup mode
            if(peripheral._setup_mode && !_upgrade_during){
                sprintf(url, "{\"ORDER\":\"AP_MODE_ON\",\"DEVICE\":\"%s\"}",  
                        _mac_address);
                https_post_json(url);
                ESP_LOGI(TAG, "Setup mode runing...");
                https_logi("Setup mode runing..."); 
                _do_setup_mode();
                break;                
            }
            // Restart ESP32
            if(peripheral._restart && !_upgrade_during){
                sprintf(url, "{\"ORDER\":\"REBOOTED\",\"DEVICE\":\"%s\"}",  
                        _mac_address);
                https_post_json(url);
                ESP_LOGI(TAG, "Rebooting...");
                https_logi("Rebooting...");                
                esp_restart();
            }
            // Set HV generator Radsens
            if(peripheral._radsens_hv_mode && !_upgrade_during && radsens_initialized()){
                bool state;
                esp_err_t err = radsens_get_hv_generator_state(&state);
                if(err != ESP_OK){
                    https_loge("RadSens: Error read HV generator state");
                }else{
                    bool need_state = (peripheral._radsens_hv_state == ON);
                    if(need_state ==  state){
                        https_logw("RadSens: The current status is equal to the required one.");
                    }else{                      
                        err = radsens_set_hv_generator(need_state);
                    }
                    if(err != ESP_OK){
                        https_loge("RadSens: Error set HV generator state");
                    }else{
                        err = radsens_get_hv_generator_state(&state);
                        if(!(err == ESP_OK && state == need_state)){
                            https_loge("RadSens: Can't change HV generator state");
                        }else{
                            sprintf(url, "{\"ORDER\":\"RADSENS_HV_SET\",\"DEVICE\":\"%s\",\"STATE\":\"%d\"}",  
                                    _mac_address, state);
                            https_post_json(url);
                            if(state == true)
                                https_logi("RadSens: HV generator on");
                            else
                                https_logi("RadSens: HV generator off");
                        }
                    }                        
                }
            }
            // Set sensitivity Radsens
            if(!_upgrade_during && radsens_initialized()){
                esp_err_t err;
                uint8_t sensitivity = radsens_get_current_sensitivity();
                if(peripheral._radsens_sensitivity != sensitivity){
                    err = radsens_set_sensitivity(peripheral._radsens_sensitivity);
                    if(err != ESP_OK){
                        https_loge("RadSens: Can't change sensitivity from %d to %d", 
                                    sensitivity, peripheral._radsens_sensitivity); 
                    }else{
                        sprintf(url, "{\"ORDER\":\"RADSENS_SENSITIVITY_SET\",\"DEVICE\":\"%s\",\"SENSITIVITY\":\"%d\"}",  
                                _mac_address, peripheral._radsens_sensitivity);
                        https_post_json(url);
                        https_logi("RadSens: sensitivity set to: %d", peripheral._radsens_sensitivity);
                    }
                }
            }
            // Clear journal
            if(peripheral._clear_journal && !_upgrade_during){
                //clear_flash_log();
                sprintf(url, "{\"ORDER\":\"JOURNAL_CLEARED\",\"DEVICE\":\"%s\"}",  
                        _mac_address);
                https_post_json(url);
            }
        }
        vTaskDelay(DELAY_TIME_BETWEEN_ITEMS_MS / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

static void cb_start_ap(void *pvParameter){
    _ap_started = true;
    if(!https_client_active()) https_client_stop();
}

static void cb_stop_ap(void *pvParameter){
    _ap_started = false;
    if(!https_client_active()) https_client_start();
}

void  https_client_init(const uint8_t* server_address, const uint16_t server_port) {

    if(_lock_http_socket == NULL) {
        _lock_http_socket = xSemaphoreCreateMutex();
        
        sprintf(_root_url, "https://%s:%d/%s", (char*)server_address, server_port, ROOT_DIR);
        sprintf(_firmware_url, "https://%s:%d/%s", (char*)server_address, server_port, FIRMWARE_DIR);
        ESP_LOGD(TAG, "Base url: %s", _root_url);
        get_device_mac_address(_mac_address);
        
        wifi_manager_set_callback(WM_ORDER_START_AP, &cb_start_ap);
        wifi_manager_set_callback(WM_ORDER_STOP_AP, &cb_stop_ap);
    }
}

void https_client_start(){
    if(!_ap_started){
        _https_client_active = true;
        https_logi("HTTP Client Started");
        xTaskCreate(&_task_load_configure, "task_load_configure", configMINIMAL_STACK_SIZE*8, NULL, 10, &xHttpsClientActive);
        if(cb_ptr_start) (*cb_ptr_start)(&xHttpsClientActive);
        task_firmware_start();
        flash_read_start();
        watchdog_start();
    }
}

void https_client_stop(){
    if(xHttpsClientActive != NULL){
        vTaskDelete( xHttpsClientActive );
        xHttpsClientActive = NULL;
        FLASH_LOGI("HTTP Client Stoped");
        if(cb_ptr_stop) (*cb_ptr_stop)(NULL);
    }
    task_firmware_start();
    flash_read_stop();
    watchdog_stop();
    _https_client_active = false;
}

esp_err_t https_check(const char* msg){
    char post_data[URL_MAX_SIZE];
    char date_time[32];
    date_time_format(date_time, 0);
    sprintf(post_data, "{\"ORDER\":\"LOGING\",\"DEVICE\":\"%s\",\"TYPE\":\"I\",\"DATE_TIME\":\"%s\",\"MESSAGE\":\"%s\"}", 
           _mac_address, date_time, msg);
    return _https_post_json(post_data, true);
}

esp_err_t https_logi(const char* format, ...){
    char message[MESSAGE_MAX_LEN];
    va_list arg;
    va_start(arg, format);
    vsprintf(message, format, arg);
    va_end(arg);

    char post_data[URL_MAX_SIZE];
    char date_time[32];
    date_time_format(date_time, 0);
    sprintf(post_data, "{\"ORDER\":\"LOGING\",\"DEVICE\":\"%s\",\"TYPE\":\"I\",\"DATE_TIME\":\"%s\",\"MESSAGE\":\"%s\"}", 
           _mac_address, date_time, message);
//    ESP_LOGW(TAG, "%s", post_data);
//    ESP_LOGI(TAG, "%s: %s", date_time, message);
    return https_post_json(post_data);
}

esp_err_t https_logw(const char* format, ...){
    char message[MESSAGE_MAX_LEN];
    va_list arg;
    va_start(arg, format);
    vsprintf(message, format, arg);
    va_end(arg);

    char post_data[URL_MAX_SIZE];
    char date_time[32];
    date_time_format(date_time, 0);
    sprintf(post_data, "{\"ORDER\":\"LOGING\",\"DEVICE\":\"%s\",\"TYPE\":\"W\",\"DATE_TIME\":\"%s\",\"MESSAGE\":\"%s\"}", 
           _mac_address, date_time, message);
    ESP_LOGE(TAG, "%s", message);
    return https_post_json(post_data);
}

esp_err_t https_loge(const char* format, ...){
    char message[MESSAGE_MAX_LEN];
    va_list arg;
    va_start(arg, format);
    vsprintf(message, format, arg);
    va_end(arg);

    char post_data[URL_MAX_SIZE];
    char date_time[32];
    date_time_format(date_time, 0);
    sprintf(post_data, "{\"ORDER\":\"LOGING\",\"DEVICE\":\"%s\",\"TYPE\":\"E\",\"DATE_TIME\":\"%s\",\"MESSAGE\":\"%s\"}", 
           _mac_address, date_time, message);
    ESP_LOGE(TAG, "%s", message);
    return https_post_json(post_data);
}

void https_ds18b20_send_data(const char* sensor_addr, const float tempr)
{
    char post_data[URL_MAX_SIZE];
    sprintf(post_data, "{\"ORDER\":\"DS18B20\",\"DEVICE\":\"%s\",\"SENSOR\":\"%s\",\"TEMPR\":\"%f\"}",  
            _mac_address, sensor_addr, tempr);
    https_post_json(post_data);
}

void https_bme280_send_data(const float t, const float h, const float p)
{
    char post_data[URL_MAX_SIZE];
    sprintf(post_data, "{\"ORDER\":\"BME280\",\"DEVICE\":\"%s\",\"TEMPR\":\"%f\",\"PRESS\":\"%f\",\"HUM\":\"%f\"}",  
            _mac_address, t, p, h);
    https_post_json(post_data); 
}

void https_radsens_send_data(const float ri_dynamic, const float ri_static, const uint32_t pulses)
{
    char post_data[URL_MAX_SIZE];
    sprintf(post_data, "{\"ORDER\":\"RADSENS\",\"DEVICE\":\"%s\",\"RID\":\"%f\",\"RIS\":\"%f\",\"PULSE\":\"%d\"}",  
            _mac_address, ri_dynamic, ri_static, pulses);
    https_post_json(post_data);
}

void https_ze08_send_data(const uint16_t ch2o) 
{
    char post_data[URL_MAX_SIZE];
    sprintf(post_data, "{\"ORDER\":\"ZE08CH2O\",\"DEVICE\":\"%s\",\"CH2O\":\"%d\"}",  
            _mac_address, ch2o);
    https_post_json(post_data);
}

void device_set_state(device_state_t* device, const esp_err_t esp_err){
    switch(esp_err){
        case ESP_OK:
            *device = ON;
            break;
        case ESP_FAIL:
        case ESP_ERR_TIMEOUT:
            *device = ERROR;
            break;
        case ESP_ERR_INVALID_RESPONSE:
            *device = OFF;
            break;
        default:
            *device = UNKNOWN;
    }
}

void subsystem_set_state(device_state_t* device, const esp_err_t esp_err){
    switch(esp_err){
        case ESP_OK:
            *device = ON;
            break;
        case ESP_FAIL:
            *device = OFF;
            break;
        default:
            *device = UNKNOWN;
    }
}