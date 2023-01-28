#include <string.h>
#include <stdarg.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <esp_https_ota.h>
#include <esp_ota_ops.h>
#include <esp_http_client.h>
#include <freertos/event_groups.h>
#include <cJSON.h>

#include "wifi_manager.h"
#include "httpclient.h"
#include "radsens.h"

#define ROOT_DIR                            CONFIG_ROOT_DIR
#define LOAD_SETTING_SCRIPTNAME             CONFIG_LOAD_SETTING_SCRIPTNAME
#define COMMAND_SCRIPTNAME                  CONFIG_COMMAND_SCRIPTNAME
#define COMMAND_UPGRADE_SUCCESS             CONFIG_COMMAND_UPGRADE_SUCCESS
#define COMMAND_UPGRADE_FAIL                CONFIG_COMMAND_UPGRADE_FAIL
#define FIRMWARE_EMPTY                      CONFIG_FIRMWARE_EMPTY

#define FIRMWARE_DIR                        CONFIG_FIRMWARE_DIR

#define BASE_URL_MAX_SIZE 64
#define URL_MAX_SIZE 256
#define MESSAGE_MAX_LEN 128

#define LOAD_CONFIG_TIMEOUT_S  5
#define FLASH_READ_TIMEOUT_S 10
#define UPGRADE_TIMEOUT_S 1
#define OTA_RECV_TIMEOUT_MS 5000

#define ON  1
#define OFF 2

const int CPU0_LOAD_START_BIT = BIT0;
const int CPU1_LOAD_START_BIT = BIT1;

EventGroupHandle_t cpu_load_event_group;
ulong idleCnt0=0;
ulong idleCnt1=0;

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

static SemaphoreHandle_t _lock_http_socket = NULL;
static esp_http_client_handle_t _client = NULL;

void (*cb_ptr_start)(void*) = NULL;
void (*cb_ptr_stop)(void*) = NULL;
void (*cb_ptr_stop_peripheral)(void*) = NULL;
void (*cb_ptr_start_peripheral)(void*) = NULL;
void (*cb_ptr_drop_peripheral)(void*) = NULL;

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

static void _do_firmware_upgrade(void* arg);
static void _task_read_flash(void* arg);

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

void https_client_set_drop_peripheral(void (*func_ptr)(void*) ){
    cb_ptr_drop_peripheral = func_ptr;    
}

static void date_time_format(char* date_time, time_t t){
    time_t now = (t==0) ? get_local_datetime() : t;
	struct tm ti = {0};	
    localtime_r(&now, &ti);
    sprintf(date_time, "20%02d-%02d-%02d %02d:%02d:%02d", ti.tm_year - 100, ti.tm_mon + 1,
           ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec);
}

void peripheral_start(){
    if(cb_ptr_start_peripheral) (*cb_ptr_start_peripheral)(NULL);
}

void peripheral_stop(){
    if(cb_ptr_stop_peripheral) (*cb_ptr_stop_peripheral)(NULL);
}

void peripheral_drop(){
    if(cb_ptr_drop_peripheral) (*cb_ptr_drop_peripheral)(NULL);
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

                ESP_LOGD(TAG, "Valve state: %d", peripheral._valve_state);
                ESP_LOGD(TAG, "CCS811 baseline:: %d", peripheral._ccs811_baseline);
                ESP_LOGD(TAG, "Min temperature:: %f", peripheral._min_temp);
                ESP_LOGD(TAG, "Max temperature:: %f", peripheral._max_temp);
                ESP_LOGD(TAG, "Valve disable: %d", peripheral._valve_disable);
                ESP_LOGD(TAG, "Firmware file: %s", peripheral._firmware);
                ESP_LOGD(TAG, "Setup mode: %d", peripheral._setup_mode);
                ESP_LOGD(TAG, "Reboot mode: %d", peripheral._restart);
                ESP_LOGD(TAG, "Radsens mode: %d", peripheral._radsens_hv_mode);
                ESP_LOGD(TAG, "Radsens state: %d", peripheral._radsens_hv_state);
                ESP_LOGD(TAG, "Radsens sensitivity: %d", peripheral._radsens_sensitivity);
                ESP_LOGD(TAG, "Clear journal: %d", peripheral._clear_journal);
            }
            free(buffer);
        }else{
            ESP_LOGE(TAG, "Http Status: %d", http_code);
        }
    }

    return ESP_OK;
}

void https_disconnect(){
    if(_client){
        esp_http_client_close(_client);
        esp_http_client_cleanup(_client);
        _client = NULL;
    }
}

static void task_firmware_start(){
    xTaskCreatePinnedToCore(_do_firmware_upgrade, "_do_firmware_upgrade", configMINIMAL_STACK_SIZE*8, NULL, 10, &xUpgradeFirmware, 0);
}

/*static void task_firmware_stop(){
    if(xUpgradeFirmware){
        vTaskDelete( xUpgradeFirmware );
        xUpgradeFirmware = NULL;
    }    
}*/


void flash_read_start(){
    xTaskCreatePinnedToCore(_task_read_flash, "_task_read_flash", configMINIMAL_STACK_SIZE*8, NULL, 10, &xFlashLogActive, 0);
}

void flash_read_stop(){
    if(xFlashLogActive != NULL)
    {
        vTaskDelete( xFlashLogActive );
        xFlashLogActive = NULL;
    }    
}

esp_err_t _https_post_json(const char* post_data, bool force){
    
    if(!https_client_active() && !force)
    {
        FLASH_LOGW("WiFi Connection lost");
        ESP_LOGW(TAG, "WiFi Connection lost");
        return ESP_FAIL;    
    }
    if(_upgrade_during){
        _http_post_stoped = true;
        return ESP_OK;
    }

    SEMAPHORE_TAKE(_lock_http_socket, portMAX_DELAY);
    ESP_LOGD(TAG, "TAKE Semaphore");
    if(!_client) {
        esp_http_client_config_t config = {
            .url = _root_url,
            .method = HTTP_METHOD_POST,
            .transport_type = HTTP_TRANSPORT_OVER_SSL,
            .cert_pem = (char*)wifi_manager_get_httpd_config()->ca, 
            .cert_len = wifi_manager_get_httpd_config()->ca_len,
            .client_cert_pem = (char*)wifi_manager_get_httpd_config()->client_crt, 
            .client_cert_len = wifi_manager_get_httpd_config()->crt_len,
            .client_key_pem = (char*)wifi_manager_get_httpd_config()->client_key, 
            .client_key_len = wifi_manager_get_httpd_config()->key_len,
            //.username = (char*)wifi_manager_get_httpd_config()->username,
            //.password = (char*)wifi_manager_get_httpd_config()->password,
            .event_handler = _http_get_config_eh
        };
        ESP_LOGI(TAG, "URL: %s", _root_url);

        _client = esp_http_client_init(&config);
        esp_http_client_set_header(_client, "Content-Type", "application/json");
    }
    esp_err_t err = esp_http_client_set_post_field(_client, post_data, strlen(post_data));
    if (err != ESP_OK){
        return err;
    }
    err = esp_http_client_perform(_client);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Lost coonnection");
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
        FLASH_LOGE("HTTP POST request failed: %s", esp_err_to_name(err));
        esp_restart();
    }
    ESP_LOGD(TAG, "Data send ok to: %s", _root_url);
    ESP_LOGD(TAG, "Data: %s", post_data);    
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

static esp_err_t https_ota_handle(esp_http_client_event_t *evt) {
    return ESP_OK;
}

static void _do_firmware_upgrade(void* arg){

    char url[URL_MAX_SIZE];
    int wait = 10;

    while(1){
        if(peripheral._firmware[0] != 0 && 
            strcmp(peripheral._firmware, FIRMWARE_EMPTY) != 0){
            
            if(!_upgrade_during) {
                ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade started ...");
                https_logi("ESP_HTTPS_OTA upgrade started ...");
                sprintf(url, "{\"ORDER\":\"UPGRADE_SUCCESS\",\"DEVICE\":\"%s\",\"FNAME\":\"%s\"}", 
                    _mac_address, peripheral._firmware);
                https_post_json(url);
                peripheral_drop();
                https_disconnect();
                _upgrade_during = true;
            }


            if(wait <= 0){
                sprintf(url, "%s/%s", _firmware_url, peripheral._firmware);
                esp_http_client_config_t config = {0};
                config.url = url;
                config.cert_pem = (char*)wifi_manager_get_httpd_config()->ca;               
                config.cert_len = wifi_manager_get_httpd_config()->ca_len;
                config.client_cert_pem = (char*)wifi_manager_get_httpd_config()->client_crt;
                config.client_cert_len = wifi_manager_get_httpd_config()->crt_len;
                config.client_key_pem = (char*)wifi_manager_get_httpd_config()->client_key;
                config.client_key_len = wifi_manager_get_httpd_config()->key_len;

                config.event_handler = https_ota_handle;
                //config.timeout_ms = 12000;
                config.keep_alive_enable = true;
                
                //config.skip_cert_common_name_check = true;

                ESP_LOGW(TAG, "URL: %s", url);
                SEMAPHORE_TAKE(_lock_http_socket, portMAX_DELAY);
                esp_err_t err = esp_https_ota(&config);
                SEMAPHORE_GIVE(_lock_http_socket);               
                if (err == ESP_OK) {
                    FLASH_LOGI("ESP_HTTPS_OTA upgrade success");
                }else{
                    FLASH_LOGE("ESP_HTTPS_OTA upgrade fail");
                }
                esp_restart();
            }
            wait--;
        }
        vTaskDelay(1000*UPGRADE_TIMEOUT_S / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
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

    while(!_upgrade_during){
        if(https_client_active()){

            ESP_LOGD(TAG, "Start flash log read");

            read_flash_log(_send_flash_log);

            ESP_LOGD(TAG, "Stop flash log read");
            vTaskDelay(1000*FLASH_READ_TIMEOUT_S / portTICK_RATE_MS);
        }
    }
    ESP_LOGW(TAG, "Read falsh task droped!");
    vTaskDelete(NULL);
}

static void _task_load_configure(void* arg) {

    //clear_flash_log();
    char url[URL_MAX_SIZE];
    while(!_upgrade_during){
        if(https_client_active())
        {

            char date_time[32];
            date_time_format(date_time, 0);
            sprintf(url, "{\"ORDER\":\"GET_SETTINGS\",\"DEVICE\":\"%s\",\"CPU0\":\"%f\",\"CPU1\":\"%f\",\"DATE_TIME\":\"%s\"}",  
                _mac_address, get_cpu0_load_percent(), get_cpu1_load_percent(), date_time);
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
                clear_flash_log();
                sprintf(url, "{\"ORDER\":\"JOURNAL_CLEARED\",\"DEVICE\":\"%s\"}",  
                        _mac_address);
                https_post_json(url);
            }
        }
        vTaskDelay(1000*LOAD_CONFIG_TIMEOUT_S / portTICK_RATE_MS);
    }
    ESP_LOGW(TAG, "Load config task droped!");
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
        
        sprintf(_root_url, "https://%s:%d%s", (char*)server_address, server_port, ROOT_DIR);
        sprintf(_firmware_url, "https://%s:%d%s", (char*)server_address, server_port, FIRMWARE_DIR); 
        get_device_mac_address(_mac_address); 
        
        initialize_cpu_monitor();

        wifi_manager_set_callback(WM_ORDER_START_AP, &cb_start_ap);
        wifi_manager_set_callback(WM_ORDER_STOP_AP, &cb_stop_ap);
    }
}

void https_client_start(){
    if(!_ap_started){
        _https_client_active = true;
        https_logi("HTTP Client Started");
        xTaskCreatePinnedToCore(_task_load_configure, "task_load_configure", configMINIMAL_STACK_SIZE*8, NULL, 10, &xHttpsClientActive, 0);
        if(cb_ptr_start) (*cb_ptr_start)(&xHttpsClientActive);
        task_firmware_start();
        flash_read_start();
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
    ESP_LOGD(TAG, "%s", post_data);
    ESP_LOGD(TAG, "%s: %s", date_time, message);
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
    char date_time[32];
    date_time_format(date_time, 0);
    sprintf(post_data, "{\"ORDER\":\"DS18B20\",\"DEVICE\":\"%s\",\"SENSOR\":\"%s\",\"TEMPR\":\"%f\",\"DATE_TIME\":\"%s\"}",  
            _mac_address, sensor_addr, tempr, date_time);
    https_post_json(post_data);
}

void https_bme280_send_data(const float t, const float h, const float p)
{
    char post_data[URL_MAX_SIZE];
    char date_time[32];
    date_time_format(date_time, 0);
    sprintf(post_data, "{\"ORDER\":\"BME280\",\"DEVICE\":\"%s\",\"TEMPR\":\"%f\",\"PRESS\":\"%f\",\"HUM\":\"%f\",\"DATE_TIME\":\"%s\"}",  
            _mac_address, t, p, h, date_time);
    https_post_json(post_data); 
}

void https_radsens_send_data(const float ri_dynamic, const float ri_static, const uint32_t pulses)
{
    char post_data[URL_MAX_SIZE];
    char date_time[32];
    date_time_format(date_time, 0);
    sprintf(post_data, "{\"ORDER\":\"RADSENS\",\"DEVICE\":\"%s\",\"RID\":\"%f\",\"RIS\":\"%f\",\"PULSE\":\"%d\",\"DATE_TIME\":\"%s\"}",  
            _mac_address, ri_dynamic, ri_static, pulses, date_time);
    https_post_json(post_data);
}

void https_ze08_send_data(const uint16_t ch2o) 
{
    char post_data[URL_MAX_SIZE];
    char date_time[32];
    date_time_format(date_time, 0);
    sprintf(post_data, "{\"ORDER\":\"ZE08CH2O\",\"DEVICE\":\"%s\",\"CH2O\":\"%d\",\"DATE_TIME\":\"%s\"}",  
            _mac_address, ch2o, date_time);
    https_post_json(post_data);
}

void https_6814_send_data(const float nh3, const float co, const float no2) 
{
    char post_data[URL_MAX_SIZE];
    char date_time[32];
    date_time_format(date_time, 0);
    sprintf(post_data, "{\"ORDER\":\"6814\",\"DEVICE\":\"%s\",\"NH3\":\"%f\",\"CO\":\"%f\",\"NO2\":\"%f\",\"DATE_TIME\":\"%s\"}",  
            _mac_address, nh3, co, no2, date_time);
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

bool isSntpOk() {
    return get_sntp_status();
}


void idleCPU0Task(void *parm){
    ulong now, now2;

    while (true){
        //Wait here if not enabled
        xEventGroupWaitBits(cpu_load_event_group, CPU0_LOAD_START_BIT, false, false, portMAX_DELAY);
        now = esp_timer_get_time(); //esp_timer_get_time();
        vTaskDelay(0 / portTICK_RATE_MS);
        now2 = esp_timer_get_time(); //esp_timer_get_time();
        idleCnt0 += (now2 - now); //accumulate the usec's
    }
}

void idleCPU1Task(void *parm)
{
    ulong now, now2;

    while (true){
    //Wait here if not enabled
    xEventGroupWaitBits(cpu_load_event_group, CPU1_LOAD_START_BIT, false, false, portMAX_DELAY);
    now = esp_timer_get_time();
    vTaskDelay(0 / portTICK_RATE_MS);
    now2 = esp_timer_get_time();
    idleCnt1 += (now2 - now); //accumulate the msec's while enabled
    }
}

void initialize_cpu_monitor() {
    cpu_load_event_group = xEventGroupCreate();
    xEventGroupClearBits(cpu_load_event_group, CPU0_LOAD_START_BIT);
    xEventGroupClearBits(cpu_load_event_group, CPU1_LOAD_START_BIT);
    xTaskCreatePinnedToCore(&idleCPU0Task, "idleCPU0Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL,0); //lowest priority CPU 0 task
    xTaskCreatePinnedToCore(&idleCPU1Task, "idleCPU1Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL,1); //lowest priority CPU 1 task
}

float get_cpu0_load_percent(){
    idleCnt0 = 0; // Reset usec timer
    xEventGroupSetBits(cpu_load_event_group, CPU0_LOAD_START_BIT); // Signal idleCPU0Task to start timing
    vTaskDelay(1000 / portTICK_RATE_MS); //measure for 1 second
    xEventGroupClearBits(cpu_load_event_group, CPU0_LOAD_START_BIT); // Signal to stop the timing
    vTaskDelay(1 / portTICK_RATE_MS); //make sure idleCnt isnt being used

    // Compensate for the 100 ms delay artifact: 900 msec = 100%
    return (float)(100 - ((99.9 / 90.0) * idleCnt0/1000.0) / 10.0);
}

float get_cpu1_load_percent(){
    idleCnt1 = 0; // Reset usec timer
    xEventGroupSetBits(cpu_load_event_group, CPU1_LOAD_START_BIT); // Signal idleCPU1Task to start timing
    vTaskDelay(1000 / portTICK_RATE_MS); //measure for 1 second
    xEventGroupClearBits(cpu_load_event_group, CPU1_LOAD_START_BIT); // Signal idle_task to stop the timing
    vTaskDelay(1 / portTICK_RATE_MS); //make sure idleCnt isnt being used

    // Compensate for the 100 ms delay artifact: 900 msec = 100%
    return (float)(100 - ((99.9 / 90.0) * idleCnt1/1000.0) / 10.0);
}
