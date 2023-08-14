#include <stdio.h>
#include <string.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <cJSON.h>

#include "flash.h"
#include "manager.h"
#include "http_client.h"
#include "ntp_client.h"
#include "i2cdev.h"
#include "radsens.h"
#include "bmp280.h"
#include "ds18x20.h"
#include "ze08ch2o.h"
#include "button.h"
#include "mics6814.h"
#include "aht.h"
#include "ssd1306.h"
#include "fonts.h"
#include "main.h"

#define I2C_SCL_IO                  GPIO_NUM_22               
#define I2C_SDA_IO                  GPIO_NUM_23               
#define DS18B20_GPIO                GPIO_NUM_21
#define ZE08CH2O_GPIO               GPIO_NUM_19
#define MICS6814_MISO               GPIO_NUM_5
#define MICS6814_SCLK               GPIO_NUM_17
#define MICS6814_CS                 GPIO_NUM_18
#define BUTTON_PIN                  GPIO_NUM_27
#define ARDUINO_RST                 GPIO_NUM_16

#define MAX_TRY_INIT                (3)            // Number of attempts to initialize sensors
#define DELAY_INIT_MS               2000           // Delay between attempts to initialize sensors

#define PULSE_COUNT                 11             // Number of pulses to reset arduino

static const char *TAG = "MAIN";

static char spin_buffer[32] = {0};

static meashure_results_t* mr = NULL;

static bmp280_t bmp280_dev = {0};

static ds18x20_addr_t ds18x20_addrs = {0};   

static esp_err_t ssd1306_status = ESP_FAIL;

static aht_t aht25_dev = { 0 };

static screen_code_t _current_screen = SC_STATUS;

TimerHandle_t oled_sleep_timer = NULL;

EventGroupHandle_t main_events;

void spin_touch() {
    const uint8_t end_pos = 20;
    uint pos = strlen(spin_buffer);
    if (pos >= end_pos) {
        spin_buffer[0] = '\0';
    }else {
        spin_buffer[pos] = '-';
        spin_buffer[pos+1] = '\0';
    }
}

static void SSD1306_Init_Log(const char* msg) {
    if (ssd1306_status == ESP_OK) {
        SSD1306_Fill(SSD1306_COLOR_BLACK);
        SSD1306_GotoXY(10, 30); 
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(10, 50); 
        SSD1306_Puts(spin_buffer, &Font_7x10, SSD1306_COLOR_WHITE);
        spin_touch();
        SSD1306_UpdateScreen();
    }
}

/**
  * @brief  "arduino reset function"
  * @retval None
  */
static void arduino_reset() {
    for(int i=0; i < PULSE_COUNT; i++) {
        gpio_set_level(ARDUINO_RST, 1);
        vTaskDelay(10 / portTICK_RATE_MS);
        gpio_set_level(ARDUINO_RST, 0);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}


/**
  * @brief  "Refresh oled dysplay"
  * @param  None
  * @retval None
  */
static void oled_refresh() {
    if (ssd1306_status == ESP_OK) {
        char line[64];
        SSD1306_Fill(SSD1306_COLOR_BLACK);
        switch(_current_screen) {
            case SC_STATUS:
                SSD1306_GotoXY(10, 0); 
                SSD1306_Puts((mr->bmp280_status == SENSOR_OK) ? BMP280_OK : BMP280_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(10, 10); 
                SSD1306_Puts((mr->ds18b20_status == SENSOR_OK) ? DS18B20_OK : DS18B20_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(10, 20); 
                SSD1306_Puts((mr->radsens_status == SENSOR_OK) ? RADSENS_OK : RADSENS_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(10, 30); 
                SSD1306_Puts((mr->ze08_status == SENSOR_OK) ? ZE08_OK : ZE08_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(10, 40); 
                SSD1306_Puts((mr->mics6814_status == SENSOR_OK) ? MICS6814_OK : MICS6814_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(10, 50); 
                SSD1306_Puts((mr->aht25_status == SENSOR_OK) ? AHT25_OK : AHT25_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                break;
            case SC_BMP280:
                SSD1306_GotoXY(10, 0); 
                SSD1306_Puts((mr->bmp280_status == SENSOR_OK) ? BMP280_OK : BMP280_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                if (mr->bmp280_status == SENSOR_OK) {
                    SSD1306_GotoXY(10, 10); 
                    SSD1306_Puts("", &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 20); 
                    sprintf(line, "T: %2.1f C", mr->bmp280_temperature);
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 30); 
                    sprintf(line, "P: %3.1f mmHg", mr->bmp280_pressure / 133.3);
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                }
                break;
            case SC_AHT25:
                SSD1306_GotoXY(10, 0); 
                SSD1306_Puts((mr->aht25_status == SENSOR_OK) ? AHT25_OK : AHT25_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                if (mr->aht25_status == SENSOR_OK) {
                    SSD1306_GotoXY(10, 10); 
                    SSD1306_Puts("", &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 20); 
                    sprintf(line, "T: %2.1f C", mr->aht25_temperature);
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 30); 
                    sprintf(line, "H: %3.1f %%", mr->aht25_humidity);
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                }
                break;
            case SC_DS18B20:
                SSD1306_GotoXY(10, 0); 
                SSD1306_Puts((mr->ds18b20_status == SENSOR_OK) ? DS18B20_OK : DS18B20_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                if (mr->ds18b20_status == SENSOR_OK) {
                    SSD1306_GotoXY(10, 10); 
                    SSD1306_Puts("", &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 20); 
                    sprintf(line, "T: %2.1f C", mr->ds18b20_temperature);
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                }
                break;
            case SC_RADSENS:
                SSD1306_GotoXY(10, 0); 
                SSD1306_Puts((mr->radsens_status == SENSOR_OK) ? RADSENS_OK : RADSENS_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                if (mr->radsens_status == SENSOR_OK) {
                    SSD1306_GotoXY(10, 10); 
                    SSD1306_Puts("", &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 20); 
                    sprintf(line, "S:  %2.1f mkR/h", mr->radsens_i_static);
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 30); 
                    sprintf(line, "D:  %2.1f mkR/h", mr->radsens_i_dynamic);
                    SSD1306_GotoXY(10, 40); 
                    sprintf(line, "HV: %s", (mr->radsens_hv_state == 1) ? "ON" : "OFF" );
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                }
                break;
            case SC_ZE08:
                SSD1306_GotoXY(10, 0); 
                SSD1306_Puts((mr->ze08_status == SENSOR_OK) ? ZE08_OK : ZE08_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                if (mr->ze08_status == SENSOR_OK) {
                    SSD1306_GotoXY(10, 10); 
                    SSD1306_Puts("", &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 20); 
                    sprintf(line, "CH2O: %d ppm", mr->ze08_ch2o);
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                }
                break;
            case SC_MISC6418:
                SSD1306_GotoXY(10, 0); 
                SSD1306_Puts((mr->mics6814_status == SENSOR_OK) ? MICS6814_OK : MICS6814_FAIL, &Font_7x10, SSD1306_COLOR_WHITE);
                if (mr->mics6814_status == SENSOR_OK) {
                    SSD1306_GotoXY(10, 10); 
                    SSD1306_Puts("", &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 20); 
                    sprintf(line, "CO:  %1.1f mg/m3", mr->mics6814_co);
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 30); 
                    sprintf(line, "NO2: %1.1f mg/m3", mr->mics6814_no2);
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(10, 40); 
                    sprintf(line, "NH3: %1.1f mg/m3", mr->mics6814_nh3);
                    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
                }
                break;
            default:
                ESP_LOGE(TAG, "Unknown screen: %d", _current_screen);
        }
        SSD1306_UpdateScreen();
        _current_screen++;
        if (_current_screen >= SCREEN_COUNT) {
            _current_screen = SC_STATUS;
        }
    }
}

/**
  * @brief  "Button count push callback function"
  * @param  pvParameters: Pointer to received data
  * @retval None
  */
static void cb_press_count(int count) {
    ESP_LOGI(TAG, "Button pressed %d times", count);
}

/**
  * @brief  "Button push callback function"
  * @param  None
  * @retval None
  */
static void cb_down_shot() {
    if ( oled_sleep_timer ){
        if (xTimerIsTimerActive(oled_sleep_timer) == pdTRUE) {
           xTimerStop( oled_sleep_timer, (TickType_t) 0 );
        }
        xTimerStart( oled_sleep_timer, (TickType_t)0 );
        /* OLED activate */
        oled_refresh();
        SSD1306_ON();
    }
}

/**
  * @brief  "Button push callback function"
  * @param  None
  * @retval None
  */
static void cb_up_shot(unsigned long time) {
    if (time > CONFIG_RESET_ESP32 && time < CONFIG_RESET_ARDUINO) {
        ESP_LOGI(TAG, "ESP32 Rebooting...");
        SSD1306_Fill(SSD1306_COLOR_BLACK);
        SSD1306_GotoXY(10, 0); 
        SSD1306_Puts("ESP32", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(10, 10); 
        SSD1306_Puts("Reboot...", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_UpdateScreen();
        SSD1306_ON();
        vTaskDelay(2000 / portTICK_RATE_MS);
        esp_restart();
    }
    if (time > CONFIG_RESET_ARDUINO && time < CONFIG_SETUP_MODE_ESP32) {
        ESP_LOGI(TAG, "Arduino Rebooting...");
        SSD1306_Fill(SSD1306_COLOR_BLACK);
        SSD1306_GotoXY(10, 0); 
        SSD1306_Puts("Arduino", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(10, 10); 
        SSD1306_Puts("Reboot...", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_UpdateScreen();
        SSD1306_ON();
        vTaskDelay(2000 / portTICK_RATE_MS);
        arduino_reset();
        SSD1306_OFF();
        return;
    }
    if (time > CONFIG_SETUP_MODE_ESP32) {
        ESP_LOGI(TAG, "ESP32 Setup mode runing..."); 
        SSD1306_Fill(SSD1306_COLOR_BLACK);
        SSD1306_GotoXY(10, 0); 
        SSD1306_Puts("ESP32", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(10, 10); 
        SSD1306_Puts("Setup...", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_UpdateScreen();
        SSD1306_ON();
        vTaskDelay(2000 / portTICK_RATE_MS);
        wifi_manager_start_setup_mode();
    }
}

/**
  * @brief  "This function enable sleep mode oled"
  * @param  xTimer: Timer handle
  * @retval None
  */
static void timer_cb( TimerHandle_t xTimer ) {
	/* stop the timer */
	xTimerStop( xTimer, (TickType_t) 0 );
    /* OLED sleep*/
    SSD1306_OFF();
}

/**
  * @brief  "HTTP client callback function"
  * @param  pvParameters: Pointer to received data
  * @retval None
  */
static void cb_http_client_response(const char* data, int len) {
    // http_client_response_t* r = (http_client_response_t*)pvParameters;
    if (len > 0) {  
        cJSON *root = cJSON_Parse(data);
        if (root) {
    
            // ESP_LOGI(TAG, "DATA: %s", r->data);

            // Check order set wifi access point mode
            cJSON* order = cJSON_GetObjectItem(root, CMD_AP);
            if (!order) {
                ESP_LOGE(TAG, "JSON object [%s] not found", CMD_AP);
            }else {
                if ( order->valueint ) {
                    ESP_LOGW(TAG, "Setup mode...");
                    vTaskDelay(1000 / portTICK_RATE_MS);
                    // wifi_manager_start_setup_mode();
                }
            }
            // Check order reboot
            order = cJSON_GetObjectItem(root, CMD_REBOOT);
            if (!order) {
                ESP_LOGE(TAG, "JSON object [%s] not found", CMD_REBOOT);
            }else {
                if( order->valueint ) {
                    ESP_LOGW(TAG, "REBOOT...");
                    vTaskDelay(1000 / portTICK_RATE_MS);
                    // esp_restart();
                }
            }
            // Check order radsens setup
            order = cJSON_GetObjectItem(root, CMD_RADSENS_MODE);
            if (!order) {
                ESP_LOGE(TAG, "JSON object [%s] not found", CMD_RADSENS_MODE);
            }else {
                if( order->valueint ) {
                    ESP_LOGW(TAG, "Radsens setup mode...");
                    cJSON* state = cJSON_GetObjectItem(root, CMD_RADSENS_STATE);
                    if (!state) {
                        ESP_LOGE(TAG, "JSON object [%s] not found", CMD_RADSENS_STATE);
                    }else {
                        if (radsens_set_hv_generator(state->valueint) == ESP_OK) {
                            ESP_LOGI(TAG, "RadSens HV: %s", (state->valueint == 0) ? "OFF" : "ON");
                        }else {
                            ESP_LOGE(TAG, "RadSens set HV fail");
                        }
                    }
                    cJSON* sensitivity = cJSON_GetObjectItem(root, CMD_RADSENS_SENS);
                    if (!sensitivity) {
                        ESP_LOGE(TAG, "JSON object [%s] not found", CMD_RADSENS_SENS);
                    }else {
                        if( radsens_set_sensitivity(sensitivity->valueint) == ESP_OK){
                            ESP_LOGI(TAG, "RadSens set sensitivity: %d", sensitivity->valueint);
                        }else {
                            ESP_LOGE(TAG, "RadSens set sensitivity fail");
                        }
                    }
                }
            }
            // Check order clear flash logging
            order = cJSON_GetObjectItem(root, CMD_CLEAR_JOURNAL);
            if (!order) {
                ESP_LOGE(TAG, "JSON object [%s] not found", CMD_CLEAR_JOURNAL);
            }else {
                if( order->valueint ) {
                    ESP_LOGW(TAG, "Clear flash logging...");
                    clear_flash_log();
                }
            }
            // Check order reboot digispark
            order = cJSON_GetObjectItem(root, CMD_DIGISPARK_REBOOT);
            if (!order) {
                ESP_LOGE(TAG, "JSON object [%s] not found", CMD_DIGISPARK_REBOOT);
            }else {
                if( order->valueint ) {
                    ESP_LOGW(TAG, "Arduino reboot...");
                    arduino_reset();
                }
            }
        }
        if(root) cJSON_Delete(root);
    }   
}

/**
  * @brief  "This callback function is called  when the http client activates"
  * @param  pvParameters: Pointer to received data
  * @retval None
  */
static void cb_http_client_ready(void* pvParameters) {
    xEventGroupSetBits(main_events, HC_STATUS_OK);
    SSD1306_Init_Log("HTTP Client OK");
}

/**
  * @brief  "This callback function is called  when the http client deactivates"
  * @param  pvParameters: Pointer to received data
  * @retval None
  */
static void cb_http_client_not_ready(void* pvParameters) {
    xEventGroupClearBits(main_events, HC_STATUS_OK);
    SSD1306_Init_Log("HTTP Client FAIL");
}

/**
  * @brief  "Callback function send flash logging"
  * @param  t: Local esp32 time
  * @param  log_type: Type logging message
  * @param  msg: Logging message
  * @retval None
  */
#ifdef CONFIG_USE_FLASH_LOGGING
void static send_flash_logging_message(const time_t t, const char log_type, const char* msg) {
    char buff[32];

    buff[0] = log_type;
    buff[1] = '\0';
    time_t sntp_time = get_sntp_time_init();
    time_t sntp_tiks = get_sntp_tiks_init();
    const time_t datetime = (t < sntp_tiks) ? (sntp_time - (sntp_tiks - t)) : t;    

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", buff);
    cJSON_AddStringToObject(root, "date_time", date_time_format(buff, datetime));
    cJSON_AddStringToObject(root, "message", msg);
    // send json sting to http server
    const char* json_string = cJSON_Print(root);
    http_client_send_message(HTTP_METHOD_POST, "/logging", json_string);
    free((void* )json_string);
    cJSON_Delete(root);
}
#endif

/**
  * @brief  "This function requests instructions from backend, measurements from stm32 and sends flash log"
  * @param  pvParameters: Pointer to received data
  * @retval None
  */
static void get_orders_task(void* pvParameters) {

    uint8_t meashure_ticks = 0;
    uint8_t flash_log_ticks = 0;
    esp_err_t err;

    for(;;) {
        // ESP_LOGI(TAG, "CLIENT: %d, TICKS: %d", http_client_ready, meashure_ticks);
        xEventGroupWaitBits(
                main_events,           // The event group being tested.
                HC_STATUS_OK,           // The bits within the event group to wait for.
                pdFALSE,                // HC_STATUS_OK should be not cleared before returning.
                pdFALSE,                // Don't wait for both bits, either bit will do.
                portMAX_DELAY );        // Wait until the bit be set.                            

        if (meashure_ticks < TICKS_MEASHURE_SEND) {
            // request order from backend
            http_client_send_message(HTTP_METHOD_GET, "/orders", NULL);
            meashure_ticks++;
        } else {
            // request ds18b20 data
            if (mr->ds18b20_status != SENSOR_INIT_FAIL) {
                err = ds18x20_measure_and_read_multi(DS18B20_GPIO, &ds18x20_addrs, 1, &mr->ds18b20_temperature);
                mr->ds18b20_status = (err == ESP_OK) ? SENSOR_OK : SENSOR_FAIL;
                // ESP_LOGW(TAG, "DS18B20: %f", mr->ds18b20_temperature);
            }
            // request bme280 data
            if (mr->bmp280_status != SENSOR_INIT_FAIL) {
                err = bmp280_read_float(&bmp280_dev, &mr->bmp280_temperature, &mr->bmp280_pressure, &mr->bmp280_humidity);
                mr->bmp280_status = (err == ESP_OK) ? SENSOR_OK : SENSOR_FAIL;
                // ESP_LOGW(TAG, "BMP280: %f", mr->bmp280_temperature);
            }
            // request radsens data
            if (mr->radsens_status != SENSOR_INIT_FAIL) {
                radsens_data comp_data;
                err = radsens_read_data(&comp_data);                   
                mr->radsens_status = (err == ESP_OK) ? SENSOR_OK : SENSOR_FAIL;
                if(mr->radsens_status == SENSOR_OK) {
                    mr->radsens_pulse = comp_data.pulse_cnt;
                    mr->radsens_i_static = comp_data.i_static;
                    mr->radsens_i_dynamic = comp_data.i_dynamic;
                    mr->radsens_hv_state = comp_data.hv_state;
                    mr->radsens_sens = comp_data.sens;
                }
                // ESP_LOGW(TAG, "RADSENS: %f, HV: %d, SENS: %d", mr->radsens_i_static, mr->radsens_hv_state, mr->radsens_sens);
            }
            // request ze08 data
            if (mr->ze08_status != SENSOR_INIT_FAIL) {
                mr->ze08_ch2o = ze08ch2o_read();
                mr->ze08_status = (mr->ze08_ch2o == ZE08CH2O_READ_ERROR_CODE) ? SENSOR_FAIL : SENSOR_OK;
                // ESP_LOGW(TAG, "ZE08: %d", mr->ze08_ch2o);
            }
            // request mics6814 data
            if (mr->mics6814_status != SENSOR_INIT_FAIL) {
                mics6814_data_t data = {0};
                err = get_mics6814_data(&data);
                mr->mics6814_status = (err == ESP_OK) ? SENSOR_OK : SENSOR_FAIL;
                mr->mics6814_co = data.co;
                mr->mics6814_no2 = data.no2;
                mr->mics6814_nh3 = data.nh3;
                // ESP_LOGW(TAG, "[MICS6814] CO: %f, NO2: %f, NH3: %f, STATUS: %d", data.co, data.no2, data.nh3, data.status);
            }
            // request aht25 data
            if (mr->aht25_status != SENSOR_INIT_FAIL) {
                err = aht_get_data(&aht25_dev, &mr->aht25_temperature, &mr->aht25_humidity);
                mr->aht25_status = (err == ESP_OK) ? SENSOR_OK : SENSOR_FAIL;
                // ESP_LOGW(TAG, "AHT25: %f", mr->aht25_humidity);
            }

            cJSON *root = cJSON_CreateObject();
            cJSON_AddNumberToObject(root, "bmp280_status", mr->bmp280_status);
            cJSON_AddNumberToObject(root, "bmp280_pressure", mr->bmp280_pressure);
            cJSON_AddNumberToObject(root, "bmp280_temperature", mr->bmp280_temperature);
            cJSON_AddNumberToObject(root, "bmp280_humidity", mr->bmp280_humidity);
            cJSON_AddNumberToObject(root, "ds18b20_status", mr->ds18b20_status);
            cJSON_AddStringToObject(root, "ds18b20_address", mr->ds18b20_address);
            cJSON_AddNumberToObject(root, "ds18b20_temperature", mr->ds18b20_temperature);
            cJSON_AddNumberToObject(root, "radsens_status", mr->radsens_status);
            cJSON_AddNumberToObject(root, "radsens_i_static", mr->radsens_i_static);
            cJSON_AddNumberToObject(root, "radsens_i_dynamic", mr->radsens_i_dynamic);
            cJSON_AddNumberToObject(root, "radsens_pulse", mr->radsens_pulse);
            cJSON_AddNumberToObject(root, "radsens_hv_state", mr->radsens_hv_state);
            cJSON_AddNumberToObject(root, "radsens_sens", mr->radsens_sens);
            cJSON_AddNumberToObject(root, "ze08_status", mr->ze08_status);
            cJSON_AddNumberToObject(root, "ze08_ch2o", mr->ze08_ch2o);
            cJSON_AddNumberToObject(root, "mics6814_status", mr->mics6814_status);
            cJSON_AddNumberToObject(root, "mics6814_co", mr->mics6814_co);
            cJSON_AddNumberToObject(root, "mics6814_nh3", mr->mics6814_nh3);
            cJSON_AddNumberToObject(root, "mics6814_no2", mr->mics6814_no2);
            cJSON_AddNumberToObject(root, "aht25_status", mr->aht25_status);
            cJSON_AddNumberToObject(root, "aht25_temperature", mr->aht25_temperature);
            cJSON_AddNumberToObject(root, "aht25_humidity", mr->aht25_humidity);
            // send json sting to http server
            const char* json_string = cJSON_Print(root);
            http_client_send_message(HTTP_METHOD_POST, "/meashure", json_string);
            free((void* )json_string);
            cJSON_Delete(root);
            meashure_ticks = 0;
        }

        if (flash_log_ticks < TICKS_FLASH_LOG_SEND) {
            flash_log_ticks++;
        }else{
#ifdef CONFIG_USE_FLASH_LOGGING             
            // send flash logging
            read_flash_log(send_flash_logging_message);
            ESP_LOGI(TAG, "Flash log sending complet!");
#endif
            flash_log_ticks = 0;
        }

        vTaskDelay(DELAY_ORDER_REQUEST_MS / portTICK_RATE_MS);
    } /* end of for loop */

	ESP_LOGI(TAG, "GET ORDERS TASK STOPPED");
  
	vTaskDelete( NULL );        
}

// static void check_mem_task(void * param) {
//     for(;;) {
//         ESP_LOGW(TAG, "HEAP: %d", esp_get_free_heap_size());
//         vTaskDelay(1000 / portTICK_RATE_MS);
//     }
//     vTaskDelete( NULL ); 
// }

void app_main(void)
{
	/* initialize flash */
	init_flash();

    /* I2C initialize */
    ESP_ERROR_CHECK(i2cdev_init());
    vTaskDelay(100 / portTICK_RATE_MS);

    /* SSD1306 Initialize */
    SSD1306_init_desc(I2C_SDA_IO, I2C_SCL_IO, 0);
	ssd1306_status = SSD1306_Init();
    if (ssd1306_status != ESP_OK) {
        ESP_LOGW(TAG, "SSD1306 initialisation fail");
    }else {
        ESP_LOGI(TAG, "SSD1306 initialisation success");
        SSD1306_Fill(SSD1306_COLOR_BLACK);
        SSD1306_Init_Log("Loading...");
        SSD1306_ON();
    }

    /* WIFI initialize */
    wifi_manager_start(false);

    /* HTTP Client initialize */
    http_client_initialize();

	/* memory allocation */
    mr = (meashure_results_t*)malloc(sizeof(meashure_results_t));
    memset(mr, 0x00, sizeof(meashure_results_t));

    /* create tasks event group */
    main_events = xEventGroupCreate();    

    /* Callbacks link */
    http_client_set_response_callback(&cb_http_client_response);
    http_client_set_ready_callback(&cb_http_client_ready);
    http_client_set_not_ready_callback(&cb_http_client_not_ready);

    /* GPIO configure */
    gpio_set_direction(ARDUINO_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(ARDUINO_RST, 0);

    /* Button Initialize */
    button_init(BUTTON_PIN, cb_press_count, cb_down_shot, cb_up_shot);

    esp_err_t err;
    uint8_t TryToInit = 0;

    /* DS18B20 Initialize */
    size_t ds18x20_sensor_count = 0;
    gpio_set_pull_mode(DS18B20_GPIO, GPIO_PULLUP_ONLY);
    do {
        err = ds18x20_scan_devices(DS18B20_GPIO, &ds18x20_addrs, 1, &ds18x20_sensor_count);
        SSD1306_Init_Log((err == ESP_OK && ds18x20_sensor_count > 0) ? DS18B20_OK : DS18B20_FAIL);
        if (err == ESP_OK) {
            break;
        }else {
            vTaskDelay(DELAY_INIT_MS / portTICK_RATE_MS);
        }
        TryToInit++;
    }while(TryToInit < MAX_TRY_INIT);
    if(err == ESP_OK && ds18x20_sensor_count > 0){
        mr->ds18b20_status = SENSOR_OK;
        sprintf(mr->ds18b20_address, "%08X%08X", (uint32_t)(ds18x20_addrs >> 32), (uint32_t)ds18x20_addrs);
        ESP_LOGI(TAG, "DS18B20 : %s initialisation success", mr->ds18b20_address);
        FLASH_LOGI("DS18B20 : %s initialisation success", mr->ds18b20_address);
    }else{
        mr->ds18b20_status = SENSOR_INIT_FAIL;
        ESP_LOGW(TAG, "%s: DS18B20 initialisation fail", esp_err_to_name(err));
        FLASH_LOGW("%s: DS18B20 initialisation fail", esp_err_to_name(err));
    }


    /* BME280 Initialize */
    bmp280_params_t bmp20_params;
    bmp280_init_default_params(&bmp20_params);
    memset(&bmp280_dev, 0, sizeof(bmp280_t));
    TryToInit = 0;
    do {
        err = (bmp280_init_desc(&bmp280_dev, BMP280_I2C_ADDRESS_0, 0, I2C_SDA_IO, I2C_SCL_IO) == ESP_OK &&
            bmp280_init(&bmp280_dev, &bmp20_params) == ESP_OK) ? ESP_OK : ESP_FAIL;
        SSD1306_Init_Log((err == ESP_OK) ? BMP280_OK : BMP280_FAIL);
        if (err == ESP_OK) {
            break;
        }else {
            vTaskDelay(DELAY_INIT_MS / portTICK_RATE_MS);
        }
        TryToInit++;
    }while(TryToInit < MAX_TRY_INIT);
    bool bme280p = bmp280_dev.id == BME280_CHIP_ID;
    if(err == ESP_OK){
        mr->bmp280_status = SENSOR_OK;
        ESP_LOGI(TAG, "%s initialisation success", bme280p ? "BME280" : "BMP280");
        FLASH_LOGI("%s initialisation success", bme280p ? "BME280" : "BMP280");
    }else{
        mr->bmp280_status = SENSOR_INIT_FAIL;
        ESP_LOGW(TAG, "BMX280 initialisation fail: %s", esp_err_to_name(err));
        FLASH_LOGW("%s: %s initialisation fail", esp_err_to_name(err), bme280p ? "BME280" : "BMP280");
    }

    /* RADSENS Initialize */
    TryToInit = 0;
    do {
        err = (radsens_init_desc(0, I2C_SDA_IO, I2C_SCL_IO) == ESP_OK && 
            radsens_init() == ESP_OK) ? ESP_OK : ESP_FAIL;
        SSD1306_Init_Log((err == ESP_OK) ? RADSENS_OK : RADSENS_FAIL);
        if (err == ESP_OK) {
            break;
        }else {
            vTaskDelay(DELAY_INIT_MS / portTICK_RATE_MS);
        }
        TryToInit++;
    }while(TryToInit < MAX_TRY_INIT);
    if(err == ESP_OK){
        mr->radsens_status = SENSOR_OK;
        ESP_LOGI(TAG, "Radsens initialisation success");
        FLASH_LOGI("Radsens initialisation success");
    }else{
        mr->radsens_status = SENSOR_INIT_FAIL;
        ESP_LOGW(TAG, "Radsens initialisation fail");
        FLASH_LOGW("%s: RadSens initialisation fail", esp_err_to_name(err));
    }

    /* ZE08 Initialize */
    TryToInit = 0;
    do {
        err = ze08ch2o_init(ZE08CH2O_GPIO);
        SSD1306_Init_Log((err == ESP_OK) ? ZE08_OK : ZE08_FAIL);
        if (err == ESP_OK) {
            break;
        }else {
            vTaskDelay(DELAY_INIT_MS / portTICK_RATE_MS);
        }
        TryToInit++;
    }while(TryToInit < MAX_TRY_INIT);
    if(err == ESP_OK){
        mr->ze08_status = SENSOR_OK;
        ESP_LOGI(TAG, "ZE08 initialisation success");
        FLASH_LOGI("ZE08 initialisation success");
    }else{
        mr->ze08_status = SENSOR_INIT_FAIL;
        ESP_LOGW(TAG, "ZE08 initialisation fail");
        FLASH_LOGW("%s: ZE08 initialisation fail", esp_err_to_name(err));
    }

    /* MICS6814 Initialize */  
    TryToInit = 0;
    do {
        err = mics6814_init(MICS6814_MISO, MICS6814_SCLK, MICS6814_CS);
        SSD1306_Init_Log((err == ESP_OK) ? MICS6814_OK : MICS6814_FAIL);
        if (err == ESP_OK) {
            break;
        }else {
            vTaskDelay(DELAY_INIT_MS / portTICK_RATE_MS);
        }
        TryToInit++;
    }while(TryToInit < MAX_TRY_INIT);
    if (err == ESP_OK) {
        mr->mics6814_status = SENSOR_OK;
        ESP_LOGI(TAG, "MICS6814 initialisation success");
        FLASH_LOGI("MICS6814 initialisation success");
    }else{
        mr->mics6814_status = SENSOR_INIT_FAIL;
        ESP_LOGW(TAG, "MICS6814 initialisation fail");
        FLASH_LOGW("%s: MICS6814 initialisation fail", esp_err_to_name(err));
    }

    /* AHT25 Initialize */  
    aht25_dev.mode = AHT_MODE_NORMAL;
    aht25_dev.type = AHT_TYPE_AHT20;
    TryToInit = 0;
    do {
        err = (aht_init_desc(&aht25_dev, AHT_I2C_ADDRESS_GND, 0, I2C_SDA_IO, I2C_SCL_IO) == ESP_OK && 
            aht_init(&aht25_dev) == ESP_OK) ? ESP_OK : ESP_FAIL;  
        SSD1306_Init_Log((err == ESP_OK) ? AHT25_OK : AHT25_FAIL);
        if (err == ESP_OK) {
            break;
        }else {
            vTaskDelay(DELAY_INIT_MS / portTICK_RATE_MS);
        }
        TryToInit++;
    }while(TryToInit < MAX_TRY_INIT);
    if(err == ESP_OK){
        mr->aht25_status = SENSOR_OK;
        ESP_LOGI(TAG, "AHT25 initialisation success");
        FLASH_LOGI("AHT25 initialisation success");
    }else{
        mr->aht25_status = SENSOR_INIT_FAIL;
        ESP_LOGW(TAG, "AHT25 initialisation fail");
        FLASH_LOGW("%s: AHT25 initialisation fail", esp_err_to_name(err));
    }

    // Create orders task
    xTaskCreate(&get_orders_task, "get_orders_task", 0x1000, NULL, CONFIG_WIFI_MANAGER_TASK_PRIORITY-2, NULL);

    if (ssd1306_status != ESP_OK) {
        FLASH_LOGW("%s: SSD1306 initialisation fail", esp_err_to_name(ssd1306_status));
    }else {
        FLASH_LOGI("SSD1306 initialisation success");
        /* Create timer for press count */
        oled_sleep_timer = xTimerCreate( NULL, pdMS_TO_TICKS(OLED_SLIP_TIMER_MS), pdFALSE, ( void * ) 0, timer_cb);

        // Activate OLED sleep timer
        cb_down_shot();
    }

    // xTaskCreate(&check_mem_task, "check_mem_task", 2048, NULL, 10, NULL);

}
