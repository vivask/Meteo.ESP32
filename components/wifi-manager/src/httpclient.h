#ifndef __HTTPCLIENT_H__
#define __HTTPCLIENT_H__


#ifdef __cplusplus
extern "C" {
#endif

typedef enum _device_state_t{
    ON = 1,
    OFF = 2,
    CALIBRATED = 3,
    ERROR = 4,
    UNKNOWN = 0,
}device_state_t;

typedef struct _peripheral_state_t{
    device_state_t i2c;
    device_state_t radsens;
    device_state_t ds18b20;
    device_state_t bmx280;
    device_state_t ze08ch2o;
    device_state_t mics6814;
}peripheral_state_t;

typedef struct _subsystem_state_t{
    device_state_t ntp;
}subsystem_state_t;

void https_client_init(const uint8_t* server_address, const uint16_t server_port);
void https_client_start();
void https_client_stop();
void flash_read_start();
void flash_read_stop();
esp_err_t https_check(const char* msg);
esp_err_t https_logi(const char* format, ...);
esp_err_t https_loge(const char* format, ...);
esp_err_t https_logw(const char* format, ...);
bool https_client_active();
char* https_get_datetime(char* date_time);
void https_ds18b20_send_data(const char* sensor_addr, const float tempr);
void https_bme280_send_data(const float t, const float h, const float p);
void https_radsens_send_data(const float ri_dynamic, const float ri_static, const uint32_t pulses);
void https_ze08_send_data(const uint16_t ch2o);
void https_6814_send_data(const float nh3, const float co, const float no2);

void device_set_state(device_state_t* device, const esp_err_t esp_err);
void subsystem_set_state(device_state_t* device, const esp_err_t esp_err);

void https_client_set_start_callback(void (*func_ptr)(void*) );
void https_client_set_stop_callback(void (*func_ptr)(void*) );
void https_client_set_stop_peripheral(void (*func_ptr)(void*) );
void https_client_set_start_peripheral(void (*func_ptr)(void*) );
void https_client_set_drop_peripheral(void (*func_ptr)(void*) );

bool isSntpOk();

void initialize_cpu_monitor();
float get_cpu0_load_percent();
float get_cpu1_load_percent();

#ifdef __cplusplus
}
#endif

#endif /* __HTTPCLIENT_H__ */