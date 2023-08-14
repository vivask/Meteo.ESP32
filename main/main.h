#pragma once

#include <stdint.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_OK                       (0)
#define SENSOR_FAIL                     (-1)
#define SENSOR_INIT_FAIL                (-2)

#define DELAY_ORDER_REQUEST_MS          (1000)  // 1 second

#define TICKS_MEASHURE_SEND             (3)
#define TICKS_FLASH_LOG_SEND            (10)

#define BMP280_OK                       "BMP280:      OK"
#define DS18B20_OK                      "DS18B20:     OK"
#define RADSENS_OK                      "RADSENS:     OK"
#define ZE08_OK                         "ZE08:        OK"
#define MICS6814_OK                     "MICS6814:    OK"
#define AHT25_OK                        "AHT25:       OK"
#define BMP280_FAIL                     "BMP280:    FAIL"
#define DS18B20_FAIL                    "DS18B20:   FAIL"
#define RADSENS_FAIL                    "RADSENS:   FAIL"
#define ZE08_FAIL                       "ZE08:      FAIL"
#define MICS6814_FAIL                   "MICS6814:  FAIL"
#define MICS6814_CAL 	                "MICS6814:   CAL"
#define AHT25_FAIL                      "AHT25:     FAIL"
#define TEXT_LEN                        sizeof(BME280_OK)

#define OLED_SLIP_TIMER_MS              (10000)  // 10 seconds

// JSON field name for orders
#define CMD_AP					"ap"			// Set wifi acces mode 
#define CMD_REBOOT				"reboot"		// Reboot
#define CMD_RADSENS_MODE		"r_mode"		// Radsens setup mode
#define CMD_RADSENS_STATE		"r_state"		// Radsens HV state
#define CMD_RADSENS_SENS		"r_sens"		// Radsens sensitivity
#define CMD_CLEAR_JOURNAL		"clear_j"		// Clear flash logging
#define CMD_DIGISPARK_REBOOT	"dg_reboot"		// Reboot digispark module

/**
 * @brief Define screen codes
 */
typedef enum _screen_code_t {
	SC_STATUS			= 0,
	SC_BMP280			= 1,
	SC_AHT25			= 2,
	SC_DS18B20			= 3,
	SC_RADSENS			= 4,
	SC_ZE08				= 5,
	SC_MISC6418			= 6,
	SCREEN_COUNT		= 7,	
}screen_code_t;

/**
 * @brief Define message codes for tx queue
 */
typedef enum _tx_message_code_t {
	TX_NONE         = 0,
	TX_RESPONSE     = 1,
	TX_REQUEST      = 2,
}tx_message_code_t;

/**
 * @brief Structure used to store one message in the queue.
 */
typedef struct{
	uint16_t code;
	void *param;
} queue_message_tx;

/**
 * @brief Structure used to store response meashure results
 */
typedef struct _meashure_results_t {
	int8_t	 		bmp280_status;
	float 			bmp280_pressure;
	float 			bmp280_temperature;
	float 			bmp280_humidity;

	int8_t 			ds18b20_status;
	char			ds18b20_address[17];
	float 			ds18b20_temperature;

	int8_t 			radsens_status;
	float 			radsens_i_static;
	float 			radsens_i_dynamic;
	uint32_t 		radsens_pulse;
	uint8_t			radsens_hv_state;
	uint8_t			radsens_sens;

	int8_t 			ze08_status;
	uint16_t 		ze08_ch2o;

	int8_t 			mics6814_status;
	float 			mics6814_co;
	float 			mics6814_nh3;
	float 			mics6814_no2;

	int8_t			aht25_status;
	float			aht25_temperature;
	float			aht25_humidity;
}meashure_results_t;

esp_err_t init_tasks();

#ifdef __cplusplus
}
#endif

/**@}*/

