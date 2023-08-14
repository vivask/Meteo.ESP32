/**
 * @file ntp_client.h
 * @defgroup ntp ntp
 * @{
 *
 * ESP-IDF NTP Client
 *
 * Ported from esp-open-rtos
 *
 * MIT Licensed as described in the file LICENSE
 */
#pragma once

#include <stdint.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize ntp client
 * @return true if initialize success, false otherwise.
 */
esp_err_t initialize_ntp(const char* timezone, const char* ntp_server_address);

/**
 * @brief Get curren system time
 */
time_t get_local_datetime();

/**
 * @brief Get time initialization
 */
time_t get_sntp_time_init();

/**
 * @brief Get ntp client status
 * @return true if ntp client initialize success, false otherwise.
 */
bool get_sntp_status();

time_t get_sntp_time_init();
time_t get_sntp_tiks_init();

char* date_time_format(char* date_time, time_t t);

#ifdef __cplusplus
}

#endif

/**@}*/

