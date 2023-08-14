/**
 * @file spiffs.h
 * @defgroup storage storage
 * @{
 *
 * ESP-IDF Repository spiffs
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
 * @brief Initialise spif file system
*/
esp_err_t init_spiffs(const char* mount_point, int max_files, bool check_on_start);


#ifdef __cplusplus
}
#endif

/**@}*/

