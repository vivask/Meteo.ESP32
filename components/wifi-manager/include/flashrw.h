/**
 * @flashrw flashrw.h
 * @defgroup flashrw flashrw
 * @{
 *
 * ESP-IDF Read and write data flash store
 *
 */
#pragma once

#include <stdint.h>
#include <esp_err.h>


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief saves to flash config file
*/
esp_err_t save_flash_data(void* data, const size_t sz, const char* fName);

/**
 * @brief saves to flash config file
*/
esp_err_t save_flash_json_data(const char* json, const char* fName);

/**
 * @brief read flashed file
*/
char* read_flash_json_data(const char* fName);

/**
 * @brief Return true if file exist
*/
bool is_flash_file_exist(const char* name);

/**
 * @brief clear file context
*/
void clear_flash_file(const char* name);

#ifdef __cplusplus
}

#endif

/**@}*/

