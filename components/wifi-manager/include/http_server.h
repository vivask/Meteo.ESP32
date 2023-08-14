/**
 * @file rest.h
 * @defgroup rest rest
 * @{
 *
 * ESP-IDF REST Server
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

esp_err_t start_rest_server(const char *base_path);

void stop_rest_server();

#ifdef __cplusplus
}
#endif

/**@}*/

