/**
 * @file rest_handlers.h
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

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)
#define MAX_ERROR_TEXT (256)

typedef struct rest_server_context {
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
} rest_server_context_t;

esp_err_t wifi_access_points_get_handler(httpd_req_t *req);

esp_err_t esp32_setup_wifi_ca_post_handler(httpd_req_t *req);

esp_err_t esp32_setup_wifi_crt_post_handler(httpd_req_t *req);

esp_err_t esp32_setup_wifi_key_post_handler(httpd_req_t *req);

esp_err_t esp32_setup_wifi_post_handler(httpd_req_t *req);

esp_err_t esp32_setup_ipv4_post_handler(httpd_req_t *req);

esp_err_t esp32_setup_http_post_handler(httpd_req_t *req);

esp_err_t esp32_setup_http_ca_post_handler(httpd_req_t *req);

esp_err_t esp32_setup_http_crt_post_handler(httpd_req_t *req);

esp_err_t esp32_setup_http_key_post_handler(httpd_req_t *req);

#ifdef __cplusplus
}
#endif

/**@}*/
