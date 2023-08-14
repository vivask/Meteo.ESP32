/**
 * @http_client http_client.h
 * @defgroup http_client http_client
 * @{
 *
 * ESP-IDF HTTP Client
 *
 */
#pragma once

#include <stdint.h>
#include <esp_err.h>
#include <esp_http_client.h>
#include "freertos/event_groups.h"

#define MAX_HTTP_URI_LEN       	64
#define MAX_REQUEST_DATA_LEN	1024

#define HC_STATUS_OK  	BIT0	// Set if http connection established
#define HC_WIFI_OK		BIT1	// Set if wifi connection established
#define HC_SEND_OK  	BIT2	// Set if sending completed successfully
#define HC_SEND_FAIL  	BIT3	// Set if send failed

#ifdef __cplusplus
extern "C" {
#endif

typedef enum order_code_t {
	HC_NONE = 0,
	HC_ORDER_CONECT = 1,
	HC_ORDER_DISCONNECT = 2,
	HC_MESSAGE_CODE_COUNT = 3 /* important for the callback array */
}order_code_t;


/**
 * @brief Structure used to store one message in the queue.
 */
typedef struct _http_client_request_t {
	uint16_t 	method;
	char 		uri[MAX_HTTP_URI_LEN];
	char 		data[MAX_REQUEST_DATA_LEN];
}http_client_request_t;

/**
 * @brief Structure used to store one message http request.
 */
typedef struct _http_client_response_t {
	int 					status;
	char* 					error;
	char* 					data;
	http_client_request_t* request;
}http_client_response_t;

/**
 * Initialize http client
 */
void http_client_initialize();

/**
 * @brief Register a callback to a custom function when specific event response happens.
 */
void http_client_set_response_callback( void (*func_ptr)(void*) );

/**
 * @brief Register a callback to a custom function when specific event ready happens.
 */
void http_client_set_ready_callback(void (*func_ptr)(void*) );

/**
 * @brief Register a callback to a custom function when specific event not ready happens.
 */
void http_client_set_not_ready_callback(void (*func_ptr)(void*) );

/**
 * @brief Send http request
 */
BaseType_t http_client_send_message_to_front(uint16_t method, const char* uri, const char* data);
BaseType_t http_client_send_message(uint16_t method, const char* uri, const char* data);

#ifdef __cplusplus
}

#endif

/**@}*/

