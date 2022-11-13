/*
Copyright (c) 2017-2020 Tony Pottier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

@file http_app.c
@author Tony Pottier
@brief Defines all functions necessary for the HTTP server to run.

Contains the freeRTOS task for the HTTP listener and all necessary support
function to process requests, decode URLs, serve files, etc. etc.

@note http_server task cannot run without the wifi_manager task!
@see https://idyl.io
@see https://github.com/tonyp7/esp32-wifi-manager
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include "esp_netif.h"
#include <esp_http_server.h>

#include "wifi_manager.h"
#include "http_app.h"


/* @brief tag used for ESP serial console messages */
static const char TAG[] = "http_server";

/* @brief the HTTP server handle */
static httpd_handle_t httpd_handle = NULL;

/* function pointers to URI handlers that can be user made */
esp_err_t (*custom_get_httpd_uri_handler)(httpd_req_t *r) = NULL;
esp_err_t (*custom_post_httpd_uri_handler)(httpd_req_t *r) = NULL;

/* strings holding the URLs of the wifi manager */
static char* http_root_url = NULL;
static char* http_redirect_url = NULL;
static char* http_js_url = NULL;
static char* http_css_url = NULL;
static char* http_connect_url = NULL;
static char* http_ap_url = NULL;
static char* http_status_url = NULL;

static httpd_config_t* httpd_config = NULL;

SemaphoreHandle_t http_app_mutex = NULL;


/**
 * @brief embedded binary data.
 * @see file "component.mk"
 * @see https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#embedding-binary-data
 */
extern const uint8_t style_css_start[] asm("_binary_style_css_start");
extern const uint8_t style_css_end[]   asm("_binary_style_css_end");
extern const uint8_t code_js_start[] asm("_binary_code_js_start");
extern const uint8_t code_js_end[] asm("_binary_code_js_end");
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

/* const httpd related values stored in ROM */
const static char http_200_hdr[] = "200 OK";
const static char http_302_hdr[] = "302 Found";
const static char http_400_hdr[] = "400 Bad Request";
const static char http_404_hdr[] = "404 Not Found";
const static char http_503_hdr[] = "503 Service Unavailable";
const static char http_location_hdr[] = "Location";
const static char http_content_type_html[] = "text/html";
const static char http_content_type_js[] = "text/javascript";
const static char http_content_type_css[] = "text/css";
//const static char http_content_type_json[] = "application/json";
const static char http_content_type_json[] = "multipart/form-data";
const static char http_cache_control_hdr[] = "Cache-Control";
const static char http_cache_control_no_cache[] = "no-store, no-cache, must-revalidate, max-age=0";
const static char http_cache_control_cache[] = "public, max-age=31536000";
const static char http_pragma_hdr[] = "Pragma";
const static char http_pragma_no_cache[] = "no-cache";


esp_err_t http_app_set_handler_hook( httpd_method_t method,  esp_err_t (*handler)(httpd_req_t *r)  ){

	if(method == HTTP_GET){
		custom_get_httpd_uri_handler = handler;
		return ESP_OK;
	}
	else if(method == HTTP_POST){
		custom_post_httpd_uri_handler = handler;
		return ESP_OK;
	}
	else{
		return ESP_ERR_INVALID_ARG;
	}

}


static esp_err_t http_server_delete_handler(httpd_req_t *req){

	ESP_LOGI(TAG, "DELETE %s", req->uri);

	/* DELETE /connect.json */
	if(strcmp(req->uri, http_connect_url) == 0){
		wifi_manager_disconnect_async();

		httpd_resp_set_status(req, http_200_hdr);
		httpd_resp_set_type(req, http_content_type_json);
		httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_no_cache);
		httpd_resp_set_hdr(req, http_pragma_hdr, http_pragma_no_cache);
		httpd_resp_send(req, NULL, 0);
	}
	else{
		httpd_resp_set_status(req, http_404_hdr);
		httpd_resp_send(req, NULL, 0);
	}

	return ESP_OK;
}

static bool get_file_from_content(uint8_t* dst, const char* src, const char* name, const size_t size)
{
	char* ptr;
	char pattern[32] = "name=\"";
	strcat(pattern, name);
	ptr = strstr(src, pattern);
	if(ptr){
		const char* x509_content = "Content-Type: application/x-x509-ca-cert";
		ptr = strstr(ptr, x509_content);
		if(ptr){
			ptr += strlen(x509_content) + 4;
			memcpy(dst, ptr, size);
			dst[size] = '\0';
			return true;
		}
	}
	return false;
}

static bool get_pkix_file_from_content(uint8_t* dst, const char* src, const char* name, const size_t size)
{
	char* ptr;
	char pattern[32] = "name=\"";
	strcat(pattern, name);
	ptr = strstr(src, pattern);
	if(ptr){
		const char* pkix_content = "Content-Type: application/pkix-cert";
		ptr = strstr(ptr, pkix_content);
		if(ptr){
			ptr += strlen(pkix_content) + 4;
			memcpy(dst, ptr, size);
			dst[size] = '\0';
			return true;
		}
	}
	return false;
}


static bool get_key_file_from_content(uint8_t* dst, const char* src, const char* name, const size_t size)
{
	char* ptr;
	char pattern[32] = "name=\"";
	strcat(pattern, name);
	ptr = strstr(src, pattern);
	if(ptr){
		const char* pkix_content = "Content-Type: application/vnd.apple.keynote";
		ptr = strstr(ptr, pkix_content);
		if(ptr){
			ptr += strlen(pkix_content) + 4;
			memcpy(dst, ptr, size);
			dst[size] = '\0';
			return true;
		}
	}
	return false;
}

/*
static bool cmpbufs(const uint8_t* buf1, const uint8_t* buf2, const size_t size)
{
	for(int i=0; i < size; i++) if(buf1[i] != buf2[i]) return false;
	return true;
}*/

static inline size_t get_file_size(const size_t size){
	return (size) ? size+1 : 0;
}

static esp_err_t http_server_post_handler(httpd_req_t *req){


	esp_err_t ret = ESP_OK;

	ESP_LOGI(TAG, "POST uri %s", req->uri);

	/* POST /connect.json */
	if(strcmp(req->uri, http_connect_url) == 0){


		/* buffers for the headers */
		size_t recived, remaining = req->content_len;
		size_t ssid_len = 0, password_len = 0;
		size_t security_len = 0, authentication_len = 0;
		size_t identity_len = 0, ca_file_size_len = 0;
		size_t peap_user_len = 0, peap_passwd_len = 0;
		size_t ttls_auth_len = 0, ttls_user_len = 0, ttls_passwd_len = 0;
		size_t tls_crt_file_size_len = 0, tls_key_file_size_len = 0;
		size_t httpd_address_len = 0, httpd_port_len = 0;
		size_t httpd_ca_len = 0, httpd_crt_len = 0, httpd_key_len = 0;
		size_t httpd_user_len = 0, httpd_passw_len = 0;
		size_t ipv4_addr_len = 0, ipv4_mask_len = 0, ipv4_gate_len= 0;
		size_t ipv4_dns_len = 0, ipv4_ntp_len = 0, ipv4_timezone_len = 0;

		char *content = NULL;
		uint8_t *ssid = NULL, *password = NULL;
		uint8_t *security = NULL, *authentication = NULL;
		uint8_t *identity = NULL, *ca_file_size = NULL;
		uint8_t *peap_user = NULL, *peap_passwd = NULL;
		uint8_t *ttls_auth = NULL, *ttls_user = NULL, *ttls_passwd = NULL;
		uint8_t *tls_crt_file_size = NULL, *tls_key_file_size = NULL;
		uint8_t *ca_file = NULL, *crt_file = NULL, *key_file = NULL;
		uint8_t *http_address = NULL, *ch_httpd_port = NULL, *ca_httpd = NULL, *crt_httpd = NULL, *key_httpd = NULL;
		uint8_t *ch_httpd_ca_len = NULL, *ch_httpd_crt_len = NULL, *ch_httpd_key_len = NULL;
		uint8_t *httpd_user = NULL, *httpd_passw = NULL;
		uint8_t *ipv4_addr = NULL, *ipv4_mask = NULL, *ipv4_gate = NULL;
		uint8_t *ipv4_dns = NULL, *ipv4_ntp = NULL, *ipv4_timezone = NULL;


		/* len of values provided */
		ssid_len = httpd_req_get_hdr_value_len(req, "X-Custom-ssid");
		password_len = httpd_req_get_hdr_value_len(req, "X-Custom-pwd");
		security_len = httpd_req_get_hdr_value_len(req, "X-Custom-security");
		authentication_len = httpd_req_get_hdr_value_len(req, "X-Custom-authentication");
		identity_len = httpd_req_get_hdr_value_len(req, "X-Custom-identity");
		ca_file_size_len = httpd_req_get_hdr_value_len(req, "X-Custom-ca-file-size");
		peap_user_len = httpd_req_get_hdr_value_len(req, "X-Custom-peap-user");
		peap_passwd_len = httpd_req_get_hdr_value_len(req, "X-Custom-peap-passwd");
		ttls_auth_len = httpd_req_get_hdr_value_len(req, "X-Custom-ttls-auth");
		ttls_user_len = httpd_req_get_hdr_value_len(req, "X-Custom-ttls-user");
		ttls_passwd_len = httpd_req_get_hdr_value_len(req, "X-Custom-ttls-passwd");
		tls_crt_file_size_len = httpd_req_get_hdr_value_len(req, "X-Custom-tls-crt-file-size");
		tls_key_file_size_len = httpd_req_get_hdr_value_len(req, "X-Custom-tls-key-file-size");
		httpd_address_len = httpd_req_get_hdr_value_len(req, "X-Custom-httpd-address");
		httpd_port_len = httpd_req_get_hdr_value_len(req, "X-Custom-httpd-port");
		httpd_ca_len = httpd_req_get_hdr_value_len(req, "X-Custom-httpd-ca-size");
		httpd_crt_len = httpd_req_get_hdr_value_len(req, "X-Custom-httpd-crt-size");
		httpd_key_len = httpd_req_get_hdr_value_len(req, "X-Custom-httpd-key-size");
		httpd_user_len = httpd_req_get_hdr_value_len(req, "X-Custom-httpd-username");
		httpd_passw_len = httpd_req_get_hdr_value_len(req, "X-Custom-httpd-password");
		ipv4_addr_len = httpd_req_get_hdr_value_len(req, "X-Custom-ipv4-addr");
		ipv4_mask_len = httpd_req_get_hdr_value_len(req, "X-Custom-ipv4-mask");
		ipv4_gate_len = httpd_req_get_hdr_value_len(req, "X-Custom-ipv4-gate");
		ipv4_dns_len = httpd_req_get_hdr_value_len(req, "X-Custom-ipv4-dns");
		ipv4_ntp_len = httpd_req_get_hdr_value_len(req, "X-Custom-ipv4-ntp");
		ipv4_timezone_len = httpd_req_get_hdr_value_len(req, "X-Custom-ipv4-timezone");

		ESP_LOGD(TAG, "ssid_len: %d, content_len: %d", ssid_len, req->content_len);

		if(ssid_len && ssid_len <= MAX_SSID_SIZE && password_len <= MAX_PASSWORD_SIZE){

			/* get the actual value of the headers */
			content = malloc(sizeof(char) * (req->content_len + 1));
			ssid = malloc(sizeof(uint8_t) * (ssid_len + 1));
			password = malloc(sizeof(uint8_t) * (password_len + 1));
			security = malloc(sizeof(uint8_t) * (security_len + 1));
			authentication = malloc(sizeof(uint8_t) * (authentication_len + 1));
			identity = malloc(sizeof(uint8_t) * (identity_len + 1));
			ca_file_size = malloc(sizeof(uint8_t) * (ca_file_size_len + 1));
			peap_user = malloc(sizeof(uint8_t) * (peap_user_len + 1));
			peap_passwd = malloc(sizeof(uint8_t) * (peap_passwd_len + 1));
			ttls_auth = malloc(sizeof(uint8_t) * (ttls_auth_len + 1));
			ttls_user = malloc(sizeof(uint8_t) * (ttls_user_len + 1));
			ttls_passwd = malloc(sizeof(uint8_t) * (ttls_passwd_len + 1));
			tls_crt_file_size = malloc(sizeof(uint8_t) * (tls_crt_file_size_len + 1));
			tls_key_file_size = malloc(sizeof(uint8_t) * (tls_key_file_size_len + 1));
			http_address = malloc(sizeof(uint8_t) * (httpd_address_len + 1));
			ch_httpd_port = malloc(sizeof(uint8_t) * (httpd_port_len + 1));
			ch_httpd_ca_len = malloc(sizeof(uint8_t) * (httpd_ca_len + 1));
			ch_httpd_crt_len = malloc(sizeof(uint8_t) * (httpd_crt_len + 1));
			ch_httpd_key_len = malloc(sizeof(uint8_t) * (httpd_key_len + 1));
			httpd_user = malloc(sizeof(uint8_t) * (httpd_user_len + 1));
			httpd_passw = malloc(sizeof(uint8_t) * (httpd_passw_len + 1));
			ipv4_addr = malloc(sizeof(uint8_t) * (ipv4_addr_len + 1));
			ipv4_mask = malloc(sizeof(uint8_t) * (ipv4_mask_len + 1));
			ipv4_gate = malloc(sizeof(uint8_t) * (ipv4_gate_len + 1));
			ipv4_dns = malloc(sizeof(uint8_t) * (ipv4_dns_len + 1));
			ipv4_ntp = malloc(sizeof(uint8_t) * (ipv4_ntp_len + 1));
			ipv4_timezone = malloc(sizeof(uint8_t) * (ipv4_timezone_len + 1));

			httpd_req_get_hdr_value_str(req, "X-Custom-ssid", (char*)ssid, ssid_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-pwd", (char*)password, password_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-security", (char*)security, security_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-authentication", (char*)authentication, authentication_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-identity", (char*)identity, identity_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-ca-file-size", (char*)ca_file_size, ca_file_size_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-peap-user", (char*)peap_user, peap_user_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-peap-passwd", (char*)peap_passwd, peap_passwd_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-ttls-auth", (char*)ttls_auth, ttls_auth_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-ttls-user", (char*)ttls_user, ttls_user_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-ttls-passwd", (char*)ttls_passwd, ttls_passwd_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-tls-crt-file-size", (char*)tls_crt_file_size, tls_crt_file_size_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-tls-key-file-size", (char*)tls_key_file_size, tls_key_file_size_len+1);

			httpd_req_get_hdr_value_str(req, "X-Custom-httpd-address", (char*)http_address, httpd_address_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-httpd-port", (char*)ch_httpd_port, httpd_port_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-httpd-ca-size", (char*)ch_httpd_ca_len, httpd_ca_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-httpd-crt-size", (char*)ch_httpd_crt_len, httpd_crt_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-httpd-key-size", (char*)ch_httpd_key_len, httpd_key_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-httpd-username", (char*)httpd_user, httpd_user_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-httpd-password", (char*)httpd_passw, httpd_passw_len+1);

			httpd_req_get_hdr_value_str(req, "X-Custom-ipv4-addr", (char*)ipv4_addr, ipv4_addr_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-ipv4-mask", (char*)ipv4_mask, ipv4_mask_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-ipv4-gate", (char*)ipv4_gate, ipv4_gate_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-ipv4-dns", (char*)ipv4_dns, ipv4_dns_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-ipv4-ntp", (char*)ipv4_ntp, ipv4_ntp_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-ipv4-timezone", (char*)ipv4_timezone, ipv4_timezone_len+1);


			char *ptr = content;
			while(remaining > 0){
				if((recived = httpd_req_recv(req, ptr, remaining)) <= 0){
					if(recived == HTTPD_SOCK_ERR_TIMEOUT){
						continue;
					}
					return ESP_FAIL;
				}
				ESP_LOGI(TAG, "Rceived len: %d", recived);
				httpd_resp_send_chunk(req, ptr, recived);
				remaining -= recived;
				ptr += recived;
			}

			size_t ca_file_len = atoi((char*)ca_file_size);
			size_t crt_file_len = atoi((char*)tls_crt_file_size);
			size_t key_file_len = atoi((char*)tls_key_file_size);
			size_t ca_httpd_len = atoi((char*)ch_httpd_ca_len);
			size_t crt_httpd_len = atoi((char*)ch_httpd_crt_len);
			size_t key_httpd_len = atoi((char*)ch_httpd_key_len);

			ca_file = malloc(sizeof(uint8_t) * (ca_file_len + 1));
			crt_file = malloc(sizeof(uint8_t) * (crt_file_len + 1));
			key_file = malloc(sizeof(uint8_t) * (key_file_len + 1));
			ca_httpd = malloc(sizeof(uint8_t) * (ca_httpd_len + 1));
			crt_httpd = malloc(sizeof(uint8_t) * (crt_httpd_len + 1));
			key_httpd = malloc(sizeof(uint8_t) * (key_httpd_len + 1));

			ESP_LOGD(TAG, "Content size: %d", req->content_len);
			ESP_LOGD(TAG, "%s", content);

			get_file_from_content(ca_file, content, "ca_file", ca_file_len);
			ESP_LOGD(TAG, "CA file size: %d", ca_file_len);
			ESP_LOGD(TAG, "%s", ca_file);

			get_file_from_content(crt_file, content, "crt_file", crt_file_len);
			ESP_LOGD(TAG, "CRT file size: %d", crt_file_len);
			ESP_LOGD(TAG, "%s", crt_file);

			get_file_from_content(key_file, content, "key_file", key_file_len);
			ESP_LOGD(TAG, "KEY file size: %d", key_file_len);
			ESP_LOGD(TAG, "%s", key_file);

			get_pkix_file_from_content(ca_httpd, content, "httpd_ca", ca_httpd_len);
			ESP_LOGD(TAG, "HTTPD CA file size: %d", ca_httpd_len);
			ESP_LOGD(TAG, "%s", ca_httpd);

			get_pkix_file_from_content(crt_httpd, content, "httpd_crt", crt_httpd_len);
			ESP_LOGD(TAG, "HTTPD Client crt file size: %d", crt_httpd_len);
			ESP_LOGD(TAG, "%s", crt_httpd);

			get_key_file_from_content(key_httpd, content, "httpd_key", key_httpd_len);
			ESP_LOGD(TAG, "HTTPD Client key file size: %d", key_httpd_len);
			ESP_LOGD(TAG, "%s", key_httpd);

			wifi_config_t* config = wifi_manager_get_wifi_sta_config();
			memset(config, 0x00, sizeof(wifi_config_t));
			memcpy(config->sta.ssid, ssid, ssid_len);

			httpd_settings_t httpd;
			memset(&httpd, 0x00, sizeof(httpd_settings_t));
			httpd.address = http_address;
			httpd.address_bytes = httpd_address_len;
			httpd.port = atoi((char*)ch_httpd_port);
			httpd.ca = ca_httpd;
			httpd.ca_bytes = get_file_size(ca_httpd_len);
			httpd.client_crt = crt_httpd;
			httpd.client_crt_bytes = get_file_size(crt_httpd_len);
			httpd.client_key = key_httpd;
			httpd.client_key_bytes = get_file_size(key_httpd_len);
			httpd.username = httpd_user;
			httpd.username_bytes = httpd_user_len;
			httpd.password = httpd_passw;
			httpd.password_bytes = httpd_passw_len;
			wifi_manager_set_httpd_config(&httpd);

			ESP_LOGD(TAG, "ipv4_adrr %s, ipv4_addr_len = %d", ipv4_addr, ipv4_addr_len);			
			ESP_LOGD(TAG, "ipv4_mask %s, ipv4_mask_len = %d", ipv4_mask, ipv4_mask_len);			
			ESP_LOGD(TAG, "ipv4_gate %s, ipv4_gate_len = %d", ipv4_gate, ipv4_gate_len);			
			ESP_LOGD(TAG, "ipv4_dns %s, ipv4_dns_len = %d", ipv4_dns, ipv4_dns_len);			
			ESP_LOGD(TAG, "ipv4_ntp %s, ipv4_ntp_len = %d", ipv4_ntp, ipv4_ntp_len);			
			ESP_LOGD(TAG, "ipv4_timezone %s, ipv4_ntp_len = %d", ipv4_timezone, ipv4_timezone_len);			

			ipv4_settings_t ipv4;
			memset(&httpd, 0x00, sizeof(httpd_settings_t));
			ipv4.address = ipv4_addr;
			ipv4.address_bytes = ipv4_addr_len;
			ipv4.mask= ipv4_mask;
			ipv4.mask_bytes = ipv4_mask_len;
			ipv4.gate = ipv4_gate;
			ipv4.gate_bytes = ipv4_gate_len;
			ipv4.dns = ipv4_dns;
			ipv4.dns_bytes = ipv4_dns_len;
			ipv4.ntp = ipv4_ntp;
			ipv4.ntp_bytes = ipv4_ntp_len;
			ipv4.timezone = ipv4_timezone;
			ipv4.timezone_bytes = ipv4_timezone_len;
			wifi_manager_set_ipv4_config(&ipv4);

			if(strcmp((char*)security, "Personal") == 0){
				memcpy(config->sta.password, password, password_len);
				ESP_LOGD(TAG, "ssid: %s, password: %s", ssid, password);
				wifi_manager_connect_async(true);
			}else if(strcmp((char*)security, "Enterprise") == 0){
				wpa_enterprise_settings_t wpa_set;
				memset(&wpa_set, 0x00, sizeof(wpa_enterprise_settings_t));
				wpa_set.ssid = ssid;
				wpa_set.ssid_bytes = ssid_len;
				wpa_set.identity = identity;
				wpa_set.identity_bytes = identity_len;
				wpa_set.ca_file = ca_file;
				wpa_set.ca_file_bytes = get_file_size(ca_file_len);
				
				if(strcmp((char*)authentication, "TLS") == 0){
					wpa_set.authentication = WPA_TLS;
					wpa_set.crt_file = crt_file;
					wpa_set.crt_file_bytes = get_file_size(crt_file_len);
					wpa_set.key_file = key_file;
					wpa_set.key_file_bytes = get_file_size(key_file_len);
				}else if(strcmp((char*)authentication, "TTLS") == 0){
					wpa_set.authentication = WPA_TTLS;
					wpa_set.username = ttls_user;
					wpa_set.username_bytes = ttls_user_len;
					wpa_set.password = ttls_passwd;
					wpa_set.password_bytes = ttls_passwd_len;
					if(strcmp((char*)ttls_auth, "MSCHAPV2") == 0){
						wpa_set.ttlsauth = MSCHAPV2;
					}else if(strcmp((char*)ttls_auth, "MSCHAP") == 0){
						wpa_set.ttlsauth = MSCHAP;
					}else if(strcmp((char*)ttls_auth, "PAP") == 0){
						wpa_set.ttlsauth = PAP;
					}else if(strcmp((char*)ttls_auth, "CHAP") == 0){
						wpa_set.ttlsauth = CHAP;
					}else{
						ESP_LOGE(TAG, "Unknown TTLS authentication method: %s", ttls_auth);
						ret = ESP_FAIL;
					}
				}else  if(strcmp((char*)authentication, "PEAP") == 0){
					wpa_set.authentication = WPA_PEAP;
					wpa_set.username = peap_user;
					wpa_set.username_bytes = peap_user_len;
					wpa_set.password = peap_passwd;
					wpa_set.password_bytes = peap_passwd_len;
				}else{
					ESP_LOGE(TAG, "Unknown authentication method: %s", authentication);
					ret = ESP_FAIL;
				}
				if(ret != ESP_FAIL){
					ESP_LOGW(TAG, "http_server_post_handler: wifi_manager_connect_async() call ENTERPRISE");					
					wifi_manager_set_enterprise_config(&wpa_set);
					wifi_manager_connect_async(false);							
				}
			}else{
				ESP_LOGE(TAG, "Unknown security type: %s", security);
				ret = ESP_FAIL;
			}

			/* free memory */
			free(content);
			free(ssid);
			free(password);
			free(security);
			free(authentication);
			free(identity);
			free(ca_file_size);
			free(peap_user);
			free(peap_passwd);
			free(ttls_auth);
			free(ttls_user);
			free(ttls_passwd);
			free(tls_crt_file_size);
			free(tls_key_file_size);
			free(ca_file);
			free(crt_file);
			free(key_file);
			free(http_address);
			free(ch_httpd_port);
			free(ca_httpd);
			free(crt_httpd);
			free(key_httpd);
			free(ch_httpd_ca_len);
			free(ch_httpd_crt_len);
			free(ch_httpd_key_len);
			free(httpd_user);
			free(httpd_passw);
			free(ipv4_addr);
			free(ipv4_mask);
			free(ipv4_gate);
			free(ipv4_dns);
			free(ipv4_ntp);

			httpd_resp_set_status(req, http_200_hdr);
			httpd_resp_set_type(req, http_content_type_json);
			httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_no_cache);
			httpd_resp_set_hdr(req, http_pragma_hdr, http_pragma_no_cache);
			httpd_resp_send(req, NULL, 0);

		}
		else{
			/* bad request the authentification header is not complete/not the correct format */
			httpd_resp_set_status(req, http_400_hdr);
			httpd_resp_send(req, NULL, 0);
		}

	}
	else{

		if(custom_post_httpd_uri_handler == NULL){
			httpd_resp_set_status(req, http_404_hdr);
			httpd_resp_send(req, NULL, 0);
		}
		else{

			/* if there's a hook, run it */
			ret = (*custom_post_httpd_uri_handler)(req);
		}
	}

	return ret;
}


static esp_err_t http_server_get_handler(httpd_req_t *req){

    char* host = NULL;
    size_t buf_len;
    esp_err_t ret = ESP_OK;

    ESP_LOGD(TAG, "GET %s", req->uri);

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
    	host = malloc(buf_len);
    	if(httpd_req_get_hdr_value_str(req, "Host", host, buf_len) != ESP_OK){
    		/* if something is wrong we just 0 the whole memory */
    		memset(host, 0x00, buf_len);
    	}
    }

	/* determine if Host is from the STA IP address */
	wifi_manager_lock_sta_ip_string(portMAX_DELAY);
	bool access_from_sta_ip = host != NULL?strstr(host, wifi_manager_get_sta_ip_string()):false;
	wifi_manager_unlock_sta_ip_string();


	if (host != NULL && !strstr(host, DEFAULT_AP_IP) && !access_from_sta_ip) {

		/* Captive Portal functionality */
		/* 302 Redirect to IP of the access point */
		httpd_resp_set_status(req, http_302_hdr);
		httpd_resp_set_hdr(req, http_location_hdr, http_redirect_url);
		httpd_resp_send(req, NULL, 0);

	}
	else{

		/* GET /  */
		if(strcmp(req->uri, http_root_url) == 0){
			httpd_resp_set_status(req, http_200_hdr);
			httpd_resp_set_type(req, http_content_type_html);
			httpd_resp_send(req, (char*)index_html_start, index_html_end - index_html_start);
		}
		/* GET /code.js */
		else if(strcmp(req->uri, http_js_url) == 0){
			httpd_resp_set_status(req, http_200_hdr);
			httpd_resp_set_type(req, http_content_type_js);
			httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_cache);
			httpd_resp_send(req, (char*)code_js_start, code_js_end - code_js_start);
		}
		/* GET /style.css */
		else if(strcmp(req->uri, http_css_url) == 0){
			httpd_resp_set_status(req, http_200_hdr);
			httpd_resp_set_type(req, http_content_type_css);
			httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_cache);
			httpd_resp_send(req, (char*)style_css_start, style_css_end - style_css_start);
		}
		/* GET /ap.json */
		else if(strcmp(req->uri, http_ap_url) == 0){

			/* if we can get the mutex, write the last version of the AP list */
			if(wifi_manager_lock_json_buffer(( TickType_t ) 10)){

				httpd_resp_set_status(req, http_200_hdr);
				httpd_resp_set_type(req, http_content_type_json);
				httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_no_cache);
				httpd_resp_set_hdr(req, http_pragma_hdr, http_pragma_no_cache);
				char* ap_buf = wifi_manager_get_ap_list_json();
				httpd_resp_send(req, ap_buf, strlen(ap_buf));
				wifi_manager_unlock_json_buffer();
			}
			else{
				httpd_resp_set_status(req, http_503_hdr);
				httpd_resp_send(req, NULL, 0);
				ESP_LOGE(TAG, "http_server_netconn_serve: GET /ap.json failed to obtain mutex");
			}

			/* request a wifi scan */
			wifi_manager_scan_async();
		}
		/* GET /status.json */ 
		else if(strcmp(req->uri, http_status_url) == 0){

			if(wifi_manager_lock_json_buffer(( TickType_t ) 10)){
				char *buff = wifi_manager_get_ip_info_json();
				if(buff){
					httpd_resp_set_status(req, http_200_hdr);
					httpd_resp_set_type(req, http_content_type_json);
					httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_no_cache);
					httpd_resp_set_hdr(req, http_pragma_hdr, http_pragma_no_cache);
					httpd_resp_send(req, buff, strlen(buff));
					wifi_manager_unlock_json_buffer();
				}
				else{
					httpd_resp_set_status(req, http_503_hdr);
					httpd_resp_send(req, NULL, 0);
				}
			}
			else{
				httpd_resp_set_status(req, http_503_hdr);
				httpd_resp_send(req, NULL, 0);
				ESP_LOGE(TAG, "http_server_netconn_serve: GET /status.json failed to obtain mutex");
			}
		}
		else{

			if(custom_get_httpd_uri_handler == NULL){
				httpd_resp_set_status(req, http_404_hdr);
				httpd_resp_send(req, NULL, 0);
			}
			else{

				/* if there's a hook, run it */
				ret = (*custom_get_httpd_uri_handler)(req);
			}
		}

	}

    /* memory clean up */
    if(host != NULL){
    	free(host);
    }

    return ret;

}

/* URI wild card for any GET request */
static const httpd_uri_t http_server_get_request = {
    .uri       = "*",
    .method    = HTTP_GET,
    .handler   = http_server_get_handler
};

static const httpd_uri_t http_server_post_request = {
	.uri	= "*",
	.method = HTTP_POST,
	.handler = http_server_post_handler
};

static const httpd_uri_t http_server_delete_request = {
	.uri	= "*",
	.method = HTTP_DELETE,
	.handler = http_server_delete_handler
};


void http_app_stop(){

	if(httpd_handle != NULL){


		/* dealloc URLs */
		if(http_root_url) {
			free(http_root_url);
			http_root_url = NULL;
		}
		if(http_redirect_url){
			free(http_redirect_url);
			http_redirect_url = NULL;
		}
		if(http_js_url){
			free(http_js_url);
			http_js_url = NULL;
		}
		if(http_css_url){
			free(http_css_url);
			http_css_url = NULL;
		}
		if(http_connect_url){
			free(http_connect_url);
			http_connect_url = NULL;
		}
		if(http_ap_url){
			free(http_ap_url);
			http_ap_url = NULL;
		}
		if(http_status_url){
			free(http_status_url);
			http_status_url = NULL;
		}

		vSemaphoreDelete(http_app_mutex);
		http_app_mutex = NULL;

		/* stop server */
		httpd_stop(httpd_handle);
		httpd_handle = NULL;
		httpd_config = NULL;
	}
}


/**
 * @brief helper to generate URLs of the wifi manager
 */
static char* http_app_generate_url(const char* page){

	char* ret;

	int root_len = strlen(WEBAPP_LOCATION);
	const size_t url_sz = sizeof(char) * ( (root_len+1) + ( strlen(page) + 1) );

	ret = malloc(url_sz);
	memset(ret, 0x00, url_sz);
	strcpy(ret, WEBAPP_LOCATION);
	ret = strcat(ret, page);

	return ret;
}

void http_app_start(bool lru_purge_enable){

	esp_err_t err;

	if(httpd_handle == NULL){

		httpd_config_t config = HTTPD_DEFAULT_CONFIG();

		/* this is an important option that isn't set up by default.
		 * We could register all URLs one by one, but this would not work while the fake DNS is active */
		config.uri_match_fn = httpd_uri_match_wildcard;
		config.lru_purge_enable = lru_purge_enable;

		/* generate the URLs */
		if(http_root_url == NULL){
			int root_len = strlen(WEBAPP_LOCATION);

			/* all the pages */
			const char page_js[] = "code.js";
			const char page_css[] = "style.css";
			const char page_connect[] = "connect.json";
			const char page_ap[] = "ap.json";
			const char page_status[] = "status.json";

			/* root url, eg "/"   */
			const size_t http_root_url_sz = sizeof(char) * (root_len+1);
			http_root_url = malloc(http_root_url_sz);
			memset(http_root_url, 0x00, http_root_url_sz);
			strcpy(http_root_url, WEBAPP_LOCATION);

			/* redirect url */
			size_t redirect_sz = 22 + root_len + 1; /* strlen(http://255.255.255.255) + strlen("/") + 1 for \0 */
			http_redirect_url = malloc(sizeof(char) * redirect_sz);
			*http_redirect_url = '\0';

			if(root_len == 1){
				snprintf(http_redirect_url, redirect_sz, "http://%s", DEFAULT_AP_IP);
			}
			else{
				snprintf(http_redirect_url, redirect_sz, "http://%s%s", DEFAULT_AP_IP, WEBAPP_LOCATION);
			}

			/* generate the other pages URLs*/
			http_js_url = http_app_generate_url(page_js);
			http_css_url = http_app_generate_url(page_css);
			http_connect_url = http_app_generate_url(page_connect);
			http_ap_url = http_app_generate_url(page_ap);
			http_status_url = http_app_generate_url(page_status);

		}
		http_app_mutex = xSemaphoreCreateMutex();
		httpd_config = &config;
		err = httpd_start(&httpd_handle, &config);

	    if (err == ESP_OK) {
	        ESP_LOGI(TAG, "Registering URI handlers");
	        httpd_register_uri_handler(httpd_handle, &http_server_get_request);
	        httpd_register_uri_handler(httpd_handle, &http_server_post_request);
	        httpd_register_uri_handler(httpd_handle, &http_server_delete_request);
	    }
	}

}
