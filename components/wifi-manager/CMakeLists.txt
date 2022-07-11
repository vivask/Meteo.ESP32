idf_component_register(
    SRC_DIRS src
    REQUIRES log mdns wpa_supplicant lwip esp_http_server vfs fatfs nvs_flash esp_https_ota app_update radsens json
    INCLUDE_DIRS src
    EMBED_FILES src/code.js src/style.css src/index.html
    )
