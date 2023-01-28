if(IDF_VERSION_MAJOR GREATER_EQUAL 4)
    idf_component_register(
        SRC_DIRS src
        REQUIRES log mdns wpa_supplicant lwip esp_http_server vfs fatfs nvs_flash esp_https_ota app_update radsens json
        INCLUDE_DIRS include
        EMBED_FILES ui/code.js ui/style.css ui/index.html
        )
else()
    set(COMPONENT_SRCDIRS src) 
    set(COMPONENT_ADD_INCLUDEDIRS include)
    set(COMPONENT_REQUIRES log mdns wpa_supplicant lwip esp_http_server vfs fatfs nvs_flash esp_https_ota app_update radsens json)
    register_component()
endif()
