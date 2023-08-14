if(IDF_VERSION_MAJOR GREATER_EQUAL 4)
    idf_component_register(
        SRC_DIRS src
        REQUIRES log wpa_supplicant lwip esp_http_server vfs nvs_flash spiffs json esp_http_client esp-tls list
        INCLUDE_DIRS include
        )
else()
    set(COMPONENT_SRCDIRS src) 
    set(COMPONENT_ADD_INCLUDEDIRS include)
    set(COMPONENT_REQUIRES log wpa_supplicant lwip esp_http_server vfs nvs_flash spiffs json esp_http_client esp-tls list)
    register_component()
endif()
