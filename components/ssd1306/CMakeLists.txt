set(component_srcs "ssd1306.c" "fonts.c")

idf_component_register(
    SRCS "${component_srcs}"
    INCLUDE_DIRS "."
    PRIV_REQUIRES i2cdev log esp_idf_lib_helpers
)
