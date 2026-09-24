idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES app_update nvs_flash esp_wifi esp_http_server driver esp_adc)
