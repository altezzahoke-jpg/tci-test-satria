idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc nvs_flash esp_wifi esp_http_server esp_timer esp_event)
