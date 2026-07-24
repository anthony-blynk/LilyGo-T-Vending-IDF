idf_component_register(SRCS "main.c" "net_manager.c" "blynk_mqtt.c"
                        INCLUDE_DIRS "."
                        REQUIRES mqtt esp_wifi esp_event esp_netif nvs_flash esp_driver_gpio esp_driver_uart esp_timer
                                  esp_https_ota esp_http_client app_update mbedtls)
