idf_component_register(SRCS "main.c" "hal_board_b.c" "links.c"
                       INCLUDE_DIRS "."
                       REQUIRES agent_core common driver esp_adc esp_wifi esp_netif esp_event nvs_flash esp_timer lwip)
