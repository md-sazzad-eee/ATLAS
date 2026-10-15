idf_component_register(SRCS "main.c" "hal_board_a.c"
                       INCLUDE_DIRS "."
                       REQUIRES carrier_core common driver esp_adc esp_timer)
