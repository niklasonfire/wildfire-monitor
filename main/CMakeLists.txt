idf_component_register(
    SRCS "main.c" "board.c" "ble_explorer.c" "cmd_ble.c" "cmd_cap.c"
         "capture.c" "capture_store.c" "display.c" "ui.c" "rtc_bm8563.c"
         "webdump.c" "imu.c" "i2c_bus.c"
    INCLUDE_DIRS "."
    REQUIRES bt console driver esp_adc esp_driver_gpio esp_driver_i2c
             esp_driver_spi esp_event esp_http_server esp_lcd esp_netif
             esp_timer esp_wifi fatfs nvs_flash spi_flash vfs wear_levelling)
