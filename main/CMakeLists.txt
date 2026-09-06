idf_component_register(SRCS "app_main.c" "zxh_ble.c" "zxh_proto.c" "zxh_zigbee.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES bt nvs_flash esp_timer driver)
