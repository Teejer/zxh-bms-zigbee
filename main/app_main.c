#include "nvs_flash.h"

#include "esp_log.h"

#include "zxh_ble.h"
#include "zxh_zigbee.h"

static const char *TAG = "zxh_main";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGI(TAG, "ZXH-BMS Zigbee gateway: BLE battery reader -> zigbee2mqtt");

    zxh_zigbee_start();
#ifndef ZXH_DISABLE_BLE
    zxh_ble_start();
#endif
}
