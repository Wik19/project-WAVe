#include "nvs_flash.h"
#include "settings.h"
#include "common.h"
#include "wifi_conn.h"
#include "imu_driver.h"
#include "tcp_server.h"

// --- GLOBAL VARIABLES DEFINITION ---
// These are the actual memory allocations. 
// The other files see these via the 'extern' keywords in common.h
imu_packet_t buffers[2][BATCH_SIZE]; 
volatile int write_buf_idx = 0;      
TaskHandle_t s_server_task_handle = NULL; 

void app_main(void) {
    // 1. Initialize NVS (Required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Setup Hardware
    init_imu();

    // 3. Connect to WiFi
    wifi_init_sta();

    // 4. Start Tasks
    xTaskCreate(imu_task, "IMU", 4096, NULL, 5, NULL);
    xTaskCreate(tcp_server_task, "Server", 4096, NULL, 5, NULL);
}