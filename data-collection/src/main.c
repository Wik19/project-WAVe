#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

// --- LwIP Includes for Sockets ---
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

// --- Our New Custom Modules ---
#include "imu_driver.h"
#include "wifi_connect.h"

#define PORT 8080
static const char *TAG = "MAIN_APP";

// =============================================================
//                   MAIN SERVER TASK
// =============================================================

static void tcp_server_task(void *pvParameters) {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    char payload[256]; 
    int16_t raw_acc[3];
    int16_t raw_gyro[3];

    while (1) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        listen(listen_sock, 1);

        ESP_LOGI(TAG, "Socket listening on port %d...", PORT);

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        
        if (sock >= 0) {
            ESP_LOGI(TAG, "Client connected!");
            
            while (1) {
                // 1. Get Data from IMU Module
                read_all_sensors(raw_gyro, raw_acc);

                // 2. Format Data using constants from IMU Module
                int len = snprintf(payload, sizeof(payload), 
                        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                        raw_acc[0] * ACC_SCALE, 
                        raw_acc[1] * ACC_SCALE, 
                        raw_acc[2] * ACC_SCALE,
                        raw_gyro[0] * GYRO_SCALE, 
                        raw_gyro[1] * GYRO_SCALE, 
                        raw_gyro[2] * GYRO_SCALE
                );

                // 3. Send over Wi-Fi
                int err = send(sock, payload, len, 0);
                
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break; 
                }

                // 4. Rate Limit (20Hz)
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }

            shutdown(sock, 0);
            close(sock);
            ESP_LOGI(TAG, "Client disconnected");
        }
        close(listen_sock);
    }
}

// =============================================================
//                       APP MAIN
// =============================================================
void app_main(void) {
    // 1. NVS Init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      nvs_flash_init();
    }

    // 2. Init Modules
    init_imu();
    wifi_init_sta();

    // 3. Start Server
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}