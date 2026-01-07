#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

// --- LwIP Includes ---
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

// --- Custom Modules ---
#include "imu_driver.h"
// Assuming you have this file or the function definition available
// #include "wifi_connect.h" 
// If you don't have a header, declare the function extern:
extern void wifi_init_sta(void);

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

    // Buffer for batching: 50 bytes per line * 30 samples approx = 1500 bytes
    char payload[1500]; 
    int payload_offset = 0;
    
    // Batch size: How many samples to group before sending?
    // 416Hz / 20 = ~20 packets per second (much healthier for WiFi)
    const int SAMPLES_PER_BATCH = 20;
    int sample_count = 0;

    imu_data_packet_t data;

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

        // --- OPTIONAL: Enable TCP_NODELAY to disable Nagle's algorithm ---
        int flag = 1;
        setsockopt(listen_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        // ----------------------------------------------------------------

        bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        listen(listen_sock, 1);

        ESP_LOGI(TAG, "Socket listening on port %d...", PORT);

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        
        if (sock >= 0) {
            ESP_LOGI(TAG, "Client connected!");
            
            // Re-apply NODELAY to the accepted socket just in case
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

            xQueueReset(imu_data_queue);
            payload_offset = 0;
            sample_count = 0;

            while (1) {
                // Wait for data
                if (xQueueReceive(imu_data_queue, &data, portMAX_DELAY)) {
                    
                    // Append to buffer instead of sending immediately
                    int written = snprintf(payload + payload_offset, sizeof(payload) - payload_offset, 
                            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                            data.acc[0] * ACC_SCALE, 
                            data.acc[1] * ACC_SCALE, 
                            data.acc[2] * ACC_SCALE,
                            data.gyro[0] * GYRO_SCALE, 
                            data.gyro[1] * GYRO_SCALE, 
                            data.gyro[2] * GYRO_SCALE
                    );

                    if (written > 0) {
                        payload_offset += written;
                        sample_count++;
                    }

                    // Send only when we have enough samples OR the buffer is getting full
                    if (sample_count >= SAMPLES_PER_BATCH || payload_offset > (sizeof(payload) - 100)) {
                        
                        int err = send(sock, payload, payload_offset, 0);
                        if (err < 0) {
                            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                            break; 
                        }
                        
                        // Reset buffer
                        payload_offset = 0;
                        sample_count = 0;
                    }
                }
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
    // Order matters: Init IMU first to create the Queue.
    init_imu();
    wifi_init_sta();

    // 3. Start Server
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}