#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"

// --- LwIP Includes (For WiFi/TCP) ---
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

// --- Custom Modules ---
#include "mic_driver.h"  // <--- Includes I2S_READ_LEN and Queue Handle
#include "imu_driver.h"  // <--- Includes IMU Queue Handle

// External declaration for your WiFi connect function
extern void wifi_init_sta(void);

#define PORT 8080
static const char *TAG = "MAIN_APP";

// =============================================================
//                   TCP SERVER TASK
// =============================================================
static void tcp_server_task(void *pvParameters) {
    char addr_str[128];
    struct sockaddr_in dest_addr;

    // =========================================================================
    // 1. ALLOCATE BUFFERS (Header + Sequence Num + Data)
    // =========================================================================
    
    // MICROPHONE: 1 Byte Header + 4 Bytes Seq + 1024 Bytes Audio
    // Total: 1029 Bytes
    uint8_t *audio_packet = (uint8_t *)malloc(1 + 4 + I2S_READ_LEN);
    
    // IMU: 1 Byte Header + 4 Bytes Seq + 240 Bytes IMU Batch
    // Total: 245 Bytes
    uint8_t *imu_packet   = (uint8_t *)malloc(1 + 4 + 240);
    
    // TEMP BUFFER: To read raw data from queue before packing
    uint8_t *raw_mic_read = (uint8_t *)malloc(I2S_READ_LEN);
    imu_data_packet_t imu_single_sample; 

    // Pre-fill Protocol Headers
    audio_packet[0] = 0xAA;
    imu_packet[0]   = 0xBB;

    if (!audio_packet || !imu_packet || !raw_mic_read) {
        ESP_LOGE(TAG, "Critical: Failed to allocate TCP buffers");
        vTaskDelete(NULL);
    }

    while (1) {
        // =====================================================================
        // 2. SETUP SOCKET
        // =====================================================================
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);

        int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        // OPTIMIZATION 1: TCP_NODELAY (Disable Nagle's Algorithm)
        int flag = 1;
        setsockopt(listen_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        // OPTIMIZATION 2: Increase Send Buffer to 32KB to handle WiFi Jitter
        int sndbuf_len = 32768; 
        setsockopt(listen_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf_len, sizeof(sndbuf_len));

        bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        listen(listen_sock, 1);

        ESP_LOGI(TAG, "Socket listening on port %d... Waiting for Python.", PORT);

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        
        // Block here until client connects
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        
        if (sock >= 0) {
            ESP_LOGI(TAG, "Client connected! Starting Stream with Counters...");
            
            // Re-apply optimizations to the active socket
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

            // Clear Queues so we start fresh
            xQueueReset(mic_data_queue);
            xQueueReset(imu_data_queue);
            
            // Initialize Sequence Counters
            uint32_t mic_seq_num = 0;
            uint32_t imu_seq_num = 0;

            int imu_sample_count = 0;
            // IMU Offset starts at 5 (1 byte Header + 4 bytes Seq)
            int imu_offset = 5; 

            // =================================================================
            // 3. MAIN POLLING LOOP
            // =================================================================
            while (1) {
                bool sent_something = false;

                // --- A. MICROPHONE HANDLING ---
                if (xQueueReceive(mic_data_queue, raw_mic_read, 0) == pdTRUE) {
                    
                    // 1. Pack Sequence Number (Bytes 1-4)
                    memcpy(&audio_packet[1], &mic_seq_num, 4);
                    mic_seq_num++; 

                    // 2. Pack Audio Data (Bytes 5-1028)
                    memcpy(&audio_packet[5], raw_mic_read, I2S_READ_LEN);

                    // 3. Send Total Packet (1029 Bytes)
                    if (send(sock, audio_packet, 1 + 4 + I2S_READ_LEN, 0) < 0) {
                        ESP_LOGE(TAG, "Send Failed (Mic)");
                        break;
                    }
                    sent_something = true;
                }

                // --- B. IMU HANDLING ---
                if (xQueueReceive(imu_data_queue, &imu_single_sample, 0) == pdTRUE) {
                    
                    // Add sample to batch buffer
                    memcpy(imu_packet + imu_offset, &imu_single_sample, 12);
                    imu_offset += 12;
                    imu_sample_count++;

                    // If batch is full (20 samples)
                    if (imu_sample_count >= 20) {
                        
                        // 1. Pack Sequence Number (Bytes 1-4)
                        memcpy(&imu_packet[1], &imu_seq_num, 4);
                        imu_seq_num++;

                        // 2. Send Total Packet (245 Bytes)
                        if (send(sock, imu_packet, 1 + 4 + 240, 0) < 0) {
                            ESP_LOGE(TAG, "Send Failed (IMU)");
                            break;
                        }
                        
                        // Reset Batch Logic
                        imu_sample_count = 0;
                        imu_offset = 5; 
                    }
                    sent_something = true;
                }
                
                // --- C. SMART YIELD / PANIC MODE ---
                if (!sent_something) {
                    // Check how full the Mic queue is
                    UBaseType_t mic_items = uxQueueMessagesWaiting(mic_data_queue);
                    
                    if (mic_items < 60) {
                        // Queue is safe -> Sleep to save power
                        vTaskDelay(1); 
                    } else {
                        // Queue is filling up -> Panic! Run immediately!
                        taskYIELD(); 
                    }
                }
                
            }
            
            ESP_LOGE(TAG, "Client disconnected");
            shutdown(sock, 0);
            close(sock);
        }
        close(listen_sock);
    }
    
    free(audio_packet);
    free(imu_packet);
    free(raw_mic_read);
    vTaskDelete(NULL);
}

// =============================================================
//                       APP MAIN
// =============================================================
void app_main(void) {
    // 1. Initialize NVS (Needed for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      nvs_flash_init();
    }

    wifi_init_sta();
    esp_wifi_set_ps(WIFI_PS_NONE); 

    init_imu();
    init_mic();

    // PRODUCERS: Priority 4
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);    
    xTaskCreate(mic_task, "mic_task", 4096, NULL, 10, NULL);

    // CONSUMER: Priority 5 (Must be higher than producers)
    // STACK: 8192 (Critical for TCP)
    xTaskCreate(tcp_server_task, "tcp_server", 8192, NULL, 5, NULL);
}