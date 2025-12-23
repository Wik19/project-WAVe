#include "tcp_server.h"
#include "settings.h"
#include "common.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include <netinet/tcp.h>

static const char *TAG = "TCP";

void tcp_server_task(void *pvParameters) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    while (1) {
        int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        listen(listen_sock, 1);
        
        ESP_LOGI(TAG, "Waiting for client on port %d...", PORT);
        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);

        if (sock >= 0) {
            ESP_LOGI(TAG, "Client Connected. Streaming...");
            
            int nodelay = 1;
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

            struct timeval tv;
            tv.tv_sec = 2; 
            tv.tv_usec = 0;
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

            xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
            s_server_task_handle = xTaskGetCurrentTaskHandle();
            uint32_t buf_to_send_idx;
            
            while (1) {
                if (xTaskNotifyWait(0, 0, &buf_to_send_idx, portMAX_DELAY) == pdTRUE) {
                    int err = send(sock, &buffers[buf_to_send_idx], sizeof(buffers[0]), 0);
                    if (err < 0) {
                        ESP_LOGE(TAG, "Send Error/Timeout. Closing.");
                        break; 
                    }
                }
            }
            s_server_task_handle = NULL;
            shutdown(sock, 0);
            close(sock);
        }
        close(listen_sock);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}