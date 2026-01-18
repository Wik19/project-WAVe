#include "mic_driver.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include <string.h>
#include <freertos/queue.h>

static const char *TAG = "MIC_DRIVER";

// Queue to send audio buffers to other tasks (like your TCP server)
QueueHandle_t mic_data_queue;

void init_mic(void) {
    // 1. Create a queue to hold audio chunks
    // We store the actual data buffer in the queue item for simplicity.
    // Each item is one chunk of audio (e.g., 1024 bytes).
    mic_data_queue = xQueueCreate(64, I2S_READ_LEN); 
    if (mic_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create mic queue");
        return;
    }

    // 2. Configure I2S Driver
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX, // Master Transmit (ESP32 generates clock), RX (Receive data)
        .sample_rate = MIC_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // ICS-43434 is 24-bit, but fits in 32-bit slot
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Set mic L/R pin to GND for Left channel
        .communication_format = I2S_COMM_FORMAT_STAND_I2S, // Standard Philips I2S
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = MIC_DMA_BUF_COUNT,
        .dma_buf_len = MIC_DMA_BUF_LEN,
        .use_apll = false, // ESP32-C3 usually doesn't need APLL for standard rates
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    // 3. Configure Pins
    i2s_pin_config_t pin_config = {
        .bck_io_num = MIC_PIN_BCK,
        .ws_io_num = MIC_PIN_WS,
        .data_out_num = I2S_PIN_NO_CHANGE, // Not used for mic
        .data_in_num = MIC_PIN_DATA_IN
    };

    // 4. Install and Start Driver
    // Use I2S_NUM_0 for ESP32-C3
    ESP_ERROR_CHECK(i2s_driver_install(MIC_I2S_PORT, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(MIC_I2S_PORT, &pin_config));
    
    ESP_LOGI(TAG, "I2S Microphone Initialized on Pins: BCK=%d WS=%d IN=%d", 
             MIC_PIN_BCK, MIC_PIN_WS, MIC_PIN_DATA_IN);
}

void mic_task(void *pvParameters) {
    // Temporary buffer to read data from DMA
    // We use int32_t because the mic sends 24 bits inside a 32-bit frame
    int32_t *read_buf = (int32_t *)malloc(I2S_READ_LEN);
    size_t bytes_read = 0;

    if (read_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for I2S read");
        vTaskDelete(NULL);
    }
    while (1) {
        // Read from I2S peripheral
        // This blocks until data is available (driven by sampling rate)
        esp_err_t err = i2s_read(MIC_I2S_PORT, 
                                 (void *)read_buf, 
                                 I2S_READ_LEN, 
                                 &bytes_read, 
                                 portMAX_DELAY);

        if (err == ESP_OK && bytes_read > 0) {
            
            // OPTIONAL: Data Processing
            // The ICS-43434 data is 24-bit. When read into 32-bit, the data is usually
            // shifted left. You might want to scale it or shift it here.
            // For now, we send raw 32-bit integers.

            // Send to Queue
            // xQueueSend copies the data into the queue storage.
            // If the queue is full, we drop the packet (timeout 0) to avoid lagging live audio.
            if (xQueueSend(mic_data_queue, read_buf, 0) != pdTRUE) {
                // Queue full - audio packet dropped
                // ESP_LOGW(TAG, "Mic Queue Full - Dropped Audio Packet");
            }
        }
    }

    free(read_buf);
    vTaskDelete(NULL);
}