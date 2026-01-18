#ifndef MIC_DRIVER_H
#define MIC_DRIVER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

// --- Configuration ---
// Adjust these pins based on your wiring!
#define MIC_I2S_PORT        I2S_NUM_0
#define MIC_PIN_BCK         0   // Bit Clock (SCK)
#define MIC_PIN_WS          1   // Word Select (LRCK / WS)
#define MIC_PIN_DATA_IN     3   // Serial Data (SD / DOUT)

// Audio Settings
#define MIC_SAMPLE_RATE     16000   // 16kHz is standard for voice
#define MIC_DMA_BUF_LEN     256     // Samples per interrupt
#define MIC_DMA_BUF_COUNT   16       // Number of buffers

// The ICS-43434 sends 24-bit data packed in 32-bit frames.
// We read 32-bit integers to capture the full dynamic range.
#define I2S_READ_LEN        (MIC_DMA_BUF_LEN * sizeof(int32_t)) 

// --- Global Queue Handle ---
// This queue will hold "chunks" of audio data to be processed/sent
extern QueueHandle_t mic_data_queue;

// --- Function Prototypes ---
void init_mic(void);
void mic_task(void *pvParameters);

#endif