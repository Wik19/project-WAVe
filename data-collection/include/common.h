#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "settings.h"

// --- DATA STRUCTURES ---
typedef struct __attribute__((packed)) {
    int16_t acc_x; int16_t acc_y; int16_t acc_z;
    int16_t gyr_x; int16_t gyr_y; int16_t gyr_z;
} imu_packet_t;

// --- SHARED GLOBALS (Declarations Only) ---
// The 'extern' keyword tells the compiler: "These exist, but are defined in main.c"
extern imu_packet_t buffers[2][BATCH_SIZE]; 
extern volatile int write_buf_idx;      
extern TaskHandle_t s_server_task_handle; 

#endif