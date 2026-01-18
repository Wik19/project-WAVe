#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Define a struct to hold one synchronized sample of data
typedef struct {
    int16_t gyro[3];
    int16_t acc[3];
} imu_data_packet_t;

// Expose the Queue handle so main.c can access it
extern QueueHandle_t imu_data_queue;

// Expose scaling factors (defined in .c file)
extern const float ACC_SCALE;
extern const float GYRO_SCALE;

// Initialization function
void init_imu(void);
void imu_task(void *pvParameters);

#endif // IMU_DRIVER_H