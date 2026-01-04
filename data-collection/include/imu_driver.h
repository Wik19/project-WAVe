#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

// Expose these constants so main.c can use them for formatting
extern const float ACC_SCALE;
extern const float GYRO_SCALE;

/**
 * @brief Initialize the SPI bus and the LSM6DSOX sensor
 */
void init_imu(void);

/**
 * @brief Reads raw data from the sensor
 * @param gyroData Array[3] to store raw X,Y,Z gyro data
 * @param accelData Array[3] to store raw X,Y,Z accel data
 */
void read_all_sensors(int16_t *gyroData, int16_t *accelData);

#endif