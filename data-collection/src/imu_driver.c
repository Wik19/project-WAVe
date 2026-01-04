#include "imu_driver.h"
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "IMU_DRIVER";

// --- PINS & REGISTERS ---
#define PIN_CS      10
#define PIN_CLK     6
#define PIN_MISO    2
#define PIN_MOSI    7

#define REG_WHO_AM_I    0x0F
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_CTRL3_C     0x12
#define REG_OUTX_L_G    0x22

// --- SCALING FACTORS (Defined here, declared extern in header) ---
const float ACC_SCALE  = 0.488f / 1000.0f * 9.80665f; 
const float GYRO_SCALE = 70.0f  / 1000.0f * 0.017453f; 

static spi_device_handle_t spi; // Private handle

// Private helper
static void write_register(uint8_t reg, uint8_t value) {
    uint8_t tx_data[2] = { reg & 0x7F, value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx_data };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

void read_all_sensors(int16_t *gyroData, int16_t *accelData) {
    uint8_t tx_buf[13] = {0};
    tx_buf[0] = REG_OUTX_L_G | 0x80;
    uint8_t rx_buf[13] = {0};
    
    spi_transaction_t t = { .length = 8 * 13, .tx_buffer = tx_buf, .rx_buffer = rx_buf };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));

    gyroData[0] = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
    gyroData[1] = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
    gyroData[2] = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);
    accelData[0] = (int16_t)((rx_buf[8] << 8) | rx_buf[7]);
    accelData[1] = (int16_t)((rx_buf[10] << 8) | rx_buf[9]);
    accelData[2] = (int16_t)((rx_buf[12] << 8) | rx_buf[11]);
}

void init_imu() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO, .mosi_io_num = PIN_MOSI, .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, .mode = 0,
        .spics_io_num = PIN_CS, .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // Wake up and configure
    write_register(REG_CTRL3_C, 0x01); // Reset
    vTaskDelay(20 / portTICK_PERIOD_MS);
    write_register(REG_CTRL3_C, 0x44); // Block Data Update
    write_register(REG_CTRL1_XL, 0x64); // Accel 416Hz
    write_register(REG_CTRL2_G,  0x6C); // Gyro 416Hz
    
    ESP_LOGI(TAG, "IMU Initialized successfully");
}