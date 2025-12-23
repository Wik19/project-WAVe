#include "imu_driver.h"
#include "settings.h"
#include "common.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "IMU";
static spi_device_handle_t spi; 

// --- INTERNAL SPI HELPERS ---
static void write_register(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg & 0x7F, value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

static uint8_t read_register(uint8_t reg) {
    uint8_t tx[2] = { reg | 0x80, 0x00 }; 
    uint8_t rx[2] = {0};
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    return rx[1];
}

static void read_raw_into_ptr(imu_packet_t *p) {
    uint8_t status;
    int retry_count = 0;

    do {
        status = read_register(REG_STATUS);
        if ((status & 0x03) != 0x03) {
            vTaskDelay(1); 
            retry_count++;
            if(retry_count > 500) { 
                 ESP_LOGE(TAG, "Sensor Timeout!");
                 vTaskDelay(1000 / portTICK_PERIOD_MS); 
                 retry_count = 0;
            }
        }
    } while ((status & 0x03) != 0x03); 

    uint8_t tx[13] = { REG_OUTX_L_G | 0x80 }; 
    uint8_t rx[13] = {0};
    spi_transaction_t t = { .length = 8 * 13, .tx_buffer = tx, .rx_buffer = rx };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));

    p->gyr_x = (int16_t)((rx[2] << 8) | rx[1]);
    p->gyr_y = (int16_t)((rx[4] << 8) | rx[3]);
    p->gyr_z = (int16_t)((rx[6] << 8) | rx[5]);
    p->acc_x = (int16_t)((rx[8] << 8) | rx[7]);
    p->acc_y = (int16_t)((rx[10] << 8) | rx[9]);
    p->acc_z = (int16_t)((rx[12] << 8) | rx[11]);
}

// --- PUBLIC FUNCTIONS ---
void init_imu() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO, .mosi_io_num = PIN_MOSI, .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, 
        .mode = 0, .spics_io_num = PIN_CS, .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    write_register(REG_CTRL3_C, 0x01); // Reset
    vTaskDelay(20 / portTICK_PERIOD_MS);
    write_register(REG_CTRL3_C, 0x44); // Block Data Update
    write_register(REG_CTRL1_XL, 0x60); // 416Hz, 2g
    write_register(REG_CTRL2_G,  0x60); // 416Hz, 250dps
    ESP_LOGI(TAG, "IMU Initialized");
}

void imu_task(void *pvParameters) {
    int sample_idx = 0;
    while (1) {
        read_raw_into_ptr(&buffers[write_buf_idx][sample_idx]);
        sample_idx++;

        if (sample_idx >= BATCH_SIZE) {
            if (s_server_task_handle != NULL) {
                xTaskNotify(s_server_task_handle, write_buf_idx, eSetValueWithOverwrite);
            }
            write_buf_idx = !write_buf_idx; 
            sample_idx = 0;
            vTaskDelay(1); 
        }
    }
}