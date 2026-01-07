#include "imu_driver.h"
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "IMU_DRIVER";

// --- PINS & REGISTERS ---
#define PIN_CS      10
#define PIN_CLK     6
#define PIN_MISO    2
#define PIN_MOSI    7
#define PIN_IMU_INT 5   // Interrupt Pin

// Common ST IMU Registers (LSM6DS series)
#define REG_INT1_CTRL   0x0D
#define REG_WHO_AM_I    0x0F
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_CTRL3_C     0x12
#define REG_OUTX_L_G    0x22

// --- SCALING FACTORS ---
const float ACC_SCALE  = 0.488f / 1000.0f * 9.80665f; 
const float GYRO_SCALE = 70.0f  / 1000.0f * 0.017453f; 

// --- GLOBALS ---
static spi_device_handle_t spi;
static SemaphoreHandle_t s_imu_data_ready_sem = NULL;
QueueHandle_t imu_data_queue = NULL; // The queue accessible by main.c

// --- SPI HELPER ---
static void write_register(uint8_t reg, uint8_t value) {
    uint8_t tx_data[2] = { reg & 0x7F, value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx_data };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

// --- INTERRUPT SERVICE ROUTINE (ISR) ---
static void IRAM_ATTR imu_gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_imu_data_ready_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// --- INTERNAL READ FUNCTION ---
// static void read_all_sensors(imu_data_packet_t *packet) {
//     uint8_t tx_buf[13] = {0};
//     tx_buf[0] = REG_OUTX_L_G | 0x80; // Read bit + address
//     uint8_t rx_buf[13] = {0};
    
//     spi_transaction_t t = { .length = 8 * 13, .tx_buffer = tx_buf, .rx_buffer = rx_buf };
//     ESP_ERROR_CHECK(spi_device_transmit(spi, &t));

//     // Parse bytes into the packet struct
//     packet->gyro[0] = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
//     packet->gyro[1] = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
//     packet->gyro[2] = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);
//     packet->acc[0]  = (int16_t)((rx_buf[8] << 8) | rx_buf[7]);
//     packet->acc[1]  = (int16_t)((rx_buf[10] << 8) | rx_buf[9]);
//     packet->acc[2]  = (int16_t)((rx_buf[12] << 8) | rx_buf[11]);
// }

static void read_all_sensors(imu_data_packet_t *packet) {
    // FIX: Define buffers as static and 4-byte aligned for DMA
    // DMA requires buffers to be in DRAM and Word-Aligned.
    static uint8_t tx_buf[16] __attribute__((aligned(4))) = {0};
    static uint8_t rx_buf[16] __attribute__((aligned(4))) = {0};

    // Clear buffers (optional but good safety)
    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));

    tx_buf[0] = REG_OUTX_L_G | 0x80; // Read bit + address

    // Transaction configuration
    spi_transaction_t t = {
        .length = 8 * 13,   // 13 bytes total transaction
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf
    };
    
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));

    // Parse bytes into the packet struct
    // The previous logic remains the same
    packet->gyro[0] = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
    packet->gyro[1] = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
    packet->gyro[2] = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);
    packet->acc[0]  = (int16_t)((rx_buf[8] << 8) | rx_buf[7]);
    packet->acc[1]  = (int16_t)((rx_buf[10] << 8) | rx_buf[9]);
    packet->acc[2]  = (int16_t)((rx_buf[12] << 8) | rx_buf[11]);
}

// --- DRIVER TASK ---
void imu_task(void *pvParameters) {
    imu_data_packet_t packet;

    while(1) {
        // Wait indefinitely for the Interrupt Semaphore
        if(xSemaphoreTake(s_imu_data_ready_sem, portMAX_DELAY) == pdTRUE) {
            
            // Read data from SPI
            read_all_sensors(&packet);

            // Send to Queue (Don't block if full, just overwrite/drop)
            xQueueSend(imu_data_queue, &packet, 0);
        }
    }
}

// --- INITIALIZATION ---
void init_imu() {
    // 1. Initialize Queue
    // imu_data_queue = xQueueCreate(10, sizeof(imu_data_packet_t));
    imu_data_queue = xQueueCreate(128, sizeof(imu_data_packet_t));

    // 2. SPI Bus Init
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

    // 3. Semaphore & GPIO Init
    s_imu_data_ready_sem = xSemaphoreCreateBinary();

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Rising edge
    io_conf.pin_bit_mask = (1ULL << PIN_IMU_INT);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0; 
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_IMU_INT, imu_gpio_isr_handler, NULL);

    // 4. Configure Sensor
    write_register(REG_CTRL3_C, 0x01); // Reset
    vTaskDelay(20 / portTICK_PERIOD_MS);
    
    write_register(REG_CTRL3_C, 0x44); // Block Data Update
    write_register(REG_CTRL1_XL, 0x64); // Accel 416Hz
    write_register(REG_CTRL2_G,  0x6C); // Gyro 416Hz

    // Enable Hardware Interrupt on Pin INT1 for Gyro Data Ready
    write_register(REG_INT1_CTRL, 0x02); 

    ESP_LOGI(TAG, "IMU Initialized with Interrupts");

    // 5. Start Driver Task
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);
}