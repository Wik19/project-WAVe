#ifndef SETTINGS_H
#define SETTINGS_H

// --- WIFI ---
#define WIFI_SSID      "iPhone di Marco" 
#define WIFI_PASS      "987654321"
#define PORT           8080

// --- BUFFERING ---
#define BATCH_SIZE     25   

// --- PINS ---
#define PIN_CS         10
#define PIN_CLK        6
#define PIN_MISO       2
#define PIN_MOSI       7

// --- IMU REGISTERS ---
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_STATUS     0x1E
#define REG_OUTX_L_G   0x22

#endif