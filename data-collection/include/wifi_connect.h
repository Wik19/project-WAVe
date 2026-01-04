#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

// Configuration
#define WIFI_SSID      "iPhone di Marco"
#define WIFI_PASS      "987654321"

/**
 * @brief Initialize Wi-Fi as Station and block until connected
 */
void wifi_init_sta(void);

#endif