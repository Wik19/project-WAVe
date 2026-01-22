#include <WiFi.h>

const char* ssid = "OlafHotspot";
const char* password = "Karak123";

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nConnected to Hotspot!");
}

void loop() {
  // Get the Received Signal Strength Indication (RSSI)
  long rssi = WiFi.RSSI();
  
  Serial.print("Signal Strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
  
  delay(2000); // Update every 2 seconds
}