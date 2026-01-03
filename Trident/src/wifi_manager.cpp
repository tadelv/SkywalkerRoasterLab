#include "wifi_manager.h"
#include "dlog.h"
#include "wifi_setup.h"
#include <WiFi.h>

static unsigned long lastWiFiCheck = 0;
static const unsigned long WIFI_CHECK_INTERVAL = 30000;  // 30 seconds
static int reconnectAttempts = 0;
static const int MAX_RECONNECT_ATTEMPTS = 5;

extern void setupAP();  // Defined in wifi_setup.cpp

void wifiMonitorTask(void *params) {
  while (1) {
    unsigned long now = millis();

    if (now - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
      lastWiFiCheck = now;

      // Check WiFi status only in STA mode
      if (WiFi.status() != WL_CONNECTED && WiFi.getMode() == WIFI_STA) {
        reconnectAttempts++;
        D_printf("WiFi disconnected. Reconnect attempt %d/%d\n",
                 reconnectAttempts, MAX_RECONNECT_ATTEMPTS);

        if (reconnectAttempts <= MAX_RECONNECT_ATTEMPTS) {
          WiFi.disconnect();
          delay(100);
          WiFi.reconnect();
        } else {
          D_println("Max reconnect attempts reached, switching to AP mode");
          setupAP();  // Fall back to AP mode
          reconnectAttempts = 0;
        }
      } else if (WiFi.status() == WL_CONNECTED) {
        // Reset counter on successful connection
        if (reconnectAttempts > 0) {
          D_println("WiFi reconnected successfully");
          reconnectAttempts = 0;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10000));  // Check every 10 seconds
  }
}

void startWiFiMonitor() {
  xTaskCreate(wifiMonitorTask, "WiFiMonitor", 3072, NULL, 1, NULL);
  D_println("WiFi monitor started");
}
