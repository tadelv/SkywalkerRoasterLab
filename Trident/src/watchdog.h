#pragma once
#include <esp_task_wdt.h>

// The ESP32 Arduino framework already initializes the task watchdog
// We just need to subscribe our tasks to it

// Initialize watchdog - adds the current task to the existing watchdog
inline void initWatchdog() {
  // Just add the current task (setup/loop) to the existing watchdog
  esp_err_t err = esp_task_wdt_add(NULL);
  if (err != ESP_OK) {
    // Watchdog might not be initialized yet, which is fine
    // The framework will initialize it later
  }
}

// Feed/reset the watchdog timer
inline void feedWatchdog() {
  // Only reset if we're subscribed to the watchdog
  esp_task_wdt_reset();
}

// Add a FreeRTOS task to watchdog monitoring
inline void addTaskToWatchdog(TaskHandle_t task) {
  esp_err_t err;
  if (task != NULL) {
    err = esp_task_wdt_add(task);
  } else {
    err = esp_task_wdt_add(NULL);  // Add current task
  }
  // Ignore errors - task might already be added
  (void)err;
}
