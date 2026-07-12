/*
 *****************************************************************************
 * File: main.c
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#include "drone.h"

#include <esp_log.h>
#include <esp_system.h>

#define TAG "MAIN"

void app_main(void) {
#ifdef ESC_PROGRAMMING_MODE
    if (init_esc_programming() != ESP_OK) {
        ESP_LOGE(TAG, "ESC programming init failed");
        esp_restart(); // This function doesn't return
    }
#else
    if (init_drone() != ESP_OK) {
        ESP_LOGE(TAG, "Drone task creation failed");
        esp_restart(); // This function doesn't return
    }
#endif

    // Done... Drone task will handle the rest.
}
