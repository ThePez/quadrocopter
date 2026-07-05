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
    if (init_drone() != ESP_OK) {
        ESP_LOGE(TAG, "Drone task creation failed");
        esp_restart(); // This function doesn't return
    }

    // Done... Drone task will handle the rest.
}
