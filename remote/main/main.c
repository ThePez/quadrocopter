/*
 ******************************************************************************
 * File: main.c
 * Author: Jack Cairns
 * Date: 03-07-2026
 ******************************************************************************
 */

#include "remote.h"

#include <esp_log.h>
#include <esp_system.h>

#define TAG "MAIN"

void app_main(void) {
    if (init_remote() != ESP_OK) {
        ESP_LOGE(TAG, "Remote task creation failed");
        esp_restart(); // This function doesn't return
    }
    
    // Done... Remote task will handle the rest.
}