/*
 *****************************************************************************
 * File: remote.h
 * Author: Jack Cairns
 * Date: 23-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef REMOTE_H
#define REMOTE_H

// STD C lib headers
#include <stdint.h>

// KConfig header
#include "sdkconfig.h"

// ESP-IDF Prebuilts
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define REMOTE_STACK (configMINIMAL_STACK_SIZE * 2)
#define REMOTE_PRIO (tskIDLE_PRIORITY + 2)
#define REMOTE_DELAY (pdMS_TO_TICKS(50))

#define ADC_MIN 0
#define ADC_MAX 4095

#define MIN_RATE -200.0
#define MAX_RATE 200.0

#define MAX_ANGLE 30.0
#define MIN_ANGLE -30.0

#define MIN_THROTTLE 1000.0
#define MAX_THROTTLE 2000.0

#define MODE_BUTTON_PIN 33
#define SHUTOFF_BUTTON_PIN 25

#define CHECK_ERR(code, msg)                                                                                           \
    do {                                                                                                               \
        esp_err_t err = (code);                                                                                        \
        if (err != ESP_OK) {                                                                                           \
            ESP_LOGE(TAG, msg);                                                                                        \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

typedef struct {
    gpio_num_t pin;
    uint8_t pushAllowed;
    uint64_t prevFalling;
    uint64_t prevRising;
} __packed button_state_t;

#endif
