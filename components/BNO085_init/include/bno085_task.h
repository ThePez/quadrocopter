/*
 *****************************************************************************
 * File: bno085_task.h
 * Author: Jack Cairns
 * Date: 20-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef BNO085_TASK_H
#define BNO085_TASK_H
#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define BNO085_QUEUE_LENGTH 5
#define BNO085_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define BNO085_PRIORITY (tskIDLE_PRIORITY + 4)

typedef struct {
    float pitchAngle;
    float pitchRate;
    float rollAngle;
    float rollRate;
    float yawAngle;
    float yawRate;
    uint64_t prevTime;
} Telemitry_t;

void bno08x_start_task(void);
extern TaskHandle_t bno085Task;
extern QueueHandle_t bno085Queue;

#ifdef __cplusplus
}
#endif
#endif
