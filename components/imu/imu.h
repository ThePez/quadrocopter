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
#define BNO085_STACK_SIZE (configMINIMAL_STACK_SIZE * 3)
#define BNO085_PRIORITY (tskIDLE_PRIORITY + 4)

typedef struct {
    double pitchAngle;
    double pitchRate;
    double rollAngle;
    double rollRate;
    double yawAngle;
    double yawRate;
} Telemitry_t;

void imu_init(void);
void imu_kill(void);

extern TaskHandle_t imuTaskHandle;
extern QueueHandle_t imuQueue;

#ifdef __cplusplus
}
#endif
#endif
