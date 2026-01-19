/*
 ******************************************************************************
 * File: terminal_monitor.h
 * Author: Jack Cairns
 * Date: 19062025
 * Brief:
 * REFERENCE:
 ******************************************************************************
 */

#ifndef SERIAL_MONITOR_H
#define SERIAL_MONITOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TERMINAL_STACK (2 * configMINIMAL_STACK_SIZE)
#define TERMINAL_PRIORITY (2 + tskIDLE_PRIORITY)

#define UART_NUM UART_NUM_0
#define TERMINAL_BUF_SIZE 1024
#define INPUT_BUFFER_SIZE 128
#define TERMINAL_QUEUE_LENGTH 5

// Need at least 20ms for the ESP32 watchdog timer on idle tasks
#define TERMINAL_DELAY pdMS_TO_TICKS(20) // 20ms
#define UART_DELAY pdMS_TO_TICKS(5)      // 5ms

#define UART_DEL_SEQUENCE "\b \b"
#define UART_DEL_LEN 3
#define UART_NEWLINE "\r\n"
#define UART_NEWLINE_LEN 2
#define ASCII_DEL 127

#define TERMINAL_FLOAT_COUNT 15

void terminal_monitor_init(void);
void uart_console_init(void);
void terminal_monitor_task(void* pvParams);
void terminal_delete(void);
uint8_t parse_doubles_from_terminal(const char* input, double out[TERMINAL_FLOAT_COUNT]);

extern TaskHandle_t terminalHandle;
extern QueueHandle_t terminalQueue;

#endif