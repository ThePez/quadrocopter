/*
 ******************************************************************************
 * File: terminal_monitor.h
 * Author: Jack Cairns
 * Date: 23-07-2025
 * Brief:
 * REFERENCE:
 ******************************************************************************
 * EXTERNAL FUNCTIONS
 ******************************************************************************
 *
 ******************************************************************************
 */

#include "terminal_monitor.h"

TaskHandle_t terminalHandle = NULL;
QueueHandle_t terminalQueue = NULL;

/* terminal_monitor_init()
 * ----------------------
 *
 */
void terminal_monitor_init(void) {

    xTaskCreate((void*) &terminal_monitor_task, "monitor", TERMINAL_STACK, NULL, TERMINAL_PRIORITY, &terminalHandle);
}

/* uart_console_init()
 * -------------------
 * Function to setup UART num 0. Installs the driver and sets up
 * the default pins used by the ESP32, using the provided ESP-idf
 * HAL functions.
 *
 * Configured for:
 *  - 115200 BAUD rate
 *  - 8-bit data
 *  - Even Parity
 *  - 1 stop bit
 *  - flow control disabled
 */
void uart_console_init(void) {

    // Step 1, Set config parameters
    const uart_config_t uart_config = {.baud_rate = 115200,
                                       .data_bits = UART_DATA_8_BITS,
                                       .parity = UART_PARITY_EVEN,
                                       .stop_bits = UART_STOP_BITS_1,
                                       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    // Step 2, set the GPIO pins (here using default pins)
    ESP_ERROR_CHECK(
        uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Step 3, install driver
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, TERMINAL_BUF_SIZE, 0, 0, NULL, 0));
}

/* terminal_monitor_task()
 * ------------------------
 * Task that monitors UART input and echos the character entered back to the
 * sender. Will cap at INPUT_BUFFER_SIZE (128 chars). Once a newline is sent,
 * the string will be sent to the task's Queue for processing by other tasks.
 */
void terminal_monitor_task(void* pvParams) {

    uint8_t recieved;
    char inputBuffer[INPUT_BUFFER_SIZE] = {0};
    int inputBufferIndex = 0;

    // Setup the UART
    uart_console_init();

    // Setup Queue for sending strings.
    terminalQueue = xQueueCreate(TERMINAL_QUEUE_LENGTH, sizeof(inputBuffer));

    while (1) {

        int len = uart_read_bytes(UART_NUM, &recieved, 1, UART_DELAY);
        if (len > 0) {
            if (recieved == '\r' || recieved == '\n') {

                // Echo newline (CRLF)
                uart_write_bytes(UART_NUM, (const void*) UART_NEWLINE, UART_NEWLINE_LEN);

                // Null terminate the string
                inputBuffer[inputBufferIndex] = '\0';

                ////////////////////////////////////////////////
                // Process the input how I want for the drone //
                ////////////////////////////////////////////////

                // Clear input string & output arguments
                memset(inputBuffer, 0, sizeof(inputBuffer));
                inputBufferIndex = 0;

            } else if (recieved == '\b' || recieved == ASCII_DEL) {

                if (inputBufferIndex > 0) {

                    // Remove last char from screen and input string
                    inputBufferIndex--;
                    inputBuffer[inputBufferIndex] = '\0';
                    uart_write_bytes(UART_NUM, (const void*) UART_DEL_SEQUENCE, UART_DEL_LEN);
                }

            } else if (inputBufferIndex < INPUT_BUFFER_SIZE - 1) {

                // Add last char to input string and echo char back to sender
                inputBuffer[inputBufferIndex++] = recieved;
                uart_write_bytes(UART_NUM, (const void*) &recieved, 1);
            }
        }

        vTaskDelay(TERMINAL_DELAY);
    }

    // In-case while loop ends, delete task and relatvent malloced data
    terminal_delete();
}

/* terminal_delete()
 * -----------------
 * Function to first free any malloc'd memory and then to delete the task.
 */
void terminal_delete(void) {

    terminalHandle = NULL;
    vQueueDelete(terminalQueue);
    terminalQueue = NULL;
    vTaskDelete(NULL);
}
