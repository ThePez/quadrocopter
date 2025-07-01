/*
 ******************************************************************************
 * File: drone.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 ******************************************************************************
 */

// STD C lib headers
#include <stdio.h>
#include <string.h>
// KConfig header
#include "sdkconfig.h"
// Prebuilts
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
// Custom Components
#include "nrf24l01plus.h"

// Defines //

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_CLK 14

// Stack Sizes
#define SYS_STACK (configMINIMAL_STACK_SIZE * 2)
#define RADIO_STACK (configMINIMAL_STACK_SIZE * 2)
// Priorities
#define SYS_PRIO (tskIDLE_PRIORITY + 2)
#define RADIO_PRIO (tskIDLE_PRIORITY + 3)

// Task Function Prototypes
void system_task(void);
void radio_task(void);

// Setup Function Prototypes
void spi_bus_setup(spi_host_device_t host);
void print_task_stats(void);

// Global Variables
TaskHandle_t systemTask = NULL;
TaskHandle_t radioTask = NULL;

//////////////////////////////////////////////////////////////////////////////

void app_main(void) {

    xTaskCreate((void*) &system_task, "SYS_CONTROL", SYS_STACK, NULL, SYS_PRIO, &systemTask);
    xTaskCreate((void*) &radio_task, "RADIO", RADIO_STACK, NULL, RADIO_PRIO, &radioTask);
    while (1) {
        // print_task_stats();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void system_task(void) {

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void radio_task(void) {

    uint8_t rx_buffer[32];
    spi_bus_setup(HSPI_HOST);
    nrf24l01plus_init(HSPI_HOST);
    nrf24l01plus_recieve_mode(); // Set radio into recieving mode
    printf("NRF24L01+ RX listening...\n");

    while (1) {
        if (nrf24l01plus_recieve_packet(rx_buffer)) {
            printf("Received: ");
            // for (int i = 0; i < NRF24L01PLUS_TX_PLOAD_WIDTH; i++) {
            //     printf("%02X ", rx_buffer[i]);
            // }
            // printf("\r\n");
            printf("%s\r\n", (char*) rx_buffer);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust to your latency needs
    }
}

void spi_bus_setup(spi_host_device_t host) {

    spi_bus_config_t bus_config = {
        .miso_io_num = HSPI_MISO,
        .mosi_io_num = HSPI_MOSI,
        .sclk_io_num = HSPI_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO));
}

void print_task_stats(void) {

    uint16_t numTasks = uxTaskGetNumberOfTasks();
    uint16_t bufferLength = numTasks * 50;
    char* taskListBuffer = pvPortMalloc(bufferLength * sizeof(char));
    if (taskListBuffer == NULL) {
        return; // Malloc Failed
    }

    vTaskList(taskListBuffer);
    printf("\r\nTask          State  Priority   Stack\tID\r\n");
    printf("=============================================\r\n");
    printf("%s\r\n", taskListBuffer);
    // Free memory
    vPortFree(taskListBuffer);
}

