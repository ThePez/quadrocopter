/*
 ******************************************************************************
 * File: remote.c
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
#include "hamming.h"
#include "nrf24l01plus.h"

// Defines //

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_CLK 14

// Stack Sizes
#define SYS_STACK (configMINIMAL_STACK_SIZE * 2)
#define JOYSTICK_STACK (configMINIMAL_STACK_SIZE * 2)
#define RADIO_STACK (configMINIMAL_STACK_SIZE * 2)
// Priorities
#define SYS_PRIO (tskIDLE_PRIORITY + 2)
#define JOYSTICK_PRIO (tskIDLE_PRIORITY + 4)
#define RADIO_PRIO (tskIDLE_PRIORITY + 3)

// Task Function Prototypes
void system_task(void);
void joystick_task(void);
void radio_remote_task(void);

// Setup Function Prototypes
void spi_bus_setup(spi_host_device_t host);
void print_task_stats(void);

// Global Variables
TaskHandle_t systemTask = NULL;
TaskHandle_t joystickTask = NULL;
TaskHandle_t radioTask = NULL;

//////////////////////////////////////////////////////////////////////////////

void app_main(void) {

    xTaskCreate((void*) &system_task, "SYS_CONTROL", SYS_STACK, NULL, SYS_PRIO, &systemTask);
    xTaskCreate((void*) &joystick_task, "JOYSTICK", JOYSTICK_STACK, NULL, JOYSTICK_PRIO, &joystickTask);
    xTaskCreate((void*) &radio_remote_task, "RADIO_REMOTE", RADIO_STACK, NULL, RADIO_PRIO, &radioTask);
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

void joystick_task(void) {

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void radio_remote_task(void) {

    spi_bus_setup(HSPI_HOST);
    nrf24l01plus_init(HSPI_HOST);

    uint8_t data[32];
    memset(data, 0, sizeof(data));

    const char* word = "Hello World!";
    for (int i = 0; i < strlen(word); i++) {
        uint16_t encodded = hamming_byte_encode(word[i]);
        data[i * 2] = (uint8_t) encodded;
        data[i * 2 + 1] = (uint8_t) (encodded >> 8);
    }
    
    printf("Sending: ");
    for (int i = 0; i < 32; i++) {
        printf("%02x ", data[i]);
    }
    printf("\r\n");

    while (1) { 
        nrf24l01plus_send_packet(data);
        vTaskDelay(pdMS_TO_TICKS(500));
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

void encode_radio_transmition(uint8_t* input, uint8_t* output) {

    uint16_t message[16];
    for (int i = 0; i < NRF24L01PLUS_TX_PLOAD_WIDTH / 2; i++) {
        message[i] = hamming_byte_encode(input[i]);
    }

    output = 
}