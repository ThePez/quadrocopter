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
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Custom Components
#include "hamming.h"
#include "mcp3208.h"
#include "nrf24l01plus.h"

/////////////////////////////////// Defines //////////////////////////////////

// SPI Pins
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

#define MAX_ANGLE (30.0)
#define MIN_ANGLE (-30.0)
#define ANGLE_RANGE (60.0)

///////////////////////////// Structures & Enums /////////////////////////////

///////////////////////////////// Prototypes /////////////////////////////////

// Task Function Prototypes
void remote_controller(void);
void remote_input_task(void);
void radio_remote_task(void);

void spi_bus_setup(spi_host_device_t host);
void encode_packet(void* input, void* output);

////////////////////////////// Global Variables //////////////////////////////
TaskHandle_t systemTask = NULL;
TaskHandle_t joystickTask = NULL;
TaskHandle_t radioTask = NULL;
QueueHandle_t radioQueue = NULL;
QueueHandle_t inputQueue = NULL;

//////////////////////////////////////////////////////////////////////////////

void app_main(void) {

    xTaskCreate((void*) &remote_controller, "REMOTE_TASK", SYS_STACK, NULL, SYS_PRIO, &systemTask);
    xTaskCreate((void*) &remote_input_task, "ADC_INPUTS", JOYSTICK_STACK, NULL, JOYSTICK_PRIO, &joystickTask);
    xTaskCreate((void*) &radio_remote_task, "RADIO_REMOTE", RADIO_STACK, NULL, RADIO_PRIO, &radioTask);

}

/* remote_controller()
 * -------------------
 * Controller task for the remote. Handle the processing of data from the ADC inputs, and 
 * forwards the encoded packet to the radio for sending.
 */
void remote_controller(void) {

    uint16_t adcValues[5] = {0};
    uint8_t packet[32] = {0};
    while (!inputQueue || !radioQueue) {
        // Idle until both queue's are created
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    while (1) {
        // Get adc values from the input queue
        if (inputQueue && xQueueReceive(inputQueue, adcValues, portMAX_DELAY) == pdTRUE) {

            float controlInputs[4];
            // Throttle
            // Convert slider. Base line is 1000us then add level * 100 for a range of 1000us to 2000us
            controlInputs[3] = (float) adcValues[4] / 4.096f + 1000.0f;
            // Pitch -> roll -> yaw
            for (uint8_t i = 0; i < 3; i++) {
                // Fix offset so that 2048 is 0 degrees, then convert number into degrees
                controlInputs[i] = ((float) adcValues[0] - 2048.0f) / 68.267f;
            }

            // Store the float data into an array of bytes
            uint8_t rawInput[16];
            memcpy(rawInput, controlInputs, sizeof(controlInputs));
            
            // Encode the inputs via hamming
            encode_packet((void*) rawInput, (void*) packet);

            // Send the resulting packet to the radio task
            if (radioQueue) {
                xQueueSendToFront(radioQueue, packet, pdMS_TO_TICKS(5));
            }
         
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* remote_input_task()
 * -------------------
 * Task handles the comunication with the MCP3208. This task requests the 5 channels of
 * adc inputs be read. This data is then passed to the remote_controller task.
 */
void remote_input_task(void) {

    uint16_t adcValues[5];
    while (!inputQueue) {
        // Loop to ensure the input queue is created
        inputQueue = xQueueCreate(5, sizeof(adcValues));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    spi_bus_setup(HSPI_HOST);
    mcp3208_init(HSPI_HOST);

    while (1) {

        for (uint8_t i = 0; i < 5; i++) {
            // Channels 0 to 3 are joysticks, 4 is slider
            adcValues[i] = mcp3208_read_adc_channel(i, 0);
        }

        if (inputQueue) {
            xQueueSendToFront(inputQueue, adcValues, pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* radio_remote_task()
 * -------------------
 * Simple task that utilises the NRF24L01plus radio module to send messages. Task
 * will wait on it's queue until a message is received, then proceed to send it.
 */
void radio_remote_task(void) {

    uint8_t toSend[NRF24L01PLUS_TX_PLOAD_WIDTH]; // Storage buffer for messages to send
    while (!radioQueue) {
        // Loop to ensure the radio queue is created
        radioQueue = xQueueCreate(2, sizeof(toSend));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Initialise SPI host and setip the radio IC
    spi_bus_setup(HSPI_HOST);
    nrf24l01plus_init(HSPI_HOST);

    while (1) {

        // Task waits until it recieves a message to send
        if (radioQueue && xQueueReceive(radioQueue, toSend, portMAX_DELAY) == pdTRUE) {
            // Message has been recieved -> send it
            nrf24l01plus_send_packet(toSend);
        }

        // Delay to ensure idle task can run to reset watchdog timer, might remove later
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* spi_bus_setup()
 * ---------------
 * Function to setup the given SPI host for use. Can only be run once.
 */
void spi_bus_setup(spi_host_device_t host) {

    static uint8_t spiInitialised = 0;

    if (spiInitialised) {
        return;
    }

    spi_bus_config_t bus_config = {
        .miso_io_num = HSPI_MISO,
        .mosi_io_num = HSPI_MOSI,
        .sclk_io_num = HSPI_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO));
    spiInitialised = 1;
}

/* encode_packet()
 * ---------------
 * Function to encode a packet of data.
 */
void encode_packet(void* input, void* output) {
    uint16_t* out = (uint16_t*) output;
    uint8_t* in = (uint8_t*) input;

    for (uint8_t i = 0; i < NRF24L01PLUS_TX_PLOAD_WIDTH / 2; i++) {
        out[i] = hamming_byte_encode(in[i]);
    }
}