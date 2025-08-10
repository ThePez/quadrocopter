/*
 ******************************************************************************
 * File: remote.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 ******************************************************************************
 */

#include "remote.h"
#include "common_functions.h"
#include "joystick_inputs.h"
#include "radio.h"

////////////////////////////// Global Variables //////////////////////////////

// Handle for the remote_controller FreeRTOS task
TaskHandle_t remoteController = NULL;

//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Main entry point for the remote controller firmware.
 *
 * Sets up SPI buses and initializes the radio and joystick modules.
 * Creates and launches the main `remote_controller` task.
 *
 * Tasks launched:
 *  - remote_controller: System controller.
 *  - joysticks task: (started in `joysticks_module_init`) Reads analog inputs via MCP3208.
 *  - radio tasks: (started in `radio_module_init`) Handles NRF24L01+ communication.
 */
void app_main(void) {
    spi_bus_setup(VSPI_HOST);
    spi_bus_setup(HSPI_HOST);

    radio_module_init(&spiHMutex, HSPI_HOST);
    joysticks_module_init(&spiVMutex, VSPI_HOST);
    xTaskCreate((void*) &remote_controller, "REMOTE_TASK", REMOTE_STACK, NULL, REMOTE_PRIO, &remoteController);
}

/**
 * @brief FreeRTOS task that collect and transmits control data.
 *
 * Waits for incoming joystick ADC data from the joystick input queue,
 * forms a control packet, and sends it to the radio transmitter queue.
 * It can also receive and print feedback from the radio receiver queue.
 *
 * Behavior:
 * - Waits until all required queues (joystick, transmitter, receiver) are ready.
 * - Receives 5 ADC values (throttle, pitch, roll, yaw, slider) from `joysticksQueue`.
 * - Builds a 16-element packet:
 *     - Index 0: Command ID (currently fixed to 1 for "setpoint update").
 *     - Index 1–5: Control values. These are currently manually overridden.
 *     - Index 6-15: '0', Filler values.
 * - Sends the packet to `radioTransmitterQueue` for NRF24L01+ transmission.
 * - Optionally receives and prints debug values from `radioReceiverQueue`.
 *
 * Notes:
 * - ADC values received from joystick task are replaced by hardcoded midpoint values (2048),
 *   likely for testing.
 *
 * This task runs indefinitely.
 */
void remote_controller(void) {

    uint16_t adcValues[5] = {0};
    uint16_t adcPacket[16] = {0};

    // Idle until all queue's are created
    while (!joysticksQueue || !radioTransmitterQueue || !radioReceiverQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    while (1) {
        // Get adc values from the input queue
        if (xQueueReceive(joysticksQueue, adcValues, portMAX_DELAY) == pdTRUE) {

            // Printing of inputs for Debugging
            
            // float inputs[4];
            // // Throttle conversion
            // inputs[0] = mapf(adcValues[0], ADC_MIN, ADC_MAX, MIN_THROTTLE, MAX_THROTTLE);
            // // Pitch, Roll & Yaw conversion
            // for (int i = 1; i < 4; i++) {
            //     inputs[i] = mapf(adcValues[i], ADC_MIN, ADC_MAX, MIN_RATE, MAX_RATE);
            // }
            // printf("Throttle: %f, Pitch: %f, Roll: %f, Yaw: %f\r\n", inputs[0], inputs[1], inputs[2],
            //        inputs[3]);

            memset(adcPacket, 0, sizeof(adcPacket));
            adcPacket[0] = 1;            // Setpoint update
            adcPacket[1] = adcValues[0]; // Throttle
            adcPacket[2] = adcValues[1]; // Pitch
            adcPacket[3] = adcValues[2]; // Roll
            adcPacket[4] = adcValues[3]; // Yaw
            adcPacket[5] = adcValues[4]; // Mode

            // Send the resulting packet to the radio task
            if (radioTransmitterQueue) {
                xQueueSendToFront(radioTransmitterQueue, adcPacket, pdMS_TO_TICKS(5));
            }
        }

        if (xQueueReceive(radioReceiverQueue, adcPacket, 0) == pdTRUE) {
            printf("Motors: %d %d %d %d\r\n", adcPacket[0], adcPacket[1], adcPacket[2], adcPacket[3]);
        }
    }
}
