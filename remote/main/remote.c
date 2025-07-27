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

TaskHandle_t remoteController = NULL;

//////////////////////////////////////////////////////////////////////////////

/* app_main()
 * ----------
 * Main entry point for the remote controller firmware.
 *
 * Creates and launches three FreeRTOS tasks:
 *   - remote_controller: Handles collecting ADC joystick data and forwarding it.
 *   - joysticks_task: Reads analog inputs from the MCP3208 ADC.
 *   - radio_remote_task: Transmits encoded control packets via NRF24L01+.
 *
 * Runs once at boot.
 */
void app_main(void) {
    spi_bus_setup(VSPI_HOST);
    spi_bus_setup(HSPI_HOST);

    radio_module_init(&spiHMutex, HSPI_HOST);
    joysticks_module_init(&spiVMutex, VSPI_HOST);
    xTaskCreate((void*) &remote_controller, "REMOTE_TASK", REMOTE_STACK, NULL, REMOTE_PRIO, &remoteController);
}

/* remote_controller()
 * --------------------
 * Main controller task for the remote unit.
 *
 * Waits for new ADC input data from the input queue, encodes it using
 * Hamming code for error detection, then forwards the resulting packet
 * to the radio queue for transmission.
 *
 * Responsibilities:
 *   - Blocks until both input and radio queues are ready.
 *   - Receives raw ADC channel data.
 *   - Encodes the data with Hamming codes for robust transmission.
 *   - Sends encoded packets to the radio task.
 *
 * Runs continuously as a FreeRTOS task.
 */
void remote_controller(void) {

    uint16_t adcValues[5] = {0};
    uint16_t adcPacket[16] = {0}; // normal packet is 32 uint8_t's -> as uint16_t's are used array is halvied

    // Idle until all queue's are created
    while (!joysticksQueue || !radioTransmitterQueue || !radioReceiverQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    while (1) {
        // Get adc values from the input queue
        if (xQueueReceive(joysticksQueue, adcValues, portMAX_DELAY) == pdTRUE) {

            double inputs[4];
            // Throttle conversion
            inputs[0] = ((double) adcValues[0] / 4.095 + 1000.0);
            // Pitch, Roll & Yaw conversion
            for (int i = 1; i < 4; i++) {
                inputs[i] = (adcValues[i] - 2048.0) / 68.267;
            }

            // printf("Throttle: %d, Pitch: %f, Roll: %f, Yaw: %f\r\n", (uint16_t) inputs[0], inputs[1], inputs[2],
            //        inputs[3]);

            memset(adcPacket, 0, sizeof(adcPacket));
            adcPacket[0] = 1; // Setpoint update
            memcpy(adcPacket + 1, adcValues, sizeof(adcValues));

            adcPacket[1] = 1300;
            // Send the resulting packet to the radio task
            if (radioTransmitterQueue) {
                xQueueSendToFront(radioTransmitterQueue, adcPacket, pdMS_TO_TICKS(5));
            }
        }

        if (xQueueReceive(radioReceiverQueue, adcPacket, 0) == pdTRUE) {
            for (int i = 0; i < 4; i++) {
                printf("Motor %d: %d ", i + 1, adcPacket[i]);
            }
            printf("\r\n");
        }
    }
}
