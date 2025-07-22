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
#include "common.h"
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
 *   - adc_input_task: Reads analog inputs from the MCP3208 ADC.
 *   - radio_remote_task: Transmits encoded control packets via NRF24L01+.
 *
 * Runs once at boot.
 */
void app_main(void) {
    spi_bus_setup(VSPI_HOST);
    spi_bus_setup(HSPI_HOST);

    radio_module_init(&spiHMutex, HSPI_HOST);
    adc_task_init(&spiVMutex, VSPI_HOST);
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
    uint8_t packet[32] = {0};
    while (!adcInputQueue || !radioTransmitterQueue) {

        // Idle until both queue's are created
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    while (1) {
        // Get adc values from the input queue
        if (xQueueReceive(adcInputQueue, adcValues, portMAX_DELAY) == pdTRUE) {

            double inputs[4];
            // Throttle conversion
            inputs[0] = ((double) adcValues[0] / 4.095 + 1000.0);
            // Pitch, Roll & Yaw conversion
            for (int i = 1; i < 4; i++) {
                inputs[i] = (adcValues[i] - 2048.0) / 68.267;
            }

            printf("Throttle: %d, Pitch: %f, Roll: %f, Yaw: %f\r\n", (uint16_t) inputs[0], inputs[1], inputs[2],
                   inputs[3]);

            uint16_t rawInput[16];
            memset(rawInput, 0, sizeof(rawInput));
            rawInput[0] = 1;
            memcpy(rawInput + 1, adcValues, sizeof(adcValues));

            // Encode the inputs via hamming
            encode_packet((void*) rawInput, (void*) packet);

            // Send the resulting packet to the radio task
            if (radioTransmitterQueue) {
                xQueueSendToFront(radioTransmitterQueue, packet, pdMS_TO_TICKS(5));
            }
        }
    }
}
