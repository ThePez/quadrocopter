/*
 *****************************************************************************
 * File: radio.c
 * Author: Jack Cairns
 * Date: 13-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "radio.h"

TaskHandle_t radioReceiverTask = NULL;
TaskHandle_t radioTransmitterTask = NULL;

QueueHandle_t radioReceiverQueue = NULL;
QueueHandle_t radioTransmitterQueue = NULL;

//////////////////////////////////////////////////////////////////////////////

/* radio_isr_handler()
 * -------------------
 * Interrupt Service Routine (ISR) for the NRF24L01+ IRQ pin.
 * This handler is triggered on a falling edge when the radio signals that
 * data is ready, a packet has been sent. The ISR notifies the radio handling
 * task by sending a direct task notification, allowing the task to safely
 * perform SPI operations outside of interrupt context.
 *
 * This ISR must be placed in IRAM and kept as short as possible.
 *
 * Parameters:
 *   None.
 */
static void IRAM_ATTR radio_isr_handler(void) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (radioReceiverTask) {
        vTaskNotifyGiveFromISR(radioReceiverTask, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* radio_receiver_task()
 * -------------
 * Initializes and runs the NRF24L01+ radio receiver.
 *
 * This task sets up the SPI bus and radio module, then listens for
 * incoming encoded packets from the remote controller.
 * Successfully received packets are decoded and translated into
 * throttle and attitude setpoints, which are sent to the flight controller
 * via a FreeRTOS queue.
 *
 * Runs continuously as a FreeRTOS task.
 *
 * Dependencies:
 *   - Uses spi_bus_setup() and nrf24l01plus_*() driver functions.
 *   - Outputs to radioReceiverQueue.
 */
void radio_receiver_task(void) {

    // Setup variables
    uint8_t rx_buffer[32];
    uint8_t decodedPacket[16];
    uint16_t inputs[5];
    ControlSetPoint_t remoteInputs = {0};

    while (!radioReceiverQueue) {
        // Loop to ensure the radio queue is created
        radioReceiverQueue = xQueueCreate(5, sizeof(ControlSetPoint_t));
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Setup hardware
    spi_bus_setup(VSPI_HOST);
    xSemaphoreTake(spiVMutex, portMAX_DELAY);
    nrf24l01plus_init(VSPI_HOST, &radio_isr_handler);
    nrf24l01plus_receive_mode();
    xSemaphoreGive(spiVMutex);

    printf("NRF24L01+ RX listening...\n");
    // Signal Transmitter that setup is complete
    xTaskNotifyGive(radioTransmitterTask);
    taskYIELD();
    xTaskNotifyGive(radioTransmitterTask);

    while (1) {

        // Wait for IQR interrupt to trigger
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Wait for the VSPI_HOST to be available
        if (xSemaphoreTake(spiVMutex, pdMS_TO_TICKS(10)) == pdTRUE) {

            // Read STATUS register
            uint8_t status = nrf24l01plus_read_register(NRF24L01PLUS_STATUS);
            printf("IQR Triggered Data\r\n");
            // Data is ready
            if (status & NRF24L01PLUS_RX_DR) {

                nrf24l01plus_recieve_packet(rx_buffer);
                // Task is done with VSPI_HOST so release the mutex
                xSemaphoreGive(spiVMutex);

                // Decode packet
                decode_packet((void*) rx_buffer, (void*) decodedPacket);
                memcpy(inputs, decodedPacket, sizeof(inputs));

                // Throttle conversion
                remoteInputs.throttle = (uint16_t) ((float) inputs[0] / 4.096f + 1000.0f);

                // Pitch, Roll & Yaw conversion with rounding & deadband
                double controlInputs[3];
                for (int i = 1; i < 4; i++) {
                    double val = ((double) inputs[i] - 2048.0) / 68.267;
                    // Round to nearest integer
                    val = round(val);
                    // Clamp small values to zero
                    if (fabs(val) < 2.0) {
                        val = 0.0;
                    }

                    controlInputs[i - 1] = val;
                }

                remoteInputs.pitch = controlInputs[0];
                remoteInputs.roll = controlInputs[1];
                remoteInputs.yaw = controlInputs[2];

                // Send input data to FC
                xQueueSendToFront(radioReceiverQueue, &remoteInputs, pdMS_TO_TICKS(5));
            }

            // Data was sent
            else if (status & NRF24L01PLUS_TX_DS) {
                // Clear TX_DS, MAX_RT bits
                nrf24l01plus_write_register(NRF24L01PLUS_STATUS, 0x30);
                nrf24l01plus_receive_mode();
                // Task is done with VSPI_HOST so release the mutex
                xSemaphoreGive(spiVMutex);
                // Signal the transmitter that it's allowed to transmit again
                xTaskNotifyGive(radioTransmitterTask);
            }
        }
    }
}

void radio_transmitter_task(void) {

    uint8_t toSend[NRF24L01PLUS_TX_PLOAD_WIDTH / 2]; // Message to encode
    uint8_t packet[NRF24L01PLUS_TX_PLOAD_WIDTH]; 
    while (!radioTransmitterQueue) {
        // Loop to ensure the radio queue is created
        radioTransmitterQueue = xQueueCreate(2, sizeof(toSend));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Wait for receiver task to finsh setting up the radio module
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

    printf("Radio Transmitter Task setup\r\n");

    while (1) {

        // Task waits until it recieves a message to send and the task was notified that sending is allowed
        if (xQueueReceive(radioTransmitterQueue, toSend, portMAX_DELAY) == pdTRUE &&
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {

            // Message has been recieved -> encode -> send it
            encode_packet((void*) toSend, (void*)packet);
            xSemaphoreTake(spiVMutex, portMAX_DELAY);
            nrf24l01plus_send_packet(packet);
            nrf24l01plus_receive_mode();
            xSemaphoreGive(spiVMutex);
        }
    }
}

/* encode_packet()
 * ----------------
 * Encodes a data packet using Hamming codes for error detection.
 *
 * Converts an array of raw input bytes into encoded 16-bit words.
 * Used to protect transmitted control data from bit errors during
 * wireless communication.
 *
 * Parameters:
 *   input  - Pointer to the raw byte array.
 *   output - Pointer to the destination buffer for encoded words.
 *
 * Dependencies:
 *   - Uses hamming_byte_encode().
 */
void encode_packet(void* input, void* output) {
    uint16_t* out = (uint16_t*) output;
    uint8_t* in = (uint8_t*) input;

    for (uint8_t i = 0; i < NRF24L01PLUS_TX_PLOAD_WIDTH / 2; i++) {
        out[i] = hamming_byte_encode(in[i]);
    }
}

/* decode_packet()
 * ---------------
 * Decodes a received data packet using Hamming decoding.
 *
 * The NRF24L01+ transmits a 32-byte encoded packet (16 words of 16 bits),
 * which this function decodes into the original 16-byte payload.
 *
 * Parameters:
 *   input  - Pointer to the raw received packet buffer.
 *   output - Pointer to the decoded output buffer.
 *
 * Dependencies:
 *   Uses hamming_word_decode() to correct single-bit errors.
 */
void decode_packet(void* input, void* output) {
    uint16_t* in = (uint16_t*) input;
    uint8_t* out = (uint8_t*) output;
    for (uint8_t i = 0; i < NRF24L01PLUS_TX_PLOAD_WIDTH / 2; i++) {
        out[i] = hamming_word_decode(in[i]);
    }
}
