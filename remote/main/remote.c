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
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_CLK 18

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
void adc_input_task(void);
void radio_remote_task(void);

void spi_bus_setup(spi_host_device_t host);
void encode_packet(void* input, void* output);

////////////////////////////// Global Variables //////////////////////////////

TaskHandle_t systemTask = NULL;
TaskHandle_t joystickTask = NULL;
TaskHandle_t radioTask = NULL;

QueueHandle_t radioQueue = NULL;
QueueHandle_t inputQueue = NULL;

// Ensure the SPI bus is only setup once
static uint8_t spiVBusInitialised = 0;
static uint8_t spiHBusInitialised = 0;

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

    xTaskCreate((void*) &remote_controller, "REMOTE_TASK", SYS_STACK, NULL, SYS_PRIO, &systemTask);
    xTaskCreate((void*) &adc_input_task, "ADC_INPUTS", JOYSTICK_STACK, NULL, JOYSTICK_PRIO, &joystickTask);
    xTaskCreate((void*) &radio_remote_task, "RADIO_REMOTE", RADIO_STACK, NULL, RADIO_PRIO, &radioTask);
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
    while (!inputQueue || !radioQueue) {
        // Idle until both queue's are created
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    while (1) {
        // Get adc values from the input queue
        if (inputQueue && xQueueReceive(inputQueue, adcValues, portMAX_DELAY) == pdTRUE) {

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
            if (radioQueue) {
                xQueueSendToFront(radioQueue, packet, pdMS_TO_TICKS(5));
            }
        }
    }
}

/* adc_input_task()
 * --------------------
 * Continuously reads analog joystick and slider inputs using the MCP3208 ADC.
 *
 * Initializes the SPI bus for the ADC and reads five channels:
 *   - Channels 0–3: Joystick axes
 *   - Channel 4: Slider or other analog input
 *
 * Collected data is sent to the remote_controller task via a queue.
 *
 * Runs continuously as a FreeRTOS task.
 */
void adc_input_task(void) {

    uint16_t adcValues[5];
    while (!inputQueue) {
        // Loop to ensure the input queue is created
        inputQueue = xQueueCreate(5, sizeof(adcValues));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    spi_bus_setup(VSPI_HOST);
    mcp3208_init(VSPI_HOST);

    while (1) {

        for (uint8_t i = 0; i < 5; i++) {
            // Channels 1 to 4 are joysticks, 0 is slider
            adcValues[i] = mcp3208_read_adc_channel(i, MCP3208_SINGLE);
        }

        if (inputQueue) {
            xQueueSendToFront(inputQueue, adcValues, pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* radio_remote_task()
 * --------------------
 * Handles wireless data transmission using the NRF24L01+ module.
 *
 * Waits on the radio queue for an encoded packet from the remote_controller,
 * then transmits it via the NRF24L01+ radio.
 *
 * Responsibilities:
 *   - Initializes the SPI bus and radio module.
 *   - Waits for new packets to send.
 *   - Sends packets wirelessly.
 *
 * Runs continuously as a FreeRTOS task.
 */
void radio_remote_task(void) {

    uint8_t toSend[NRF24L01PLUS_TX_PLOAD_WIDTH]; // Storage buffer for messages to send
    while (!radioQueue) {
        // Loop to ensure the radio queue is created
        radioQueue = xQueueCreate(2, sizeof(toSend));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Initialise SPI host and setup the radio IC
    spi_bus_setup(HSPI_HOST);
    nrf24l01plus_init(HSPI_HOST, NULL);
    printf("RADIO setup\r\n");

    while (1) {

        // Task waits until it recieves a message to send
        if (radioQueue && xQueueReceive(radioQueue, toSend, portMAX_DELAY) == pdTRUE) {
            // Message has been recieved -> send it
            nrf24l01plus_send_packet(toSend);
        }
    }
}

/* spi_bus_setup()
 * ----------------
 * Initializes the specified SPI bus (HSPI or VSPI) for use with
 * either the ADC or the radio module.
 *
 * Ensures that each bus is only initialized once to avoid redundant setup.
 *
 * Parameters:
 *   host - The SPI bus to initialize (HSPI_HOST or VSPI_HOST).
 *
 * Dependencies:
 *   - Uses ESP-IDF SPI Master driver.
 *   - Tracks initialization state with static flags.
 */
void spi_bus_setup(spi_host_device_t host) {

    if ((host == VSPI_HOST && spiVBusInitialised) || (host == HSPI_HOST && spiHBusInitialised)) {
        return;
    }

    spi_bus_config_t bus_config = {
        .miso_io_num = (host == HSPI_HOST) ? HSPI_MISO : VSPI_MISO,
        .mosi_io_num = (host == HSPI_HOST) ? HSPI_MOSI : VSPI_MOSI,
        .sclk_io_num = (host == HSPI_HOST) ? HSPI_CLK : VSPI_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO));

    if (host == HSPI_HOST) {
        spiHBusInitialised = 1;
    } else if (host == VSPI_HOST) {
        spiVBusInitialised = 1;
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
