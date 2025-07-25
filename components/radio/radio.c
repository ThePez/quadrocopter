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

//////////////////////////// Function Prototypes /////////////////////////////

// ISR Handler function
static void radio_isr_handler(void);
// Task Function Prototypes
static void control_task(void* pvParams);
static void receiver_task(void* pvParams);
static void transmitter_task(void* pvParams);

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t radioReceiverTask = NULL;
TaskHandle_t radioTransmitterTask = NULL;
TaskHandle_t radioControlTask = NULL;
// Queue Handles
QueueHandle_t radioReceiverQueue = NULL;
QueueHandle_t radioTransmitterQueue = NULL;
// Tag for log messages
static const char* TAG = "RADIO";

//////////////////////////////////////////////////////////////////////////////

/* radio_module_init()
 * -------------------
 *
 */
void radio_module_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost) {

    // Setup the NRF24L01plus IC
    xSemaphoreTake(*spiMutex, portMAX_DELAY);
    nrf24l01plus_init(spiHost, &radio_isr_handler);
    uint8_t clearBoth = NRF24L01PLUS_TX_DS | NRF24L01PLUS_RX_DR;
    nrf24l01plus_write_register(NRF24L01PLUS_STATUS, clearBoth);
    xSemaphoreGive(*spiMutex);

    // Start the controlling Tasks
    RadioParams_t* radioTaskParams = pvPortMalloc(sizeof(RadioParams_t));
    radioTaskParams->spiMutex = spiMutex;
    xTaskCreate(&control_task, "CON_RADIO", RADIO_STACK, radioTaskParams, RADIO_PRIO, &radioControlTask);
    xTaskCreate(&receiver_task, "RX_RADIO", RADIO_STACK, radioTaskParams, RADIO_PRIO + 1, &radioReceiverTask);
    xTaskCreate(&transmitter_task, "TX_RADIO", RADIO_STACK, radioTaskParams, RADIO_PRIO + 1, &radioTransmitterTask);
}

/* radio_isr_handler()
 * -------------------
 * Interrupt Service Routine (ISR) for the NRF24L01+ IRQ pin.
 *
 * This ISR is triggered by the radio module on a falling edge whenever a packet
 * is received or a transmission is complete. The ISR signals the radio control
 * task using a direct task notification so that SPI operations can be handled
 * safely outside of interrupt context.
 *
 * Notes:
 *   - Must be placed in IRAM.
 *   - Must remain as short as possible.
 *
 * Parameters:
 *   None.
 */
static void IRAM_ATTR radio_isr_handler(void) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (radioControlTask) {
        vTaskNotifyGiveFromISR(radioControlTask, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* control_task()
 * ---------------------
 * Radio control task for handling NRF24L01+ status events.
 *
 * This task performs initial radio hardware setup (if it wins the setup mutex race).
 * It then continuously waits for interrupts from the radio ISR indicating that
 * new data is ready or a transmission has completed.
 *
 * When a receive interrupt occurs, the control task clears the status register
 * and notifies the receiver task that a packet is available.
 *
 * When a transmit interrupt occurs, it clears the status register and notifies
 * the transmitter task that sending is allowed again.
 *
 * Runs continuously as a FreeRTOS task.
 *
 * Dependencies:
 *   - Uses radio_setup() to initialize hardware.
 *   - Notifies radioTransmitterTask and radioReceiverTask when needed.
 */
static void control_task(void* pvParams) {

    // Deal with input parameters
    RadioParams_t* params = (RadioParams_t*) pvParams;
    SemaphoreHandle_t spiMutex = *(params->spiMutex);

    // Wait until transmitter task is alive
    while (!radioTransmitterTask) {
        vTaskDelay(pdMS_TO_TICKS(10));
        // Initial signal to transmitter that sends are allowed
        if (radioTransmitterTask) {
            xTaskNotifyGive(radioTransmitterTask);
        }
    }
    
    // Enable interrupts now that the controller task is setup
    gpio_intr_enable(NRF24L01PLUS_IQR_PIN);
    ESP_LOGI(TAG, "Control task initialised");
    while (1) {

        // Wait for IQR interrupt handler signal
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Wait for SPI to be available
        xSemaphoreTake(spiMutex, portMAX_DELAY);
        // Read status register
        uint8_t status = nrf24l01plus_read_register(NRF24L01PLUS_STATUS);
        if (status & NRF24L01PLUS_RX_DR) {

            // Data available
            xSemaphoreGive(spiMutex);
            ESP_LOGI(TAG, "Radio: date available ISR");
            // Notify the Receiver Task
            if (radioReceiverTask) {
                xTaskNotifyGive(radioReceiverTask);
            }

        } else if (status & NRF24L01PLUS_TX_DS) {
            
            // Data was sent
            nrf24l01plus_write_register(NRF24L01PLUS_STATUS, NRF24L01PLUS_TX_DS);
            nrf24l01plus_receive_mode();
            xSemaphoreGive(spiMutex);
            ESP_LOGI(TAG, "Radio: data sent ISR");
            // Notify the Transmitter Task that further sends are now allowed
            if (radioTransmitterTask) {
                xTaskNotifyGive(radioTransmitterTask);
            }

        } else {

            // Failsafe to ensure SPI is allowed again, this shouldn't ever happen
            uint8_t clearBoth = NRF24L01PLUS_TX_DS | NRF24L01PLUS_RX_DR;
            nrf24l01plus_write_register(NRF24L01PLUS_STATUS, clearBoth);
            xSemaphoreGive(spiMutex);
            ESP_LOGI(TAG, "Radio: Failsafe ISR");
        }
    }
}

/* receiver_task()
 * ----------------------
 * Radio receiver task for handling incoming packets.
 *
 * This task participates in the shared radio hardware setup (via radio_setup()).
 * It waits for a signal from the control task indicating a new packet is ready,
 * then receives the packet over SPI, decodes it, and forwards the result to
 * the flight controller via the radioReceiverQueue.
 *
 * Runs continuously as a FreeRTOS task.
 *
 * Dependencies:
 *   - Uses radio_setup() to ensure hardware is initialized.
 *   - Uses nrf24l01plus_*() driver functions.
 *   - Outputs decoded packets to radioReceiverQueue.
 */
static void receiver_task(void* pvParams) {

    // Deal with input parameters
    RadioParams_t* params = (RadioParams_t*) pvParams;
    SemaphoreHandle_t spiMutex = *(params->spiMutex);

    uint8_t rxBuffer[32];
    uint8_t payload[16];
    while (!radioReceiverQueue) {

        // Loop to ensure the queue is created
        radioReceiverQueue = xQueueCreate(RADIO_QUEUE_LENGTH, sizeof(payload));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Reciever task initialised");
    while (1) {

        // Wait for Control to signal a read is available
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Wait for the SPI to be available
        xSemaphoreTake(spiMutex, portMAX_DELAY);
        nrf24l01plus_recieve_packet(rxBuffer);
        xSemaphoreGive(spiMutex);
        // Decode packet
        decode_packet((void*) rxBuffer, (void*) payload);
        // Send input data to Queue for processing
        xQueueSendToBack(radioReceiverQueue, payload, pdMS_TO_TICKS(5));
    }
}

/* transmitter_task()
 * --------------------------
 * Radio transmitter task for sending packets.
 *
 * This task participates in the shared radio hardware setup (via radio_setup()).
 * It waits until both a packet is ready to send (from radioTransmitterQueue)
 * and the control task has signaled that sending is allowed.
 *
 * Once ready, it encodes the message, sends it over SPI to the radio module,
 * and waits for the next transmission opportunity.
 *
 * Runs continuously as a FreeRTOS task.
 *
 * Dependencies:
 *   - Uses radio_setup() to ensure hardware is initialized.
 *   - Uses nrf24l01plus_*() driver functions.
 *   - Waits on radioTransmitterQueue for outgoing packets.
 */
static void transmitter_task(void* pvParams) {

    // Deal with input parameters
    RadioParams_t* params = (RadioParams_t*) pvParams;
    SemaphoreHandle_t spiMutex = *(params->spiMutex);

    uint8_t payload[NRF24L01PLUS_TX_PLOAD_WIDTH / 2]; // Message to encode
    uint8_t packet[NRF24L01PLUS_TX_PLOAD_WIDTH];      // Encoded message
    while (!radioTransmitterQueue) {

        // Loop to ensure the radio queue is created
        radioTransmitterQueue = xQueueCreate(RADIO_QUEUE_LENGTH, sizeof(payload));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Transmitter task initialised");
    while (1) {

        // Task waits until it recieves a message to send and the task was notified that sending is allowed
        if (xQueueReceive(radioTransmitterQueue, payload, portMAX_DELAY) == pdTRUE &&
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {

            // Message has been recieved -> encode -> send it
            encode_packet((void*) payload, (void*) packet);
            xSemaphoreTake(spiMutex, portMAX_DELAY);
            nrf24l01plus_send_packet(packet);
            xSemaphoreGive(spiMutex);
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
