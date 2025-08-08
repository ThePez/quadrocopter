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

#include "driver/gpio.h"
#include "esp_log.h"

#include "nrf24l01plus.h"

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

// Event Group used internally to signal the controlling tasks
static EventGroupHandle_t radioEventGroup = NULL;

// Buffers for the receiver & transmitter queue's
static uint8_t rxBuffer[NRF24L01PLUS_TX_PLOAD_WIDTH];
static uint8_t txBuffer[NRF24L01PLUS_TX_PLOAD_WIDTH];

// Tag for log messages
static const char* TAG = "RADIO";

//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initialize the radio module and start all related tasks.
 *
 * Sets up the NRF24L01+ radio module by initializing the hardware,
 * clearing its status flags, creating the shared event group,
 * and launching the control, receiver, and transmitter tasks.
 *
 * @param spiMutex Pointer to the SPI bus mutex for safe SPI access.
 * @param spiHost  The SPI host device connected to the radio.
 */
void radio_module_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost) {

    // Setup the NRF24L01plus IC
    xSemaphoreTake(*spiMutex, portMAX_DELAY);
    nrf24l01plus_init(spiHost, &radio_isr_handler);
    xSemaphoreGive(*spiMutex);

    // Ensure the event group is created
    while (!radioEventGroup) {
        radioEventGroup = xEventGroupCreate();
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Ensure the queue is created
    while (!radioReceiverQueue) {
        radioReceiverQueue = xQueueCreate(RADIO_QUEUE_LENGTH, sizeof(rxBuffer));
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Ensure the queue is created
    while (!radioTransmitterQueue) {
        radioTransmitterQueue = xQueueCreate(RADIO_QUEUE_LENGTH, sizeof(txBuffer));
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Start the controlling Tasks
    RadioParams_t* radioTaskParams = pvPortMalloc(sizeof(RadioParams_t));
    radioTaskParams->spiMutex = spiMutex;
    xTaskCreate(&control_task, "CTRL_RADIO", RADIO_STACK, radioTaskParams, RADIO_PRIO, &radioControlTask);
    xTaskCreate(&receiver_task, "RX_RADIO", RADIO_STACK, radioTaskParams, RADIO_PRIO + 1, &radioReceiverTask);
    xTaskCreate(&transmitter_task, "TX_RADIO", RADIO_STACK, radioTaskParams, RADIO_PRIO + 1, &radioTransmitterTask);
}

/**
 * @brief ISR handler for the NRF24L01+ IRQ pin.
 *
 * Triggered on a falling edge when the radio signals that
 * data has been received or a transmission is complete.
 * Notifies the control task using a direct task notification
 * so SPI work can run outside the interrupt context.
 *
 * @note Must reside in IRAM. Keep minimal and fast.
 */
static void IRAM_ATTR radio_isr_handler(void) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (radioControlTask) {
        vTaskNotifyGiveFromISR(radioControlTask, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief FreeRTOS task for handling NRF24L01+ status events.
 *
 * Waits for notifications from the ISR. When triggered,
 * reads the radio status register, clears flags, and
 * signals the receiver or transmitter task as needed.
 *
 * Runs continuously while the system is active.
 *
 * @param pvParams Pointer to the task parameters (RadioParams_t).
 *
 * @note Uses NRF24L01+ driver functions.
 */
static void control_task(void* pvParams) {

    // Deal with input parameters
    RadioParams_t* params = (RadioParams_t*) pvParams;
    SemaphoreHandle_t spiMutex = *(params->spiMutex);

    xSemaphoreTake(spiMutex, portMAX_DELAY);
    uint8_t clearBoth = NRF24L01PLUS_TX_DS | NRF24L01PLUS_RX_DR;
    nrf24l01plus_flush_rx();
    nrf24l01plus_flush_tx();
    nrf24l01plus_write_register(NRF24L01PLUS_STATUS, clearBoth); // Clear interrupts
    xSemaphoreGive(spiMutex);

    // Enable interrupts now that the controller task is setup
    gpio_intr_enable(NRF24L01PLUS_IQR_PIN);
    // Set bit for TX ready
    xEventGroupSetBits(radioEventGroup, RADIO_TX_READY);

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
            xEventGroupSetBits(radioEventGroup, RADIO_RX_READY);

        } else if (status & NRF24L01PLUS_TX_DS) {

            // Data was sent
            nrf24l01plus_write_register(NRF24L01PLUS_STATUS, NRF24L01PLUS_TX_DS);
            nrf24l01plus_receive_mode();
            xSemaphoreGive(spiMutex);
            ESP_LOGI(TAG, "Radio: data sent ISR");
            // Notify the Transmitter Task that further sends are now allowed
            xEventGroupSetBits(radioEventGroup, RADIO_TX_READY);

        } else {

            // Failsafe to ensure SPI is allowed again, this shouldn't ever happen
            uint8_t clearBoth = NRF24L01PLUS_TX_DS | NRF24L01PLUS_RX_DR;
            nrf24l01plus_write_register(NRF24L01PLUS_STATUS, clearBoth);
            xSemaphoreGive(spiMutex);
            ESP_LOGI(TAG, "Radio: Failsafe ISR");
        }
    }
}

/**
 * @brief FreeRTOS task for handling incoming radio packets.
 *
 * Waits for a signal from the control task that a new packet is ready.
 * Receives the packet over SPI and places the decoded data onto
 * the radioReceiverQueue for processing.
 *
 * Runs continuously while the system is active.
 *
 * @param pvParams Pointer to the task parameters (RadioParams_t).
 *
 * @note Uses NRF24L01+ driver functions.
 */
static void receiver_task(void* pvParams) {

    // Deal with input parameters
    RadioParams_t* params = (RadioParams_t*) pvParams;
    SemaphoreHandle_t spiMutex = *(params->spiMutex);

    ESP_LOGI(TAG, "Reciever task initialised");

    while (1) {

        // Wait for Control to signal a read is available
        EventBits_t bits = xEventGroupWaitBits(radioEventGroup, RADIO_RX_READY, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & RADIO_RX_READY) {

            // Wait for the SPI to be available
            xSemaphoreTake(spiMutex, portMAX_DELAY);
            nrf24l01plus_recieve_packet(rxBuffer);
            xSemaphoreGive(spiMutex);
            xQueueSendToBack(radioReceiverQueue, rxBuffer, pdMS_TO_TICKS(5));
        }
    }
}

/**
 * @brief FreeRTOS task for sending outgoing radio packets.
 *
 * Waits for a message on radioTransmitterQueue and for permission
 * to transmit. When both are ready, sends the packet to the radio
 * module over SPI.
 *
 * Runs continuously while the system is active.
 *
 * @param pvParams Pointer to the task parameters (RadioParams_t).
 *
 * @note Uses NRF24L01+ driver functions.
 */
static void transmitter_task(void* pvParams) {

    // Deal with input parameters
    RadioParams_t* params = (RadioParams_t*) pvParams;
    SemaphoreHandle_t spiMutex = *(params->spiMutex);

    ESP_LOGI(TAG, "Transmitter task initialised");

    while (1) {

        // Task waits until it recieves a message to send
        if (xQueueReceive(radioTransmitterQueue, txBuffer, portMAX_DELAY) == pdTRUE) {

            // Task checks the Event bits that sending is ready
            EventBits_t bits = xEventGroupWaitBits(radioEventGroup, RADIO_TX_READY, pdTRUE, pdFALSE, portMAX_DELAY);
            if (bits & RADIO_TX_READY) {

                // Send message
                xSemaphoreTake(spiMutex, portMAX_DELAY);
                nrf24l01plus_send_packet(txBuffer);
                xSemaphoreGive(spiMutex);
            }
        }
    }
}
