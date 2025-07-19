/*
 *****************************************************************************
 * File: sensor-fusion.c
 * Author: Jack Cairns
 * Date: 17-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "sensor_fusion.h"

#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

// Prototypes
void fusion_hardware_reset(void);
int fusion_hal_open(sh2_Hal_t* self);
void fusion_hal_close(sh2_Hal_t* self);
int fusion_hal_read(sh2_Hal_t* self, uint8_t* pBuffer, unsigned int len, uint32_t* timeUS);
int fusion_hal_write(sh2_Hal_t* self, uint8_t* pBuffer, unsigned int len);
uint32_t fusion_hal_getTimeUs(sh2_Hal_t* self);
void fusion_hal_callback(void* cookie, sh2_AsyncEvent_t* pEvent);
void fusion_sensor_handler(void* cookie, sh2_SensorEvent_t* event);

// Global vaiables
TaskHandle_t fusionTaskHandle = NULL;
QueueHandle_t fusionQueue = NULL;

// SPI
static spi_device_handle_t fusionSPIHandle;
static SemaphoreHandle_t spiMutex = NULL;
// static txZeros[SH2_HAL_MAX_TRANSFER_IN] = {0};

// SH2 objects
static sh2_Hal_t fusionHal;
static sh2_SensorValue_t sensorValue;

/* fusion_isr_handler()
 * -------------------
 *
 * Notes:
 *   - Must be placed in IRAM.
 *   - Must remain as short as possible.
 *
 * Parameters:
 *   None.
 */
static void IRAM_ATTR fusion_isr_handler(void) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (fusionTaskHandle) {
        vTaskNotifyGiveFromISR(fusionTaskHandle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* fusion_spi_init()
 * -----------------------
 * Initializes the SPI device handle for the BNO085 fusion sensor.
 * Configures the clock speed, SPI mode (CPOL=0, CPHA=0), chip select pin,
 * and adds the device to the specified SPI bus.
 *
 * Parameters:
 *   spi_bus - The SPI bus to use (e.g., HSPI_HOST).
 */
void fusion_spi_init(spi_host_device_t spiBus) {

    // Setup the SPI for the radio module
    spi_device_interface_config_t deviceConfig = {
        .clock_speed_hz = 1000000,     // 1 MHz Clock speed
        .spics_io_num = FUSION_CS_PIN, // Chip Select pin for device
        .queue_size = 1,               // Number of pending transactions allowed
        .mode = 3                      /* SPI mode, representing a pair of (CPOL, CPHA). CPOL = 1 (clock idles low)
                                         CPHA = 1 (data is sampled on the rising edge, changed on the falling edge) */
    };

    ESP_ERROR_CHECK(spi_bus_add_device(spiBus, &deviceConfig, &fusionSPIHandle));
}

/* fusion_interrupt_init()
 * -----------------------------
 * Configures the GPIO pin used for the BNO085 INT pin as an external interrupt source.
 * Sets the pin direction to input, enables an internal pull-up resistor (to handle the
 * open-drain active-low INT signal), and configures the interrupt to trigger on a falling edge.
 * Installs the GPIO ISR service if not already installed, and attaches the user-provided
 * interrupt handler to the INT pin.
 *
 * Parameters:
 *   handler - A pointer to the ISR handler function to be called when the interrupt fires.
 */
void fusion_interrupt_init(void* handler) {

    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT,                  // Direction of the pin
        .pin_bit_mask = (1ULL << FUSION_INT_PIN), // Pin is listed here as a bit mask
        .pull_up_en = GPIO_PULLUP_ENABLE,         // Enable pull-up (INT is active low)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    // No internal pull-down
        .intr_type = GPIO_INTR_NEGEDGE            // Interrupt on the falling edge
    };

    ESP_ERROR_CHECK(gpio_config(&config));

    // Install the ISR service if not already installed
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    // Hook ISR handler for the IQR pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(FUSION_INT_PIN, handler, NULL));
}

void fusion_reset_init(void) {

    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT_OUTPUT,             // Direction of the pin
        .pin_bit_mask = (1ULL << FUSION_RESET_PIN), // Pin is listed here as a bit mask
        .pull_up_en = GPIO_PULLUP_ENABLE,           // Enable pull-up (IRQ is active low)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      // No internal pull-down
        .intr_type = GPIO_INTR_DISABLE              // No Interrupts
    };

    ESP_ERROR_CHECK(gpio_config(&config));
}

void fusion_hardware_reset(void) {
    gpio_set_level(FUSION_RESET_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(FUSION_RESET_PIN, 0);
    esp_rom_delay_us(20);
    gpio_set_level(FUSION_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Datasheet: wait for boot
}

int fusion_hal_open(sh2_Hal_t* self) {

    fusion_hardware_reset();
    return SH2_OK;
}

void fusion_hal_close(sh2_Hal_t* self) {

    // Nothing needed
}

int fusion_hal_read(sh2_Hal_t* self, uint8_t* pBuffer, unsigned int len, uint32_t* timeUS) {

    // Read 4-byte header
    spi_transaction_t transactionHeader = {
        .rx_buffer = pBuffer,
        .tx_buffer = NULL,
        .length = 32 // Bits
    };

    esp_err_t err = spi_device_transmit(fusionSPIHandle, &transactionHeader);
    if (err != ESP_OK) {
        return 0;
    }

    uint16_t packet_size = (uint16_t) pBuffer[0] | ((uint16_t) pBuffer[1] << 8);
    packet_size &= ~0x8000; // clear continue bit

    if (packet_size > len) {
        return 0;
    }

    spi_transaction_t transactionBody = {
        .rx_buffer = pBuffer,
        .tx_buffer = NULL,
        .length = (packet_size) * 8 // Bits
    };

    // uint8_t* dummyBuffer = malloc(sizeof(uint8_t) * packet_size);
    // memset(dummyBuffer, 0xFF, sizeof(dummyBuffer));
    // transaction.tx_buffer = dummyBuffer;
    // transaction.rx_buffer = pBuffer;
    // transaction.length = packet_size * 8;
    err = spi_device_transmit(fusionSPIHandle, &transactionBody);
    // free(dummyBuffer);
    if (err != ESP_OK) {
        return 0;
    }

    *timeUS = esp_timer_get_time();
    return packet_size;
}

int fusion_hal_write(sh2_Hal_t* self, uint8_t* pBuffer, unsigned int len) {

    spi_transaction_t transaction = {
        .length = len * 8,
        .tx_buffer = pBuffer,
        .rx_buffer = NULL,
    };

    esp_err_t err = spi_device_transmit(fusionSPIHandle, &transaction);
    if (err == ESP_OK) {
        return len;
    }
    return 0;
}

uint32_t fusion_hal_getTimeUs(sh2_Hal_t* self) {
    return esp_timer_get_time();
}

void fusion_hal_callback(void* cookie, sh2_AsyncEvent_t* pEvent) {
    if (pEvent->eventId == SH2_RESET) {
    }
}

void fusion_sensor_handler(void* cookie, sh2_SensorEvent_t* event) {

    SensorData_t data;
    uint8_t gyro = 0;
    uint8_t rotate = 0;
    int rc = sh2_decodeSensorEvent(&sensorValue, event);
    if (rc != SH2_OK) {

        sensorValue.timestamp = 0;
        return;
    }

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        data.qw = sensorValue.un.rotationVector.real;
        data.qx = sensorValue.un.rotationVector.i;
        data.qy = sensorValue.un.rotationVector.j;
        data.qz = sensorValue.un.rotationVector.k;
        rotate = 1;
    }

    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        data.gx = sensorValue.un.gyroscope.x;
        data.gy = sensorValue.un.gyroscope.y;
        data.gz = sensorValue.un.gyroscope.z;
        gyro = 1;
    }

    // Now send data to IMU Task from here.
    if (fusionQueue && (gyro || rotate)) {
        printf("FUSION DATA SENT\r\n");
        xQueueSendToFront(fusionQueue, &data, pdMS_TO_TICKS(1));
    }
}

void fusion_init(spi_host_device_t spiHost, void* handler) {

    fusion_spi_init(spiHost);
    fusion_interrupt_init(handler);
    fusion_reset_init();
    printf("FUSION HARDWARE DONE\r\n");
    fusion_hardware_reset();

    // Fill HAL
    fusionHal.open = fusion_hal_open;
    fusionHal.close = fusion_hal_close;
    fusionHal.read = fusion_hal_read;
    fusionHal.write = fusion_hal_write;
    fusionHal.getTimeUs = fusion_hal_getTimeUs;

    ESP_ERROR_CHECK(sh2_open(&fusionHal, fusion_hal_callback, NULL));

    printf("FUSION OPEN DONE\r\n");

    sh2_SensorConfig_t config = {.reportInterval_us = 10000, // 100 Hz
                                 .changeSensitivityEnabled = false,
                                 .wakeupEnabled = false,
                                 .alwaysOnEnabled = true};

    ESP_ERROR_CHECK(sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config));
    ESP_ERROR_CHECK(sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config));

    // Register sensor callback function
    ESP_ERROR_CHECK(sh2_setSensorCallback(fusion_sensor_handler, NULL));
    printf("FUSION INIT COMPLETE\r\n");
}

void fusion_task(void* pvParams) {

    FusionParams_t* params = (FusionParams_t*) pvParams;
    spiMutex = *(params->spiMutex);

    while (!fusionQueue) {
        fusionQueue = xQueueCreate(FUSION_QUEUE_LENGTH, sizeof(SensorData_t));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    fusion_init(params->spiHost, &fusion_isr_handler);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xSemaphoreTake(spiMutex, portMAX_DELAY);
        sh2_service();
        printf("FUSION SERVICE DONE\r\n");
        xSemaphoreGive(spiMutex);
    }
}
