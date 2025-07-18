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
#include <math.h>

spi_device_handle_t fusionSPIHandle;
TaskHandle_t fusionTaskHandle = NULL;
static uint8_t resetInitialised = 0;

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

void fusion_reset_pin_init(void) {

    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT_OUTPUT,             // Direction of the pin
        .pin_bit_mask = (1ULL << FUSION_RESET_PIN), // Pin is listed here as a bit mask
        .pull_up_en = GPIO_PULLUP_ENABLE,           // Enable pull-up (IRQ is active low)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      // No internal pull-down
        .intr_type = GPIO_INTR_DISABLE              // No Interrupts
    };

    ESP_ERROR_CHECK(gpio_config(&config));
    resetInitialised = 1;
}

static void fusion_hardware_reset(void) {
    gpio_set_level(FUSION_RESET_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(FUSION_RESET_PIN, 0);
    esp_rom_delay_us(20);
    gpio_set_level(FUSION_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Datasheet: wait for boot
}

static int fusion_hal_open(sh2_Hal_t* self) {

    fusion_hardware_reset();
    return SH2_OK;
}

static void fusion_hal_close(sh2_Hal_t* self) {

    // Nothing needed
}

static int fusion_hal_read(sh2_Hal_t* self, uint8_t* pBuffer, uint32_t len, uint32_t* pBytesRead) {
    uint8_t txDummy[2] = {0x00, 0x00};

    spi_transaction_t t = {
        .length = len * 8,
        .rx_buffer = pBuffer,
        .tx_buffer = txDummy,
    };
    esp_err_t err = spi_device_transmit(fusionSPIHandle, &t);
    if (err == ESP_OK) {
        *pBytesRead = len;
        return SH2_OK;
    }
    return SH2_ERR_IO;
}

static int fusion_hal_write(sh2_Hal_t* self, uint8_t* pBuffer, uint32_t len) {
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = pBuffer,
    };
    esp_err_t err = spi_device_transmit(fusionSPIHandle, &t);
    return err == ESP_OK ? SH2_OK : SH2_ERR_IO;
}

static uint32_t fusion_hal_getTimeUs(sh2_Hal_t* self) {
    return esp_timer_get_time();
}

void quatToEuler(float qw, float qx, float qy, float qz, float* roll, float* pitch, float* yaw) {

    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (pow(qx, 2) + pow(qy, 2));
    *roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        *pitch = asinf(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (pow(qy, 2) + pow(qz, 2));
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

static void fusion_sensor_handler(void* cookie, sh2_SensorEvent_t* pEvent) {

    int rc = sh2_decodeSensorEvent(&sensorValue, pEvent);
    if (rc != SH2_OK) {

        sensorValue.timestamp = 0;
        return;
    }

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        float qw = sensorValue.un.rotationVector.real;
        float qx = sensorValue.un.rotationVector.i;
        float qy = sensorValue.un.rotationVector.j;
        float qz = sensorValue.un.rotationVector.k;

        float roll, pitch, yaw;
        quatToEuler(qw, qx, qy, qz, &roll, &pitch, &yaw);
    }

    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        float x = sensorValue.un.gyroscope.x;
        float y = sensorValue.un.gyroscope.y;
        float z = sensorValue.un.gyroscope.z;
    }

    // Now send data to IMU Task from here.
}

void fusion_init(spi_host_device_t spiHost, void* handler) {

    fusion_spi_init(spiHost);
    fusion_interrupt_init(handler);
    fusion_reset_pin_init();

    // Fill HAL
    fusionHal.open = fusion_hal_open;
    fusionHal.close = fusion_hal_close;
    fusionHal.read = fusion_hal_read;
    fusionHal.write = fusion_hal_write;
    fusionHal.getTimeUs = fusion_hal_getTimeUs;

    ESP_ERROR_CHECK(sh2_open(&fusionHal, fusion_sensor_handler, NULL));

    sh2_SensorConfig_t config = {.reportInterval_us = 10000, // 100 Hz
                                 .changeSensitivityEnabled = false,
                                 .wakeupEnabled = false,
                                 .alwaysOnEnabled = true};

    ESP_ERROR_CHECK(sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config));
    ESP_ERROR_CHECK(sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config));
}

/* --- Task -------------------------------------------------------------- */
void fusion_task(void* param) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        sh2_service();
    }
}
