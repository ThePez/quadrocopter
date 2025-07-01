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
#include "freertos/semphr.h"
#include "freertos/task.h"
// Custom Components
#include "nrf24l01plus.h"

// Defines //

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

// Task Function Prototypes
void system_task(void);
void joystick_task(void);
void radio_task(void);

// Setup Function Prototypes
void spi_bus_setup(spi_host_device_t host);
void print_task_stats(void);

// Global Variables
TaskHandle_t systemTask = NULL;
TaskHandle_t joystickTask = NULL;
TaskHandle_t radioTask = NULL;

//////////////////////////////////////////////////////////////////////////////

void app_main(void) {

    xTaskCreate((void*) &system_task, "SYS_CONTROL", SYS_STACK, NULL, SYS_PRIO, &systemTask);
    xTaskCreate((void*) &joystick_task, "JOYSTICK", JOYSTICK_STACK, NULL, JOYSTICK_PRIO, &joystickTask);
    xTaskCreate((void*) &radio_task, "RADIO", RADIO_STACK, NULL, RADIO_PRIO, &radioTask);
    while (1) {
        // print_task_stats();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void system_task(void) {

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void joystick_task(void) {

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void radio_task(void) {

    spi_bus_setup(HSPI_HOST);
    nrf24l01plus_init(HSPI_HOST);

    uint8_t value = nrf24l01plus_read_register(NRF24L01PLUS_CONFIG);
    printf("Radio CONFIG register: %x\r\n", value);
    uint8_t data[32];
    memset(data, 0, sizeof(data));
    const char* word = "Hello World!";
    for (int i = 0; i < strlen(word); i++) {
        data[i] = word[i];
    }
    
    printf("Sending: ");
    for (int i = 0; i < 32; i++) {
        printf("%02x ", data[i]);
    }
    printf("\r\n");

    while (1) { 
        nrf24l01plus_send_packet(data);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void spi_bus_setup(spi_host_device_t host) {

    spi_bus_config_t bus_config = {
        .miso_io_num = HSPI_MISO,
        .mosi_io_num = HSPI_MOSI,
        .sclk_io_num = HSPI_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO));
}

void print_task_stats(void) {

    uint16_t numTasks = uxTaskGetNumberOfTasks();
    uint16_t bufferLength = numTasks * 50;
    char* taskListBuffer = pvPortMalloc(bufferLength * sizeof(char));
    if (taskListBuffer == NULL) {
        return; // Malloc Failed
    }

    vTaskList(taskListBuffer);
    printf("\r\nTask          State  Priority   Stack\tID\r\n");
    printf("=============================================\r\n");
    printf("%s\r\n", taskListBuffer);
    // Free memory
    vPortFree(taskListBuffer);
}

/****************** Not Used At this stage **************
xTaskCreate((void*) &adc_callback_task, "Callback Task", 4096, NULL, 0, &adc_callback_handle);

adc_channel_t channels[2] = {ADC1_CHANNEL_0, ADC1_CHANNEL_3};
adc_cali_handle_t adc_calis[2] = {NULL};
adc_continuous_handle_t adc_handle = NULL;
TaskHandle_t adc_callback_handle = NULL;
int adc_data[2] = {0};
int voltage[2] = {0};

void adc_callback_task(void* pvArgs) {
    uint8_t buffer[30];
    uint32_t rx_length = 0;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for the DMA to have polled enough ADC conversions
        adc_continuous_read(adc_handle, buffer, 8, &rx_length, 0); // Read out the data
        for (int i = 0; i < rx_length; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t* outputs = (adc_digi_output_data_t*) &buffer[i];
            uint16_t channel = outputs->type1.channel;
            uint16_t data = outputs->type1.data;
            if (channel == ADC1_CHANNEL_0) {
                adc_data[0] = data;
            } else if (channel == ADC1_CHANNEL_3) {
                adc_data[1] = data;
            }
        }
    }
}

static bool IRAM_ATTR adc_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t* edata,
                                   void* user_data) {
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(adc_callback_handle, &mustYield);
    return (mustYield == pdTRUE);
}

void adc_init(adc_channel_t* channels, uint8_t numberOfChannels) {
    // handle configuration
    adc_continuous_handle_cfg_t handle_config = {
        .conv_frame_size = 4,    // Bytes per conversion frame
        .max_store_buf_size = 8, // Max bytes that can be stored
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &adc_handle));

    adc_digi_pattern_config_t channel_config[2];
    for (int i = 0; i < numberOfChannels; i++) {
        channel_config[i].channel = channels[i];       // The channels to be used
        channel_config[i].atten = ADC_ATTEN_DB_12;     // 150mV to 2450mV
        channel_config[i].bit_width = ADC_BITWIDTH_12; // 12bit adc -> 0 to 4095
        channel_config[i].unit = ADC_UNIT_1;           // which adc driver is being used, 1 or 2.
    }

    // ADC Configuration with Channels
    adc_continuous_config_t adc_config = {
        .adc_pattern = channel_config,          // Config for each channel used
        .pattern_num = numberOfChannels,        // Number of adc channel to be used
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,    // What type of mode the DMA works in
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1, // Type of output data (data & channel for 1)
        .sample_freq_hz = 40 * 1000,            // 40kHz
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));

    // Callback Configuration
    adc_continuous_evt_cbs_t cb_config = {.on_conv_done = adc_callback};
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cb_config, NULL));

    // Calibration Init
    for (int i = 0; i < numberOfChannels; i++) {
        adc_calibration_init(ADC_UNIT_1, channels[i], ADC_ATTEN_DB_12, &adc_calis[i]);
    }
}

*/
