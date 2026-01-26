#include "driver/uart.h"
#include "esp_log.h"
#include "espnow_comm.h"

#define TAG "BRIDGE"
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

static void telemetry_task(void* pvParameters) {

    uint16_t telemetry[16];

    while (1) {
        // Check for telemetry from drone (ESP-NOW -> UART)

        if (xQueueReceive(wifiQueue, telemetry, portMAX_DELAY) == pdTRUE) {

            // Check if it's telemetry data (cmd_id = 3)
            if (telemetry[15] == 3) {
                // Forward to laptop via UART
                uart_write_bytes(UART_NUM, telemetry, 32);
            }
        }
    }
}

static void uart_task(void* pvParameters) {
    uint8_t data[BUF_SIZE];

    while (1) {
        // Read from UART (laptop)
        // 18 bytes -> 3 * uint16_t and 3 * float
        int len = uart_read_bytes(UART_NUM, data, 18, portMAX_DELAY);

        if (len == 18) {
            uint16_t* packet = (uint16_t*) data;
            uint16_t cmd_id = packet[0];
            // PID update packet
            if (cmd_id == 2) {
                ESP_LOGI(TAG, "PID Packet received");
                // Forward to drone
                if (xSemaphoreTake(wifiSendSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                    esp_err_t result = esp_now_send(drone_mac, (uint8_t*) packet, 32);
                    if (result != ESP_OK) {
                        ESP_LOGW(TAG, "PID update send failed");
                    }
                }
            }
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-S3 Bridge Starting...");

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uint8_t* macs[] = {drone_mac};
    // Initialize ESP-NOW (connect to drone)
    esp_now_module_init(macs, 1);

    ESP_LOGI(TAG, "Bridge initialized");

    esp_log_level_set("*", ESP_LOG_NONE);

    // Start Tasks
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
}