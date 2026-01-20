#include "driver/uart.h"
#include "esp_log.h"
#include "espnow_comm.h"

#define TAG "BRIDGE"
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

static void uart_task(void* pvParameters) {
    uint8_t data[BUF_SIZE];

    ESP_LOGI(TAG, "UART task started - waiting for PID packets from laptop");

    while (1) {
        // Read from UART (laptop)
        int len = uart_read_bytes(UART_NUM, data, 32, pdMS_TO_TICKS(100));

        if (len == 32) {
            uint16_t* packet = (uint16_t*) data;
            uint16_t cmd_id = packet[0];

            if (cmd_id == 2) { // PID update packet
                ESP_LOGI(TAG, "Received PID packet from laptop");

                // Forward to drone via ESP-NOW
                esp_err_t result = esp_send_packet(data, 32, NULL);

                if (result == ESP_OK) {
                    ESP_LOGI(TAG, "PID packet forwarded to drone");
                } else {
                    ESP_LOGE(TAG, "Failed to forward packet");
                }
            } else {
                ESP_LOGW(TAG, "Unknown command ID: %d", cmd_id);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
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

    // Initialize ESP-NOW (connect to drone)
    esp_now_module_init(drone_mac);

    ESP_LOGI(TAG, "Bridge initialized - ready to forward packets");

    // Start UART task
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}