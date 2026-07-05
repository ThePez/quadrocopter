#include "espnow_comm.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_rom_crc.h>

#define TAG "BRIDGE"
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

static void telemetry_task(void* pvParameters) {

    struct wifi_packet_t telemetry;

    while (1) {
        // Check for telemetry from drone (ESP-NOW -> UART)
        if (xQueueReceive(wifiQueue, &telemetry, portMAX_DELAY) == pdTRUE) {

            uint16_t expected =
                esp_rom_crc16_le(0, (uint8_t*) &(telemetry.data), sizeof(union packet_data));
            if (expected != telemetry.crc16) {
                // corrupted packet
                continue;
            }

            if (telemetry.packet_id == SENSOR) {
                // Forward to laptop via UART
                uart_write_bytes(UART_NUM, &telemetry.data.sensor,
                                 sizeof(struct sensor_telemetry_t));
            }
        }
    }
}

static void uart_task(void* pvParameters) {

    struct wifi_packet_t packet = {.packet_id = PID_CONFIG};
    struct pid_config_telemetry_t* pid_config = &packet.data.pid_config;

    while (1) {
        // Read from UART (laptop): kp, ki, kd (float) then axis, mode (uint16_t)
        int len = uart_read_bytes(UART_NUM, (uint8_t*) pid_config,
                                  sizeof(struct pid_config_telemetry_t), portMAX_DELAY);

        if (len == sizeof(struct pid_config_telemetry_t)) {
            ESP_LOGI(TAG, "PID Packet received");

            packet.crc16 = esp_rom_crc16_le(0, (uint8_t*) &packet.data, sizeof(union packet_data));

            // Forward to drone
            if (xSemaphoreTake(wifiSendSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                esp_err_t result =
                    esp_now_send(drone_mac, (uint8_t*) &packet, sizeof(struct wifi_packet_t));
                if (result != ESP_OK) {
                    ESP_LOGW(TAG, "PID update send failed");
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