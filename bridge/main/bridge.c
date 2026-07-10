/*
 *****************************************************************************
 * File: bridge.c
 * Author: Jack Cairns
 * Date: 08-07-2026
 *****************************************************************************
 */

#include "espnow_comm.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_rom_crc.h>

#define TAG "BRIDGE"
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

// READ States
#define MAGIC_0_REC 0
#define MAGIC_1_REC 1
#define PAYLOAD_REC 2
#define CRC_REC 3

#define UART_MAGIC_0 0xAA
#define UART_MAGIC_1 0x55

struct __packed uart_packet {
    uint8_t magic[2];
    struct sensor_telemetry_t payload;
    uint16_t crc;
};

static void uart_comm_write_frame(uart_port_t port, struct sensor_telemetry_t* payload) {
    struct uart_packet packet = {.magic = {UART_MAGIC_0, UART_MAGIC_1}};
    packet.payload = *payload;
    packet.crc = esp_rom_crc16_le(0, (uint8_t*) payload, sizeof(struct sensor_telemetry_t));
    uart_write_bytes(port, &packet, sizeof(struct uart_packet));
}

static void uart_comm_read_frame(uart_port_t port, struct pid_config_telemetry_t* payload,
                                 TickType_t byte_timeout) {
    uint8_t byte = 0;
    uint8_t state = 0;
    uint16_t crc;
    int len = 0;

    // State machine for where the read is at
    while (1) {

        switch (state) {
        case MAGIC_0_REC:
            if (uart_read_bytes(port, &byte, 1, portMAX_DELAY) != 1) {
                continue;
            }

            state = (byte == UART_MAGIC_0) ? MAGIC_1_REC : MAGIC_0_REC;
            continue;

        case MAGIC_1_REC:
            if (uart_read_bytes(port, &byte, 1, portMAX_DELAY) != 1) {
                continue;
            }

            state = (byte == UART_MAGIC_1) ? PAYLOAD_REC : MAGIC_0_REC;
            continue;

        case PAYLOAD_REC:
            len = uart_read_bytes(port, (uint8_t*) payload, sizeof(struct pid_config_telemetry_t),
                                  byte_timeout);
            state = (len == sizeof(struct pid_config_telemetry_t)) ? CRC_REC : MAGIC_0_REC;
            continue;

        case CRC_REC:
            len = uart_read_bytes(port, &crc, sizeof(uint16_t), byte_timeout);
            if (len != sizeof(uint16_t) ||
                crc != esp_rom_crc16_le(0, (uint8_t*) payload,
                                        sizeof(struct pid_config_telemetry_t))) {
                state = MAGIC_0_REC;
                continue;
            }
            return;
        }
    }
}

static void telemetry_task(void* pvParameters) {

    struct wifi_packet_t telemetry;

    while (1) {
        // Check for telemetry from drone (ESP-NOW -> UART)
        if (xQueueReceive(wifiQueue, &telemetry, portMAX_DELAY) == pdTRUE) {
            if (telemetry.packet_id == SENSOR) {
                // Forward to laptop via UART
                uart_comm_write_frame(UART_NUM, &telemetry.data.sensor);
            }
        }
    }
}

static void uart_task(void* pvParameters) {

    struct wifi_packet_t packet = {.packet_id = PID_CONFIG};
    struct pid_config_telemetry_t* pid_config = &packet.data.pid_config;

    while (1) {
        // Read from UART (laptop): kp, ki, kd (float) then axis, mode (uint16_t)
        uart_comm_read_frame(UART_NUM, pid_config, pdMS_TO_TICKS(100));

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