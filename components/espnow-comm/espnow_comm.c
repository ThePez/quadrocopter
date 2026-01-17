/*
 ******************************************************************************
 * File: espnow_comm.c
 * Author: Jack Cairns
 * Date: 17-01-2026
 * Brief:
 * REFERENCE: None
 ******************************************************************************
 */

#include "espnow_comm.h"

#define TAG "ESPNOW_DRONE"

#define CHECK_ERR(code, msg)                                                                                           \
    do {                                                                                                               \
        esp_err_t err = (code);                                                                                        \
        if (err != ESP_OK) {                                                                                           \
            ESP_LOGE(TAG, msg);                                                                                        \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

#define CHECK_ERR_NO_LOG(code)                                                                                         \
    do {                                                                                                               \
        esp_err_t err = (code);                                                                                        \
        if (err != ESP_OK) {                                                                                           \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

// DRONE's MAC ADDRESS
const uint8_t drone_mac[6] = {0xD8, 0x13, 0x2A, 0x2C, 0x4D, 0x74};
// REMOTE's MAC ADDRESS
const uint8_t remote_mac[6] = {0x10, 0x06, 0x1C, 0xF2, 0x42, 0xA4};

static uint8_t peer_mac[6];

// Encryption keys
// PMK (Primary Master Key) - must be same on both devices
static uint8_t pmk_key[16] = {0x9F, 0x3C, 0x8A, 0x2D, 0x7B, 0x14, 0xE6, 0xF0, 0xA5, 0xC9, 0xD2, 0xB1};
// LMK (Local Master Key) - must be same for this peer relationship
static uint8_t lmk_key[16] = {0x4A, 0x7E, 0x1D, 0x9C, 0x0F, 0x6B, 0x2A, 0x8E, 0x5D, 0x3C, 0x91, 0xB4};

// Queue for recieving packets
QueueHandle_t wifiQueue = NULL;

static void espnow_send_cb(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "Send failed");
        // ESP_LOGI(TAG, "Send success");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {
    uint8_t packet[32];
    if (len != sizeof(packet)) {
        return;
    }

    memcpy(&packet, data, sizeof(packet));
    if (xQueueSendToBack(wifiQueue, packet, pdMS_TO_TICKS(10)) == pdTRUE) {
        ESP_LOGI(TAG, "Packet recieved and posted");
    }
}

static esp_err_t wifi_init(void) {
    CHECK_ERR(esp_netif_init(), "netif init failed");
    CHECK_ERR(esp_event_loop_create_default(), "event loop failed");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    CHECK_ERR(esp_wifi_init(&cfg), "wifi init failed");
    CHECK_ERR(esp_wifi_set_storage(WIFI_STORAGE_RAM), "storage init failed");
    CHECK_ERR(esp_wifi_set_mode(WIFI_MODE_STA), "mode set failed");
    CHECK_ERR(esp_wifi_start(), "wifi not started");

    // Print MAC address
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Wifi initialised");
    return ESP_OK;
}

static esp_err_t espnow_init(const uint8_t peer_addr[6]) {
    // Initialize ESP-NOW
    CHECK_ERR(esp_now_init(), "init failed");

    // Set PMK after initialization
    CHECK_ERR(esp_now_set_pmk(pmk_key), "PMK failed");

    // Register the callback functions
    CHECK_ERR(esp_now_register_send_cb(espnow_send_cb), "send cb register failed");
    CHECK_ERR(esp_now_register_recv_cb(espnow_recv_cb), "recieve cb register failed");

    // Add peer with encryption enabled
    esp_now_peer_info_t peer_info = {
        .channel = 0,         // Channel
        .ifidx = WIFI_IF_STA, // WIFI interface
        .encrypt = true       // Encryption enabled
    };

    memcpy(peer_info.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
    memcpy(peer_info.lmk, lmk_key, 16);

    CHECK_ERR(esp_now_add_peer(&peer_info), "peer add failed");

    ESP_LOGI(TAG, "ESP-NOW initialized with encryption, peer added");
    return ESP_OK;
}

esp_err_t esp_now_module_init(const uint8_t peer_addr[6]) {

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        CHECK_ERR_NO_LOG(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    CHECK_ERR(ret, "nvs flash code failed");

    // Initialize WiFi and ESP-NOW
    wifi_init();
    espnow_init(peer_addr);
    memcpy(peer_mac, peer_addr, ESP_NOW_ETH_ALEN);

    while (!wifiQueue) {
        wifiQueue = xQueueCreate(5, sizeof(uint8_t) * 32);
    }

    return ESP_OK;
}

esp_err_t esp_send_packet(void* packet, const uint8_t len) {

    uint8_t* data = (uint8_t*) packet;
    CHECK_ERR(esp_now_send(peer_mac, data, len), "Send error");
    return ESP_OK;
}
