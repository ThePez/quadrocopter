/*
 ******************************************************************************
 * File: espnow_comm.c
 * Author: Jack Cairns
 * Date: 17-01-2026
 ******************************************************************************
 */

#include "espnow_comm.h"

#include "common_functions.h"
#include "nvs.h"

#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_now.h>
#include <esp_rom_crc.h>
#include <esp_wifi.h>
#include <string.h>

#define TAG "ESPNOW"

// DRONE's MAC ADDRESS
uint8_t drone_mac[6] = {0xD8, 0x13, 0x2A, 0x2C, 0x4D, 0x74};
// REMOTE's MAC ADDRESS
uint8_t remote_mac[6] = {0x10, 0x06, 0x1C, 0xF2, 0x42, 0xA4};
// Bridge MAC ADDRESS
uint8_t bridge_mac[6] = {0x58, 0x8C, 0x81, 0xCA, 0x5F, 0x80};

// Encryption keys
// PMK (Primary Master Key) - must be same on both devices
const uint8_t pmk_key[16] = {0x9F, 0x3C, 0x8A, 0x2D, 0x7B, 0x14,
                             0xE6, 0xF0, 0xA5, 0xC9, 0xD2, 0xB1};
// LMK (Local Master Key) - must be same for this peer relationship
const uint8_t lmk_key[16] = {0x4A, 0x7E, 0x1D, 0x9C, 0x0F, 0x6B,
                             0x2A, 0x8E, 0x5D, 0x3C, 0x91, 0xB4};

// Queue for recieving packets
QueueHandle_t wifiQueue = NULL;
SemaphoreHandle_t wifiSendSemaphore = NULL;

// ESP-NOW send-complete callback: releases wifiSendSemaphore so the next
// send can proceed, and warns if delivery failed.
static void espnow_send_cb(const uint8_t* mac_addr, esp_now_send_status_t status) {
    ARG_UNUSED(mac_addr);
    xSemaphoreGive(wifiSendSemaphore);
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "MESSAGE NOT DELIVERED");
    }
}

// ESP-NOW receive callback: validates the packet's CRC16 and, if valid,
// posts it to wifiQueue for processing.
static void espnow_recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {

    ARG_UNUSED(recv_info);
    ARG_UNUSED(len);

    struct wifi_packet_t* packet = (struct wifi_packet_t*) data;

    // Check CRC16 value
    if (packet->crc16 != esp_rom_crc16_le(0, (uint8_t*) &packet->data, sizeof(union packet_data))) {
        // Invalid Packet
        ESP_LOGW(TAG, "Invalid wifi packet recieved");
        return;
    }

    // No waiting
    if (xQueueSendToBack(wifiQueue, packet, 0) != pdTRUE) {
        ESP_LOGI(TAG, "Packet recieved but not posted");
    }
}

// Brings up WiFi in station mode on a fixed channel with power-save
// disabled, as required for reliable ESP-NOW.
static esp_err_t wifi_init(void) {
    CHECK_ERR(esp_netif_init(), "netif init failed");
    CHECK_ERR(esp_event_loop_create_default(), "event loop failed");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    CHECK_ERR(esp_wifi_init(&cfg), "wifi init failed");
    CHECK_ERR(esp_wifi_set_storage(WIFI_STORAGE_RAM), "storage init failed");
    CHECK_ERR(esp_wifi_set_mode(WIFI_MODE_STA), "mode set failed");

    CHECK_ERR(esp_wifi_start(), "wifi not started");

    // Disable modem sleep - power save delays/drops ESP-NOW RX between DTIM wakeups
    CHECK_ERR(esp_wifi_set_ps(WIFI_PS_NONE), "Wifi power save disable failed");

    CHECK_ERR(esp_wifi_set_channel(9, WIFI_SECOND_CHAN_NONE), "Wifi channel not set");

    ESP_LOGI(TAG, "Wifi initialised");

    return ESP_OK;
}

// Initializes the ESP-NOW driver, sets the PMK, registers the send/receive
// callbacks, and adds each given MAC as an encrypted peer.
static esp_err_t espnow_init(uint8_t* peer_addr[], uint8_t num_peers) {
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

    // Copy in the LMK
    memcpy(peer_info.lmk, lmk_key, 16);

    // Add all peers
    for (uint8_t i = 0; i < num_peers; i++) {
        // Copy in the mac address
        memcpy(peer_info.peer_addr, peer_addr[i], ESP_NOW_ETH_ALEN);
        CHECK_ERR(esp_now_add_peer(&peer_info), "peer add failed");
    }

    ESP_LOGI(TAG, "ESP-NOW initialized with encryption, peer added");
    return ESP_OK;
}

esp_err_t esp_now_module_init(uint8_t* peer_addr[], uint8_t num_peers) {

    // Ensure NVS is initialised
    CHECK_ERR_NO_LOG(nvs_init());

    // Initialise WiFi and ESP-NOW
    wifi_init();
    espnow_init(peer_addr, num_peers);

    // Now initialise the messaging primatives
    while (!wifiQueue) {
        wifiQueue = xQueueCreate(10, sizeof(struct wifi_packet_t));
    }

    wifiSendSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(wifiSendSemaphore);

    return ESP_OK;
}

esp_err_t esp_now_send_packet(const uint8_t* addr, struct wifi_packet_t* packet) {
    // Add the CRC before sending
    packet->crc16 = esp_rom_crc16_le(0, (uint8_t*) &packet->data, sizeof(union packet_data));
    return esp_now_send(addr, (uint8_t*) packet, sizeof(struct wifi_packet_t));
}
