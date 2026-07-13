/*
 ******************************************************************************
 * File: nvs.c
 * Author: Jack Cairns
 * Date: 14-07-2026
 ******************************************************************************
 */

#include "nvs.h"

#include "common_functions.h"

#include <nvs_flash.h>
#include <stdbool.h>

#define TAG "NVS"

static bool initialised = false;

esp_err_t nvs_init(void) {

    if (initialised) {
        return ESP_OK;
    }

    // Initialize NVS flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        CHECK_ERR_NO_LOG(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    CHECK_ERR(ret, "Flash initialise failed");

    // NVS init success
    initialised = true;
    return ESP_OK;
}

esp_err_t nvs_read(const char* namespace, const char* key, void* data, size_t len) {
    if (!initialised) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }

    nvs_handle_t handle;
    CHECK_ERR(nvs_open(namespace, NVS_READONLY, &handle), "open(%s) failed", namespace);
    size_t actual = len;
    CHECK_ERR(nvs_get_blob(handle, key, data, &actual), "read(%s/%s) failed", namespace, key);

    // Ensure length is expected
    if (actual != len) {
        ESP_LOGW(TAG, "Size mismatch: expected %d, got %d", (int) len, (int) actual);
        return ESP_ERR_NVS_INVALID_LENGTH;
    }

    return ESP_OK;
}

esp_err_t nvs_write(const char* namespace, const char* key, const void* data, size_t len) {
    if (!initialised) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }

    nvs_handle_t handle;
    CHECK_ERR(nvs_open(namespace, NVS_READWRITE, &handle), "open(%s) failed", namespace);

    esp_err_t err = nvs_set_blob(handle, key, data, len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set failed (%s/%s)", namespace, key);
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "commit failed (%s/%s)", namespace, key);
    }

    nvs_close(handle);
    return err;
}
