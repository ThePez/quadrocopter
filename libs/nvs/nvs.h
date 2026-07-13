/*
 ******************************************************************************
 * File: nvs.h
 * Author: Jack Cairns
 * Date: 14-07-2026
 ******************************************************************************
 */

#ifndef NVS_H
#define NVS_H

#include <esp_err.h>

/**
 * @brief Initializes the NVS flash partition.
 *
 * Erases and reinitializes the partition if no free pages are available or
 * a newer NVS version is found. Safe to call more than once - subsequent
 * calls are a no-op once initialization has succeeded.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t nvs_init(void);

/**
 * @brief Reads a blob value from NVS.
 *
 * @param namespace NVS namespace to open.
 * @param key       Key to read within the namespace.
 * @param data      Destination buffer for the blob.
 * @param len       Expected size of the blob in bytes.
 *
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_INITIALIZED if nvs_init()
 *         hasn't been called, ESP_ERR_NVS_INVALID_LENGTH if the stored
 *         blob's size doesn't match len, or another error code on failure.
 */
esp_err_t nvs_read(const char* namespace, const char* key, void* data, size_t len);

/**
 * @brief Writes and commits a blob value to NVS.
 *
 * @param namespace NVS namespace to open.
 * @param key       Key to write within the namespace.
 * @param data      Source buffer holding the blob to store.
 * @param len       Size of the blob in bytes.
 *
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_INITIALIZED if nvs_init()
 *         hasn't been called, or another error code on failure.
 */
esp_err_t nvs_write(const char* namespace, const char* key, const void* data, size_t len);

#endif