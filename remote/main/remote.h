/*
 *****************************************************************************
 * File: remote.h
 * Author: Jack Cairns
 * Date: 23-07-2025
 *****************************************************************************
 */

#ifndef REMOTE_H
#define REMOTE_H

#include <esp_err.h>

#define MODE_BUTTON_PIN 25
#define SHUTOFF_BUTTON_PIN 32

esp_err_t init_remote(void);

#endif
