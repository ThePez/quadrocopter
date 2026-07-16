/*
 ******************************************************************************
 * File: remote.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 ******************************************************************************
 */

#include "remote.h"

#include "common_functions.h"
#include "controller_task.h"
#include "device_config.h"
#include "espnow_comm.h"
#include "mcp3208.h"
#include "remote_defaults.h"
#include "rx_task.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <string.h>

struct button_state_t {
    uint64_t prevFalling;
    uint64_t prevRising;
    gpio_num_t pin;
    uint8_t pushAllowed;
    uint32_t notifyBit;
};

#define TAG "REMOTE"

#define DEBOUNCE_US 50000

////////////////////////////// Global Variables //////////////////////////////

// Button states
static struct button_state_t mode_button = {
    .pin = MODE_BUTTON_PIN, .pushAllowed = 1, .notifyBit = MODE_BUTTON_BIT};
static struct button_state_t emergency_button = {
    .pin = SHUTOFF_BUTTON_PIN, .pushAllowed = 1, .notifyBit = EMERGENCY_BUTTON_BIT};

// Live remote config
struct nvs_remote_cfg_t remoteCfg;

// Default remote config
static struct nvs_remote_cfg_t remoteDefaults = {
    .version = NVS_REMOTE_CFG_VERSION,
    .voltage_cal_multiplier = VOLTAGE_CAL_MULTIPLIER,
    .low_voltage = LOW_VOLTAGE,
    .critical_voltage = CRITICAL_VOLTAGE,
    .throttle = {.min = JOYSTICK_CAL_MIN, .centre = JOYSTICK_CAL_CENTRE, .max = JOYSTICK_CAL_MAX},
    .pitch = {.min = JOYSTICK_CAL_MIN, .centre = JOYSTICK_CAL_CENTRE, .max = JOYSTICK_CAL_MAX},
    .roll = {.min = JOYSTICK_CAL_MIN, .centre = JOYSTICK_CAL_CENTRE, .max = JOYSTICK_CAL_MAX},
    .yaw = {.min = JOYSTICK_CAL_MIN, .centre = JOYSTICK_CAL_CENTRE, .max = JOYSTICK_CAL_MAX}};

//////////////////////////////////////////////////////////////////////////////

// ISR: Callback for button interrupts
static void IRAM_ATTR intr_handler(void* args) {

    struct button_state_t* button = (struct button_state_t*) args;
    int state = gpio_get_level(button->pin);
    uint64_t currentTick = esp_timer_get_time();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (!state && button->pushAllowed && (currentTick - button->prevRising >= DEBOUNCE_US)) {
        // Falling edge
        button->prevFalling = currentTick;
        button->pushAllowed = 0;
        xTaskNotifyFromISR(remoteTaskHandle, button->notifyBit, eSetBits,
                           &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else if (state && (currentTick - button->prevFalling >= DEBOUNCE_US)) {
        // Rising edge
        button->prevRising = currentTick;
        button->pushAllowed = 1;
    }
}

// ISR: Callback for signalling remote task to compelete one loop
static void IRAM_ATTR remote_callback(void* args) {
    ARG_UNUSED(args);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (remoteTaskHandle) {
        xTaskNotifyFromISR(remoteTaskHandle, TIMER_BIT, eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Configures a button's GPIO as an interrupt input and installs the given ISR handler.
static esp_err_t interrupt_init(void* handler, struct button_state_t* button) {

    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT,               // Direction of the pin
        .pin_bit_mask = (1ULL << button->pin), // Pin is listed here as a bit mask
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Enable internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable internal pull-down
        .intr_type = GPIO_INTR_ANYEDGE         // Interrupt on both edges, switch is active low
    };

    // Configure the GPIO-pin
    CHECK_ERR(gpio_config(&config), "GPIO config failed");
    CHECK_ERR(gpio_intr_disable(button->pin), "Interrupt disable failed");

    // Install the ISR service if not already installed
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ISR install service failed");
        return err;
    }

    // Hook ISR handler for the IQR pin
    CHECK_ERR(gpio_isr_handler_add(button->pin, handler, button), "ISR handler add failed");
    return ESP_OK;
}

// Initialises a timer callback for signaling a periodic task action.
static esp_err_t remote_timer_init(void) {
    const esp_timer_create_args_t args = {.callback = remote_callback,
                                          .dispatch_method = ESP_TIMER_ISR};
    esp_timer_handle_t remote_timer = NULL;
    CHECK_ERR(esp_timer_create(&args, &remote_timer), "Timer create failed");
    CHECK_ERR(esp_timer_start_periodic(remote_timer, 50000), "Timer start failed");
    ESP_LOGI(TAG, "Remote Timer Initialised");
    return ESP_OK;
}

// Converts a wire-format joystick calibration into the NVS-stored shape
static struct nvs_joystick_cal_t to_nvs_cal(struct joystick_cal_telemetry_t cal) {
    return (struct nvs_joystick_cal_t){.min = cal.min, .centre = cal.centre, .max = cal.max};
}

esp_err_t remote_config_handle_update(struct remote_config_telemetry_t* packet) {
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    remoteCfg.voltage_cal_multiplier = packet->voltage_cal_multiplier;
    remoteCfg.low_voltage = packet->low_voltage;
    remoteCfg.critical_voltage = packet->critical_voltage;
    remoteCfg.throttle = to_nvs_cal(packet->throttle);
    remoteCfg.pitch = to_nvs_cal(packet->pitch);
    remoteCfg.roll = to_nvs_cal(packet->roll);
    remoteCfg.yaw = to_nvs_cal(packet->yaw);
    xSemaphoreGive(cfgMutex);

    ESP_LOGI(TAG, "Remote config updated live (not yet saved to flash)");
    return ESP_OK;
}

esp_err_t remote_config_handle_save(void) {
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    struct nvs_remote_cfg_t snapshot = remoteCfg;
    xSemaphoreGive(cfgMutex);

    CHECK_ERR(device_config_save("remoteCfg", &snapshot, sizeof(struct nvs_remote_cfg_t)),
              "Failed to save remote config to flash");
    ESP_LOGI(TAG, "Remote config saved to flash");
    return ESP_OK;
}

// Logs the currently loaded remoteCfg.
static void print_config(void) {
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    struct nvs_remote_cfg_t cfg = remoteCfg;
    xSemaphoreGive(cfgMutex);

    ESP_LOGI(TAG, "==== Remote Config (v%u) ====", cfg.version);
    ESP_LOGI(TAG, "== Battery ==");
    ESP_LOGI(TAG, "Voltage cal multiplier: %.4f", cfg.voltage_cal_multiplier);
    ESP_LOGI(TAG, "Battery: low=%u mV critical=%u mV", cfg.low_voltage, cfg.critical_voltage);
    ESP_LOGI(TAG, "== Joystick calibration ==");
    ESP_LOGI(TAG, "Throttle: min=%u centre=%u max=%u", cfg.throttle.min, cfg.throttle.centre,
             cfg.throttle.max);
    ESP_LOGI(TAG, "Pitch:    min=%u centre=%u max=%u", cfg.pitch.min, cfg.pitch.centre,
             cfg.pitch.max);
    ESP_LOGI(TAG, "Roll:     min=%u centre=%u max=%u", cfg.roll.min, cfg.roll.centre, cfg.roll.max);
    ESP_LOGI(TAG, "Yaw:      min=%u centre=%u max=%u", cfg.yaw.min, cfg.yaw.centre, cfg.yaw.max);
}

esp_err_t init_remote(void) {
    CHECK_ERR(nvs_init(), "NVS init failed");

    esp_err_t err = device_config_load("remoteCfg", &remoteCfg, sizeof(struct nvs_remote_cfg_t),
                                       NVS_REMOTE_CFG_VERSION, &remoteDefaults);
    if (err == ESP_ERR_NVS_TYPE_MISMATCH) {
        // Either first boot or struct update has occured
        remoteCfg = remoteDefaults;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to load remote config from flash");
        return err;
    }

    // Display loaded config
    print_config();

    const uint8_t* macs[] = {drone_mac, bridge_mac};
    CHECK_ERR_NO_LOG(esp_now_module_init(macs, 2));

    CHECK_ERR_NO_LOG(interrupt_init(intr_handler, &mode_button));
    CHECK_ERR_NO_LOG(interrupt_init(intr_handler, &emergency_button));
    CHECK_ERR(spi_bus_setup(VSPI_HOST), "SPI bus failed init");
    // Remote MCP3208 cs pin is 25
    CHECK_ERR(mcpx_task_init(&spiVMutex, 0x1F, VSPI_HOST, MCPx_CS_PIN_REMOTE),
              "MCPx Task creation failed");

    // Create the 2 remote tasks
    CHECK_ERR(rx_task_init(), "Failed to create the RX task");
    CHECK_ERR_NO_LOG(controller_task_init());
    CHECK_ERR_NO_LOG(remote_timer_init());
    return ESP_OK;
}
