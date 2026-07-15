/*
 ******************************************************************************
 * File: remote.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 ******************************************************************************
 */

#include "remote.h"

#include "common_functions.h"
#include "device_config.h"
#include "espnow_comm.h"
#include "mcp3208.h"
#include "remote_defaults.h"

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

struct remote_state_t {
    enum flight_mode_t flightMode;
    uint8_t armed;
    uint8_t emergency;
};

#define REMOTE_STACK (configMINIMAL_STACK_SIZE * 2)
#define REMOTE_PRIO (tskIDLE_PRIORITY + 2)
#define REMOTE_DELAY (pdMS_TO_TICKS(50))

#define TAG "REMOTE"

#define STATUS_LED 2
#define DEBOUNCE_US 50000

#define ADC_MIN 0
#define ADC_MAX 4095

#define MODE_BUTTON_BIT (1UL << 0)
#define EMERGENCY_BUTTON_BIT (1UL << 1)
#define TIMER_BIT (1UL << 31)

////////////////////////////// Global Variables //////////////////////////////

TaskHandle_t remoteTaskHandle = NULL;

// Button states
struct button_state_t mode_button = {
    .pin = MODE_BUTTON_PIN, .pushAllowed = 1, .notifyBit = MODE_BUTTON_BIT};
struct button_state_t emergency_button = {
    .pin = SHUTOFF_BUTTON_PIN, .pushAllowed = 1, .notifyBit = EMERGENCY_BUTTON_BIT};

struct nvs_remote_cfg_t remoteCfg;

// Default remote config - identity joystick calibration and a neutral
// voltage multiplier, since no calibration or battery hardware exists yet
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
void IRAM_ATTR remote_callback(void* args) {
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

// Remaps a raw ADC reading through its per-channel calibration back into the
// nominal ADC_MIN..ADC_MAX range, split around `centre` so an off-centre rest
// position doesn't skew the low and high halves of travel.
static uint16_t apply_joystick_cal(uint16_t raw, struct nvs_joystick_cal_t cal) {
    double adcMid = (ADC_MIN + ADC_MAX) / 2.0;
    double corrected = (raw < cal.centre) ? mapf(raw, cal.min, cal.centre, ADC_MIN, adcMid)
                                          : mapf(raw, cal.centre, cal.max, adcMid, ADC_MAX);
    return (uint16_t) constrainf(corrected, ADC_MIN, ADC_MAX);
}

// Periodic tick: triggers an ADC read, handles arm/re-arm logic from the
// joystick-press gesture, and sends the resulting REMOTE packet to the drone.
static void handle_timer_tick(struct remote_state_t* state, uint32_t notifyValue,
                              struct wifi_packet_t* packet) {
    // Notify the MCPx task to perform an ADC reading, then wait for the reply
    xTaskNotify(mcpxTaskHandle, 0, eNoAction);
    uint16_t adcValues[MCP3208_MAX_CHANNELS];
    if (xQueueReceive(mcpxQueue, adcValues, REMOTE_DELAY) != pdTRUE) {
        return;
    }

    // Correct each channel's raw reading for its calibrated centre/travel
    // before anything downstream (arming check, packet fill) uses it
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    struct nvs_joystick_cal_t throttleCal = remoteCfg.throttle;
    struct nvs_joystick_cal_t pitchCal = remoteCfg.pitch;
    struct nvs_joystick_cal_t rollCal = remoteCfg.roll;
    struct nvs_joystick_cal_t yawCal = remoteCfg.yaw;
    xSemaphoreGive(cfgMutex);

    adcValues[0] = apply_joystick_cal(adcValues[0], rollCal);
    adcValues[1] = apply_joystick_cal(adcValues[1], pitchCal);
    adcValues[2] = apply_joystick_cal(adcValues[2], throttleCal);
    adcValues[3] = apply_joystick_cal(adcValues[3], yawCal);

    // Check if now armed
    if (!state->armed && (notifyValue & MODE_BUTTON_BIT) && (notifyValue & EMERGENCY_BUTTON_BIT)) {
        if (!state->emergency) {
            state->armed = 1;
            ESP_LOGW(TAG, "REMOTE ARMED - communication is now enabled.");
        } else if (adcValues[2] < 50) {
            state->emergency = 0;
            state->armed = 1;
            ESP_LOGW(TAG, "REMOTE REARMED - communication is now enabled.");
        }

        if (state->armed) {
            // Consume the ARMING bits now that arming has happened
            xTaskNotifyWait(MODE_BUTTON_BIT | EMERGENCY_BUTTON_BIT, 0, NULL, 0);
        }
    }

    struct remote_telemetry_t* wifiData = &packet->data.remote;
    wifiData->flight_mode = state->flightMode;
    wifiData->throttle = (state->emergency) ? 0 : adcValues[2];
    wifiData->pitch = adcValues[1];
    wifiData->roll = adcValues[0];
    wifiData->yaw = adcValues[3];

    // Send the resulting packet to the drone
    if (state->armed) {
        esp_now_send_packet(drone_mac, packet);
    }

    if (state->emergency) {
        state->armed = 0;
    }
}

// Toggles flight mode (and the status LED) on a mode-button press, if armed.
static void handle_mode_button(struct remote_state_t* state) {
    if (!state->armed) {
        return;
    }

    state->flightMode = (state->flightMode != ACRO) ? ACRO : STABILISE;
    gpio_set_level(STATUS_LED, (state->flightMode == STABILISE) ? 1 : 0);
    ESP_LOGI(TAG, "Flight Mode %s", (state->flightMode == STABILISE) ? "Stabilise" : "Acro");

    // Consume the set MODE bit
    xTaskNotifyWait(MODE_BUTTON_BIT, 0, NULL, 0);
}

// Sets the emergency flag on an emergency-button press, if armed.
static void handle_emergency_button(struct remote_state_t* state) {
    if (!state->armed) {
        return;
    }

    state->emergency = 1;
    ESP_LOGE(TAG, "Emergancy");

    // Consume the set MODE & EMERGENCY bits
    xTaskNotifyWait(MODE_BUTTON_BIT | EMERGENCY_BUTTON_BIT, 0, NULL, 0);
}

// Converts a wire-format joystick calibration into the NVS-stored shape (same
// fields, different struct type).
static struct nvs_joystick_cal_t to_nvs_cal(struct joystick_cal_telemetry_t cal) {
    return (struct nvs_joystick_cal_t){.min = cal.min, .centre = cal.centre, .max = cal.max};
}

// Applies a REMOTE_CONFIG update (joystick calibration/battery constants)
// live. Writes straight into remoteCfg under cfgMutex; doesn't touch flash -
// a later REMOTE_CONFIG_SAVE is what persists the change.
static esp_err_t remote_config_handle_update(struct remote_config_telemetry_t* packet) {
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

// Persists the currently-live remoteCfg to flash. Snapshots remoteCfg under
// cfgMutex, then hands that snapshot to device_config_save() as the new
// flash contents.
static esp_err_t remote_config_handle_save(void) {
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    struct nvs_remote_cfg_t snapshot = remoteCfg;
    xSemaphoreGive(cfgMutex);

    CHECK_ERR(device_config_save("remoteCfg", &snapshot, sizeof(struct nvs_remote_cfg_t)),
              "Failed to save remote config to flash");
    return ESP_OK;
}

// Consumes incoming ESP-NOW config packets
static void cfg_task(void* pvParameters) {
    ARG_UNUSED(pvParameters);
    struct wifi_packet_t packet;

    while (1) {
        xQueueReceive(wifiQueue, &packet, portMAX_DELAY);
        switch (packet.packet_id) {
        case REMOTE_CFG:
            remote_config_handle_update(&packet.data.remote_config);
            break;

        case REMOTE_CFG_STORE:
            remote_config_handle_save();
            break;

        default:
            ESP_LOGW(TAG, "Unexpected packet id %d", packet.packet_id);
            break;
        }
    }
}

// Main remote task
static void remote_controller(void* pvParams) {
    (void) pvParams;
    struct wifi_packet_t packet = {.packet_id = REMOTE};

    // Idle until all queue's are created
    while (!mcpxQueue || !wifiQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    struct remote_state_t state = {.flightMode = ACRO};
    uint32_t notifyValue;

    // Initialize status LED
    gpio_config_t led_config = {.mode = GPIO_MODE_OUTPUT,
                                .pin_bit_mask = (1ULL << STATUS_LED),
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&led_config);
    gpio_set_level(STATUS_LED, 0); // Start with LED off for ACRO mode

    ESP_LOGW(TAG, "Waiting to be armed, press both joysticks.");

    while (1) {
        xTaskNotifyWait(0, TIMER_BIT, &notifyValue, portMAX_DELAY);
        if (notifyValue & MODE_BUTTON_BIT) {
            handle_mode_button(&state);
        }

        if (notifyValue & EMERGENCY_BUTTON_BIT) {
            handle_emergency_button(&state);
        }

        if (notifyValue & TIMER_BIT) {
            handle_timer_tick(&state, notifyValue, &packet);
        }
    }
}

esp_err_t init_remote(void) {
    const uint8_t* macs[] = {drone_mac, bridge_mac};
    CHECK_ERR_NO_LOG(esp_now_module_init(macs, 2));

    esp_err_t err = device_config_load("remoteCfg", &remoteCfg, sizeof(struct nvs_remote_cfg_t),
                                       NVS_REMOTE_CFG_VERSION, &remoteDefaults);
    if (err == ESP_ERR_NVS_TYPE_MISMATCH) {
        // Either first boot or struct update has occured
        remoteCfg = remoteDefaults;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to load remote config from flash");
        return err;
    }

    CHECK_ERR_NO_LOG(interrupt_init(intr_handler, &mode_button));
    CHECK_ERR_NO_LOG(interrupt_init(intr_handler, &emergency_button));
    CHECK_ERR(spi_bus_setup(VSPI_HOST), "SPI bus failed init");
    // Remote MCP3208 cs pin is 25
    CHECK_ERR(mcpx_task_init(&spiVMutex, 0x1F, VSPI_HOST, MCPx_CS_PIN_REMOTE),
              "MCPx Task creation failed");

    CHECK_ERR_NO_LOG(remote_timer_init());

    // Create the 2 remote tasks
    CHECK_ERR(xTaskCreate(cfg_task, "CFG_TASK", REMOTE_STACK, NULL, REMOTE_PRIO, NULL) == pdPASS
                  ? ESP_OK
                  : ESP_FAIL,
              "Failed to create the config task");
    CHECK_ERR_NO_LOG(xTaskCreate(remote_controller, "REMOTE_TASK", REMOTE_STACK, NULL, REMOTE_PRIO,
                                 &remoteTaskHandle) == pdPASS
                         ? ESP_OK
                         : ESP_FAIL);
    return ESP_OK;
}
