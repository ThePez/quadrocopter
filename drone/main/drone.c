/*
 *****************************************************************************
 * File: drone.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "drone.h"

#include "common_functions.h"
#include "espnow_comm.h"
#include "imu.h"
#include "input_task.h"
#include "kalman.h"
#include "motors.h"
#include "pid_task.h"

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define TAG "DRONE"

#define BATTERY_ADC_CHANNEL ADC_CHANNEL_7

////////////////////////////// Global Variables //////////////////////////////

struct drone_config_t droneData = {.mode = ACRO};
struct target_parameters_t remoteIn;

// ADC
static adc_oneshot_unit_handle_t adcHandle;
static adc_cali_handle_t adcCaliHandle;

//////////////////////////////////////////////////////////////////////////////

/* Setup Functions */

static esp_err_t adc_calibration_init(void) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    uint8_t calibrated = 0;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = BATTERY_ADC_CHANNEL,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = 1;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is \"Line Fitting\"");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };

        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = 1;
        }
    }
#endif

    adcCaliHandle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
        return ESP_OK;
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t adc_init(void) {

    adc_oneshot_unit_init_cfg_t unitConfig = {.unit_id = ADC_UNIT_1};
    CHECK_ERR(adc_oneshot_new_unit(&unitConfig, &adcHandle), "ADC new unit failed");

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    CHECK_ERR(adc_oneshot_config_channel(adcHandle, BATTERY_ADC_CHANNEL, &config),
              "ADC channel config failed");

    esp_err_t err = adc_calibration_init();
    if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
        return err;
    }

    return ESP_OK;
}

static void timer_task_callback_init(int periodUS, void (*cb)(void*)) {

    esp_timer_create_args_t config = {
        .callback = cb,                    // Function to execute
        .dispatch_method = ESP_TIMER_TASK, // Where the function is called from
        .arg = NULL                        // Input argument
    };

    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&config, &timerHandle);
    esp_timer_start_periodic(timerHandle, periodUS);
}

/* Functions run in the ESP-TIMER-TASK */

static void battery_callback(void* args) {
    ARG_UNUSED(args);

    int raw_value;
    esp_err_t ret = adc_oneshot_read(adcHandle, BATTERY_ADC_CHANNEL, &raw_value);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed");
        return;
    }

    float battery_voltage;

    if (adcCaliHandle != NULL) {
        // Use calibrated reading
        int calibrated_voltage;
        adc_cali_raw_to_voltage(adcCaliHandle, raw_value, &calibrated_voltage);
        battery_voltage = (float) calibrated_voltage * VOLTAGE_MULTIPLIER;
    } else {
        // Fallback to uncalibrated (0-3.3V range for 12dB attenuation)
        battery_voltage = (float) raw_value * NON_CALIBRATED_MULTIPLIER * VOLTAGE_MULTIPLIER;
    }

    // Store as millivolts
    xSemaphoreTake(droneData.mutex, portMAX_DELAY);
    droneData.battery = (uint16_t) battery_voltage; // mV
    xSemaphoreGive(droneData.mutex);

    // If battery volatage is too low -> send a signal to the remote
    if (battery_voltage < LOW_VOLTAGE) {
        struct wifi_packet_t packet = {.packet_id = POWER};
        packet.data.power.battery = battery_voltage;

        // Only allow sending after the previous message was sent
        if (xSemaphoreTake(wifiSendSemaphore, 0) == pdTRUE) {
            esp_err_t result = esp_now_send_packet(remote_mac, &packet);
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "Power callback send failed");
                xSemaphoreGive(wifiSendSemaphore);
            }
        } else {
            ESP_LOGW(TAG, "wifi semaphore unavailable");
        }
    }
}

static void system_data_callback(void* args) {
    ARG_UNUSED(args);

    struct imu_packet_t attitude;
    kalman_get(&attitude);

    struct wifi_packet_t packet = {.packet_id = SENSOR};
    struct sensor_telemetry_t* telemetry = &packet.data.sensor;

    telemetry->pitch_angle = attitude.pitchAngle;
    telemetry->roll_angle = attitude.rollAngle;
    telemetry->yaw_angle = attitude.yawAngle;
    telemetry->pitch_rate = attitude.pitchRate;
    telemetry->roll_rate = attitude.rollRate;
    telemetry->yaw_rate = attitude.yawRate;
    xSemaphoreTake(droneData.mutex, portMAX_DELAY);
    telemetry->mode = droneData.mode;
    telemetry->battery = droneData.battery;
    xSemaphoreGive(droneData.mutex);
    telemetry->pid_pitch = outputPID.pitch_pid;
    telemetry->pid_roll = outputPID.roll_pid;
    telemetry->pid_yaw = outputPID.yaw_pid;
    telemetry->motor_a = motors.motorA;
    telemetry->motor_b = motors.motorB;
    telemetry->motor_c = motors.motorC;
    telemetry->motor_d = motors.motorD;

    // Only allow sending after the previous message was sent
    if (xSemaphoreTake(wifiSendSemaphore, 0) == pdTRUE) {
        esp_err_t result = esp_now_send_packet(bridge_mac, &packet);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "data callback send failed");
            xSemaphoreGive(wifiSendSemaphore);
        }
    } else {
        ESP_LOGW(TAG, "wifi semaphore unavailable");
    }
}

esp_err_t init_drone(void) {

    // Setup PWM - motors off until the flight loop takes over
    CHECK_ERR_NO_LOG(esc_pwm_init(MOTOR_SPEED_MIN));

    // ADC for measuring the battery voltage
    CHECK_ERR_NO_LOG(adc_init());

    // Start the C++ imu task
    CHECK_ERR(imu_init(), "IMU init failed");
    // Wait for the IMU initialisation to finish
    vTaskDelay(pdMS_TO_TICKS(50));

    // Setup ESP-NOW (pair the bridge and remote)
    uint8_t* macs[] = {remote_mac, bridge_mac};
    CHECK_ERR_NO_LOG(esp_now_module_init(macs, 2));

    // Wait to ensure the wifiQueue is created before timer callback tasks are registered
    while (!wifiQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Mutex protecting droneData - must exist before task startup
    droneData.mutex = xSemaphoreCreateMutex();
    if (droneData.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create droneData mutex");
        return ESP_FAIL;
    }

    // Mutex protecting kalman's shared state - must be called before task startup
    CHECK_ERR(kalman_init(), "Failed to create kalman mutex");

    // Initialise a callback for returning info back to the PC app
    timer_task_callback_init(100000, system_data_callback); // Interval of 50ms

    // Initialise a callback for checking battery voltage
    timer_task_callback_init(1000000, battery_callback); // Interval of 1 second

    // Create the 2 control Tasks
    CHECK_ERR(input_task_init(), "Input task init failed");
    CHECK_ERR(pid_task_init(), "PID task init failed");

    return ESP_OK;
}

/* ESC programming mode - bypasses flight control entirely so the throttle
 * channel drives all 4 ESCs directly, allowing entry to the ESC's own
 * programming menu (which requires seeing max throttle on power-up). */

static void esc_program_task(void* pvParams) {
    ARG_UNUSED(pvParams);

    while (1) {
        xSemaphoreTake(droneData.mutex, portMAX_DELAY);
        int64_t lastRemoteTime = droneData.lastRemoteTime;
        xSemaphoreGive(droneData.mutex);

        // Leave the startup duty cycle untouched until the remote actually
        // connects - otherwise remoteIn.throttle's zeroed default would
        // immediately stomp the max-throttle signal the ESCs need to see.
        if (lastRemoteTime != 0) {
            uint16_t duty = (uint16_t) remoteIn.throttle;
            for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                esc_pwm_set_duty_cycle((MotorIndex) i, duty);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t init_esc_programming(void) {

    // Motors up at max throttle immediately - most ESCs enter their
    // programming menu by seeing max throttle as the first signal on power-up
    CHECK_ERR_NO_LOG(esc_pwm_init(MOTOR_SPEED_MAX));

    // Only the remote is needed for this
    uint8_t* macs[] = {remote_mac};
    CHECK_ERR_NO_LOG(esp_now_module_init(macs, 1));

    while (!wifiQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    droneData.mutex = xSemaphoreCreateMutex();
    if (droneData.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create droneData mutex");
        return ESP_FAIL;
    }

    // Reuse the input task to keep remoteIn.throttle updated from REMOTE packets
    CHECK_ERR(input_task_init(), "Input task init failed");

    BaseType_t result = xTaskCreate(esc_program_task, "ESC_PROGRAM_TASK",
                                    configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
