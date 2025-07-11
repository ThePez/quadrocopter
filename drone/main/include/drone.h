/*
 *****************************************************************************
 * File: drone.h
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef DRONE_H
#define DRONE_H

// STD C lib headers
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

// KConfig header
#include "sdkconfig.h"

// ESP-IDF Prebuilts
#include "driver/i2c_master.h"
#include "driver/mcpwm_prelude.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Custom Components
#include "hamming.h"
#include "hmc5883l.h"
#include "l3gd20.h"
#include "lis3DH.h"
#include "nrf24l01plus.h"

#define NRF24L01PLUS_IQR_TRIGGER (1 << 30)

// SPI Pins
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_CLK 14
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_CLK 18

// Motor/ESC's GPIO Pins
// Ensure these are sequential as the PWM setup requires this.
// If not possible alter the esc_pwm_init() function.
#define MOTOR_A_CW 4   // Front left
#define MOTOR_B_CCW 16 // Rear left
#define MOTOR_C_CW 17  // Rear right
#define MOTOR_D_CCW 5  // Front right

// Throttle speeds / Duty cycle's in us
#define MOTOR_SPEED_MIN 1000
#define MOTOR_SPEED_MAX 2000
#define MAX_MOTOR_ADJUSTMENT 100
#define NUMBER_OF_MOTORS 4

#define GYRO_ALPHA 0.40

// Stack Sizes
#define SYS_STACK (configMINIMAL_STACK_SIZE * 2)
#define RADIO_STACK (configMINIMAL_STACK_SIZE * 2)
#define LIS_STACK (configMINIMAL_STACK_SIZE * 2)
// Priorities
#define SYS_PRIO (tskIDLE_PRIORITY + 2)
#define RADIO_PRIO (tskIDLE_PRIORITY + 3)
#define LIS_PRIO (tskIDLE_PRIORITY + 4)

///////////////////////////// Structures & Enums /////////////////////////////

typedef enum { MOTOR_A, MOTOR_B, MOROT_C, MOTOR_D } MotorIndex;

typedef struct {
    double kp;           // Proportional scaler
    double ki;           // Intergral scaler
    double kd;           // Derivative scaler
    double intergral;    // running average total of the error
    double prevError;    // previous error
    uint64_t prevTimeUS; // Last update time in us
} PID_t;

typedef struct {
    uint16_t throttle;
    double pitch;
    double roll;
    double yaw;
} ControlSetPoint_t;

typedef struct {
    double errPitch;
    double errRoll;
    double errYaw;
} ControlError_t;

typedef struct {
    double pitchAngle;
    double rollAngle;
    double yawAngle;
    uint64_t prevTime;
} Telemitry_t;

///////////////////////////////// Prototypes /////////////////////////////////

// Task Function Prototypes
void flight_controller(void);
void radio_task(void);
void imu_task(void);
void imu_sign_check_task(void);

// Setup Function Prototypes
void spi_bus_setup(spi_host_device_t host);
i2c_master_bus_handle_t i2c_bus_setup(void);
void esc_pwm_init(void);

// Other functions
void esc_pwm_set_duty_cycle(MotorIndex motor, uint16_t duty_cycle);
void update_escs(uint16_t throttle, double pitchPID, double rollPID, double yawPID);
double pid_update(PID_t* pid, double error);
void decode_packet(void* input, void* output);
void print_task_stats(void);

#endif
