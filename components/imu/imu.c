/*
 *****************************************************************************
 * File: radio.c
 * Author: Jack Cairns
 * Date: 13-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "imu.h"

// STD C lib headers
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

// Custom Components
#include "hmc5883l.h"
#include "l3gd20.h"
#include "lis3DH.h"

TaskHandle_t imuTask = NULL;
QueueHandle_t imuQueue = NULL;

//////////////////////////////////////////////////////////////////////////////

/* imu_task()
 * ----------
 * Continuously reads sensor data from the LIS3DH accelerometer
 * and L3GD20 gyroscope.
 *
 * Calculates pitch, roll, and yaw angles using a complementary filter
 * to fuse accelerometer and gyro data. Sends updated telemetry to
 * the flight controller via imuQueue.
 *
 * Runs continuously as a FreeRTOS task.
 *
 * Dependencies:
 *   - Uses lis3dh_init(), gyro_init(), lisReadAxisData(), gyroReadAxisData(),
 *     getPitchAngle(), getRollAngle().
 *   - Outputs to imuQueue.
 */
void imu_task(void* pvParams) {

    ImuParams_t* params = (ImuParams_t*) pvParams;
    SemaphoreHandle_t spiHMutex = *(params->spiHMutex);
    SemaphoreHandle_t spiVMutex = *(params->spiVMutex);
    SemaphoreHandle_t i2cMutex = *(params->i2cMutex);
    i2c_master_bus_handle_t bus = *(params->i2cHost);

    xSemaphoreTake(spiHMutex, portMAX_DELAY);
    lis3dh_init(HSPI_HOST);
    xSemaphoreGive(spiHMutex);

    xSemaphoreTake(spiVMutex, portMAX_DELAY);
    gyro_init(VSPI_HOST);
    xSemaphoreGive(spiVMutex);

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    magnetometer_init(bus);
    xSemaphoreGive(i2cMutex);

    // Setup variables
    Telemitry_t imuData = {0};
    uint8_t accSuccess, gyroSuccess, magnoSuccess;
    int16_t x, y, z;
    double pitch, roll, yaw, pitchRate, rollRate, yawRate;

    while (!imuQueue) {
        // Loop to ensure the imu queue is created
        imuQueue = xQueueCreate(IMU_QUEUE_LENGTH, sizeof(Telemitry_t));
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    while (1) {

        // Get the axis data from the accelerometer
        if (xSemaphoreTake(spiHMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lisReadAxisData(&x, &y, &z);
            xSemaphoreGive(spiHMutex);
            // printf("X: %d\r\n", x);
            // Get Pitch & Roll angles from accelerometer
            pitch = getPitchAngle(x, y, z); // Nose up with the -1 gives positive sign
            roll = getRollAngle(y, z);      // Left wing up gives positive angle

            accSuccess = 1;
        } else {
            accSuccess = 0;
        }

        // Get the axis data from the gyroscope
        if (xSemaphoreTake(spiVMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            gyroReadAxisData(&x, &y, &z);
            xSemaphoreGive(spiVMutex);

            // Calculate the changes in angles from the Gyroscope data
            pitchRate = y * GYRO_SENSITIVITY; // Nose up gives positive angle
            rollRate = x * GYRO_SENSITIVITY;  // Left wing up gives positive angle with -1
            yawRate = z * GYRO_SENSITIVITY;   // Nose turned right gives positive angle with -1
            gyroSuccess = 1;
        } else {
            gyroSuccess = 0;
        }

        // Get the axis data from the magnetometer
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Read Compass data
            magnetometer_read_axis(&x, &y, &z);
            xSemaphoreGive(i2cMutex);
            if (accSuccess) {
                yaw = getYawAngle(pitch, roll, x, y, z);
                magnoSuccess = 1;
            } else {
                magnoSuccess = 0;
            }

        } else {
            magnoSuccess = 0;
        }

        // All SPI/I2C reads were successful -> process the data and send to queue
        if (accSuccess && gyroSuccess && magnoSuccess) {
            printf("Angle Pitch: %.03f, Roll: %.03f, Yaw %.03f\r\n", pitch, roll, yaw);
            printf("Rates Pitch: %.03f, Roll: %.03f, Yaw: %.03f\r\n", pitchRate, rollRate, yawRate);

            // Calculate the Pitch & Roll angles using both sets of data
            imuData.pitchAngle = pitch;
            imuData.pitchRate = pitchRate;
            imuData.rollAngle = roll;
            imuData.rollRate = rollRate;
            imuData.yawAngle = yaw;
            imuData.yawRate = yawRate;

            // Send data to the Flight controller for processing
            if (imuQueue) {
                xQueueSendToFront(imuQueue, &imuData, pdMS_TO_TICKS(1));
            }
        }

        // Repeat this vey quickly
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* IMU sign check test()
 * ---------------------
 * This task reads accelerometer and gyro data, calculates pitch/roll
 * angles from the accelerometer, and prints raw gyro rates.
 * Use this to check if signs and axes match.
 */
void imu_sign_check_task(void) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    lis3dh_init(HSPI_HOST);
    gyro_init(VSPI_HOST);

    while (1) {
        // Read accelerometer data
        lisReadAxisData(&ax, &ay, &az);
        double accPitch = getPitchAngle(ax, ay, az);
        double accRoll = getRollAngle(ay, az);

        // Read gyro raw rates
        gyroReadAxisData(&gx, &gy, &gz);

        printf("ACC: Pitch = %.2f deg, Roll = %.2f deg | ", -accPitch, accRoll);
        printf("GYRO: X = %d, Y = %d, Z = %d (raw dps)\r\n", -gx, gy, -gz);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
