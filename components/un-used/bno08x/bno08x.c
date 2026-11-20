/*
 *****************************************************************************
 * File: bno08x.c
 * Author: Jack Cairns
 * Brief: BNO08x IMU driver (SPI), built on the SH2 library in sh2/.
 *****************************************************************************
 */

#include "bno08x.h"

#include "sh2.h"            // sh2_open(), sh2_service(), sh2_setSensorCallback(), ...
#include "sh2_SensorValue.h" // sh2_decodeSensorEvent(), sh2_SensorValue_t
#include "sh2_err.h"        // SH2_OK, SH2_ERR_*
#include "sh2_hal.h"        // sh2_Hal_t, the struct filled in below

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

/*
 * HOW THE SH2 HAL WORKS
 * ---------------------
 * The SH2 library never touches hardware. You hand it a sh2_Hal_t (a struct
 * of 5 function pointers) via sh2_open(), and it calls back into them.
 *
 *   - Nothing runs in the background. The library only moves data when you
 *     call sh2_service(); that call in turn calls hal->read() (and
 *     hal->write() if something is queued). So you call sh2_service() whenever
 *     the sensor's INT pin goes low, or on a regular poll.
 *
 *   - Data is in SHTP packets: a 4-byte header (16-bit length, channel,
 *     sequence) followed by payload. The length field (bit 15 = continuation
 *     flag, mask it off) tells you how many bytes the whole packet has.
 *
 *   - Over SPI the sensor is the one that decides when there is data: it
 *     pulls INT low, then you clock out the 4-byte header, read the length,
 *     and clock the rest. Reads and writes both start with CS low.
 *
 *   - The library has no idea about time except via getTimeUs(); it uses it
 *     for timeouts and to timestamp sensor events.
 */

// ---------------------------------------------------------------------------
// HAL functions (see sh2_hal.h for the full contract)
// ---------------------------------------------------------------------------

/**
 * Called once by sh2_open().
 * Init SPI bus/device + GPIOs (CS, INT, RST), then hard-reset the sensor
 * (pulse RST low, release, wait for it to boot).
 * @return 0 on success, non-zero on failure.
 */
static int hal_open(sh2_Hal_t* self)
{
    // TODO
    return -1;
}

/**
 * Called by sh2_close().
 * Hold the sensor in reset and release SPI / GPIO resources.
 */
static void hal_close(sh2_Hal_t* self)
{
    // TODO
}

/**
 * Called by sh2_service() every time it wants to poll for data.
 *
 * If a full SHTP packet is available (INT is low): read it into pBuffer
 * (never more than len bytes), set *t_us to the time INT was detected, and
 * return the packet length.
 * If nothing is available: return 0. Do not block.
 */
static int hal_read(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len, uint32_t* t_us)
{
    // TODO
    return 0;
}

/**
 * Called when the library has a packet to send (header already included in
 * pBuffer).
 *
 * If the sensor can't accept data right now return 0 and the library will
 * retry later. Otherwise transmit and return the number of bytes accepted
 * (normally len).
 */
static int hal_write(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len)
{
    // TODO
    return 0;
}

/**
 * Free running microsecond counter. May roll over at 2^32.
 */
static uint32_t hal_getTimeUs(sh2_Hal_t* self)
{
    // TODO
    return 0;
}

// The struct handed to sh2_open()
static sh2_Hal_t s_hal = {
    .open = hal_open,
    .close = hal_close,
    .read = hal_read,
    .write = hal_write,
    .getTimeUs = hal_getTimeUs,
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// TODO
