#include <stdio.h>

#include "nrf24l01plus.h"

void app_main(void)
{
    nrf24l01plus_init(SPI2_HOST);
}
