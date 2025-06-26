#include <stdio.h>

#include "nrf24l01plus.h"

void app_main(void)
{
    radio_send("Hello world\r\n");
}

