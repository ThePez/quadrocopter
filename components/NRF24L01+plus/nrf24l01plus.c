#include "nrf24l01plus.h"
#include <stdio.h>

void radio_send(const char *message) {
  printf("Sending radio message: %s\n", message);
}