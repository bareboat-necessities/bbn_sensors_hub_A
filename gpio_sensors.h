#ifndef gpio_sensors_h
#define gpio_sensors_h

#include "gpio_pir_as312.h"
#include "gpio_limit_switch.h"

void gpio_sensors_init() {
  gpio_pir_as312_try_init();
  gpio_limit_switch_try_init();
}

#endif
