#ifndef i2c_sensors_h
#define i2c_sensors_h

#include "i2c_sht30.h"
#include "i2c_qmp6988.h"
#include "i2c_bmp280.h"
#include "i2c_bme680.h"
#include "i2c_dht12.h"
#include "i2c_ads1115.h"

void i2c_sensors_scan() {
  i2c_ads1115_try_init();
  i2c_sht30_try_init();
  i2c_dht12_try_init();
  i2c_bmp280_try_init();
  i2c_qmp6988_try_init();
  i2c_bme680_try_init();
}

void i2c_sensors_update() {
}

#endif
