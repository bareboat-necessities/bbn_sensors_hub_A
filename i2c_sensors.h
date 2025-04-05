#ifndef i2c_sensors_h
#define i2c_sensors_h

bool sht30_found = false;

#include "i2c_sht30.h"
#include "i2c_qmp6988.h"
#include "i2c_bmp280.h"
#include "i2c_bme680.h"
#include "i2c_dht12.h"
#include "i2c_ads1115.h"
#include "i2c_bh1750fvi_tr.h"
#include "i2c_ain_4_20ma.h"
#include "i2c_sgp30.h"
#include "i2c_vl53l0x.h"
#include "i2c_ina219.h"
#include "i2c_as3935.h"
#include "i2c_dps310.h"
#include "i2c_shtc3.h"

void i2c_sensors_scan(bool i2c_alt_enable_scan) {
  i2c_as3935_try_init();
  sht30_found = i2c_sht30_try_init();
  i2c_ina219_try_init(&Wire);
  i2c_ads1115_try_init(&Wire, G2, G1, I2C_FREQ);
  i2c_ain_4_20ma_try_init(&Wire, G2, G1, I2C_FREQ);
  if (i2c_alt_enable_scan) {
    i2c_ina219_try_init(&Wire1);
    i2c_ads1115_try_init(&Wire1, G38, G39, I2C_FREQ);
    i2c_ain_4_20ma_try_init(&Wire1, G38, G39, I2C_FREQ);
  }
  bool dps310_found = i2c_dps310_try_init();
  if (!dps310_found) {
    i2c_bmp280_try_init();
  }
  bool qmp6988_found = i2c_qmp6988_try_init();
  if (!dps310_found) {
    i2c_bme680_try_init();
  }
  i2c_dht12_try_init();
  if (!qmp6988_found) {
    i2c_shtc3_try_init();
  }
  i2c_sgp30_try_init();
  i2c_bh1750fvi_tr_try_init();
  i2c_vl53l0x_try_init();
}

void i2c_sensors_update() {
}

#endif
