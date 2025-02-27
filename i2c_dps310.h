#ifndef i2c_dps310_h
#define i2c_dps310_h

#include <Adafruit_DPS310.h>

#include "NmeaXDR.h"
#include "Nmea0183Msg.h"

#define DPS310_I2C_ADDR (0x77)

Adafruit_DPS310 i2c_dps310_sensor;

void i2c_dps310_report() {
  if (i2c_dps310_sensor.temperatureAvailable() && i2c_dps310_sensor.pressureAvailable()) {
    sensors_event_t temp_event, pressure_event;
    dps.getEvents(&temp_event, &pressure_event);
    gen_nmea0183_xdr("$BBXDR,C,%.2f,C,TEMP_DPS310", temp_event.temperature);      // C
    gen_nmea0183_xdr("$BBXDR,P,%.2f,P,TEMP_DPS310", 100 * temp_event.pressure);   // Pa
  }
}

bool i2c_dps310_try_init() {
  bool i2c_dps310_found = false;
  for (int i = 0; i < 3; i++) {
    i2c_dps310_found = i2c_dps310_sensor.begin_I2C(DPS310_I2C_ADDR, &Wire);
    if (i2c_dps310_found) {
      break;
    }
    delay(10);
  }
  if (i2c_dps310_found) {
    gen_nmea0183_msg("$BBTXT,01,01,01,ENVIRONMENT found dps310 sensor at address=0x%s", String(DPS310_I2C_ADDR, HEX).c_str());
    i2c_dps310_sensor.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    i2c_dps310_sensor.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    app.onRepeat(5000, []() {
      i2c_dps310_report();
    });
  }
  return i2c_dps310_found;
}

#endif
