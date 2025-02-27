#ifndef i2c_shtc3_h
#define i2c_shtc3_h

#include <Wire.h>
#include <Adafruit_SHTC3.h>

#include "NmeaXDR.h"
#include "Nmea0183Msg.h"

#define SHTC3_I2C_ADDR 0x70  // SHTC3 I2C address (0x71 is 'read' address)

Adafruit_SHTC3 i2c_shtc3_sensor;  

void i2c_shtc3_report() {
  sensors_event_t humidity, temp;
  i2c_shtc3_sensor.getEvent(&humidity, &temp);
  gen_nmea0183_xdr("$BBXDR,H,%.2f,P,HUMI_SHTC3", humidity.relative_humidity);  // %
  gen_nmea0183_xdr("$BBXDR,C,%.2f,C,TEMP_SHTC3", temp.temperature);            // C
}

bool i2c_shtc3_try_init() {
  bool i2c_shtc3_found = false;
  for (int i = 0; i < 3; i++) {
    i2c_shtc3_found = i2c_shtc3_sensor.begin(&Wire);  // Initialize SHTC3 sensor
    if (i2c_shtc3_found) {
      break;
    }
    delay(10);
  }
  if (i2c_shtc3_found) {
    gen_nmea0183_msg("$BBTXT,01,01,01,ENVIRONMENT found shtc3 sensor at address=0x%s", String(SHTC3_I2C_ADDR, HEX).c_str());
    app.onRepeat(5000, []() {
      i2c_shtc3_report();
    });
  }
  return i2c_shtc3_found;
}

#endif
