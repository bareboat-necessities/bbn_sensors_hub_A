#include <M5Unified.h>
#include <Wire.h>
#include <ReactESP.h>  // https://github.com/mairas/ReactESP

using namespace reactesp;
ReactESP app;

#include "NmeaXDR.h"
#include "Nmea0183Msg.h"
#include "1w_sensors.h"
#include "i2c_sensors.h"
#include "gpio_sensors.h"

bool i2c_alt_enable = true;

static const char* firmware_tag = "bbn_sensors_hub_A";

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  //M5.begin();
  Wire.begin(G2, G1, 100000UL);
  if (i2c_alt_enable) {
    Wire1.begin(G38, G39, 100000UL);
  }
  Serial.begin(38400);
  gen_nmea0183_msg("$BBTXT,01,01,01,FirmwareTag: %s", firmware_tag);
  i2c_sensors_scan(i2c_alt_enable);
  one_wire_sensors_scan();
  gpio_sensors_init();
}

void loop() {
  M5.update();
  i2c_sensors_update();
  app.tick();
  delayMicroseconds(10000);
}
