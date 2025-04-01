#ifndef i2c_as3935_h
#define i2c_as3935_h

#include <Wire.h>
#include <DFRobot_AS3935_I2C.h>

#include "NmeaXDR.h"
#include "NmeaChecksum.h"

volatile int8_t AS3935IsrTrig = 0;

// Connect the sensor's IRQ pin to a GPIO pin on the microcontroller
// then replace the number below with the GPIO pin number
#define AS3935_IRQ_PIN       G7

// Antenna tuning capcitance (must be integer multiple of 8, 8 - 120 pf)
#define AS3935_CAPACITANCE   96

// Indoor/outdoor mode selection
#define AS3935_INDOORS       0
#define AS3935_OUTDOORS      1
#define AS3935_MODE          AS3935_INDOORS

// Enable/disable disturber detection
#define AS3935_DIST_DIS      0
#define AS3935_DIST_EN       1
#define AS3935_DIST          AS3935_DIST_EN

// I2C address
#define AS3935_I2C_ADDR      AS3935_ADD3

void AS3935_ISR();

DFRobot_AS3935_I2C  i2c_as3935_sensor((uint8_t)AS3935_IRQ_PIN, (uint8_t)AS3935_I2C_ADDR);

void i2c_as3935_report() 
  if (digitalRead(lightningInt) == HIGH) {
    // Hardware has alerted us to an event, now we read the interrupt register
    // to see exactly what it is.
    uint8_t intVal = i2c_as3935_sensor.readInterruptReg();
    if (intVal == NOISE_INT) {
      // Noise Detected!
      //Serial.println("Noise Detected!");
    }
    else if (intVal == DISTURBER_INT) {
      // Disturber Detected!
      //Serial.println("Disturber Detected!");
    }
    else if (intVal == LIGHTNING_INT) {
      //Serial.println("Lightning Strike Detected!");
      // Lightning! Now how far away is it? Distance estimation takes into
      // account any previously seen events in the last 15 seconds.
      gen_nmea0183_xdr("$BBXDR,D,%.0f,M,LIGHTNING_RANGE", (float)i2c_as3935_sensor.distanceToStorm() * 1000.0);
      gen_nmea0183_xdr("$BBXDR,X,%.0f,,LIGHTNING_LEVEL", (float)i2c_as3935_sensor.lightningEnergy());
    }
  }
}

bool i2c_as3935_try_init() {
  bool i2c_as3935_found = false;

  i2c_as3935_found = i2c_as3935_sensor.begin();
  if (i2c_as3935_found) {
    i2c_as3935_sensor.defInit();
    attachInterrupt(digitalPinToInterrupt(AS3935_IRQ_PIN), AS3935_ISR, RISING);
    i2c_as3935_sensor.manualCal(AS3935_CAPACITANCE, AS3935_MODE, AS3935_DIST);

    gen_nmea0183_msg("$BBTXT,01,01,01,ENVIRONMENT found as3935 sensor at address=0x%s", String(AS3935_I2C_ADDR, HEX).c_str());
    app.onRepeat(10, []() {
      i2c_as3935_report();
    });
  }
  return i2c_as3935_found;
}

void IRAM_ATTR AS3935_ISR() {
  AS3935IsrTrig = 1;
}

#endif

