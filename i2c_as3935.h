#ifndef i2c_as3935_h
#define i2c_as3935_h

#include <Wire.h>
#include <AS3935I2C.h>

#include "NmeaXDR.h"
#include "NmeaChecksum.h"

// Connect the sensor's IRQ pin to a GPIO pin on the microcontroller
// then replace the number below with the GPIO pin number
#define AS3935_IRQ_PIN  G7

AS3935I2C i2c_as3935_sensor(AS3935I2C::AS3935I2C_A03, AS3935_IRQ_PIN);

void IRAM_ATTR AS3935_ISR();

volatile bool AS3935IsrTrig = false;

constexpr uint32_t SENSE_INCREASE_INTERVAL = 15000;  // 15 s sensitivity increase interval
uint32_t sense_adj_last_ = 0L;                       // time of last sensitivity adjustment

void i2c_as3935_report() {
  if (AS3935IsrTrig) {
    delay(3);
    AS3935IsrTrig = false;
    uint8_t event = i2c_as3935_sensor.readInterruptSource();
    if (event == AS3935MI::AS3935_INT_L) {
      // Get distance data
      uint8_t lightningDistKm = i2c_as3935_sensor.readStormDistance();

      // Get lightning energy intensity
      uint32_t lightningEnergyVal = i2c_as3935_sensor.readEnergy();

      // Lightning! Now how far away is it? Distance estimation takes into
      // account any previously seen events in the last 15 seconds.
      gen_nmea0183_xdr("$BBXDR,D,%.0f,M,LIGHTNING_RANGE", (float)lightningDistKm * 1000.0);
      gen_nmea0183_xdr("$BBXDR,X,%.0f,,LIGHTNING_LEVEL", (float)lightningEnergyVal);
    } else if (event == AS3935MI::AS3935_INT_D) {
      gen_nmea0183_msg("$BBTXT,01,01,01,ENVIRONMENT LIGHTNING %s", "Disturber discovered");
      
      // increasing the Watchdog Threshold and / or Spike Rejection setting improves the AS3935s resistance 
      // against disturbers but also decrease the lightning detection efficiency (see AS3935 datasheet)
      uint8_t wdth = i2c_as3935_sensor.readWatchdogThreshold();
      uint8_t srej = i2c_as3935_sensor.readSpikeRejection();

      if ((wdth < AS3935MI::AS3935_WDTH_10) || (srej < AS3935MI::AS3935_SREJ_10)) {
        sense_adj_last_ = millis();
        // alternatively increase spike rejection and watchdog threshold 
        if (srej < wdth) {
          i2c_as3935_sensor.increaseSpikeRejection();
        } else {
          i2c_as3935_sensor.increaseWatchdogThreshold();
        }
      }
    } else if (event == AS3935MI::AS3935_INT_NH) {
      gen_nmea0183_msg("$BBTXT,01,01,01,ENVIRONMENT LIGHTNING %s", "Noise level too high");
      // if the noise floor threshold setting is not yet maxed out, increase the setting.
      // note that noise floor threshold events can also be triggered by an incorrect
      // analog front end setting.
      uint8_t nf_lev = AS3935MI::AS3935_NFL_0;
      i2c_as3935_sensor.increaseNoiseFloorThreshold(nf_lev);
    }
  }

  // increase sensor sensitivity every once in a while. SENSE_INCREASE_INTERVAL controls how quickly the code 
  // attempts to increase sensitivity. 
  if (millis() - sense_adj_last_ > SENSE_INCREASE_INTERVAL) {
    sense_adj_last_ = millis();
    uint8_t wdth = i2c_as3935_sensor.readWatchdogThreshold();
    uint8_t srej = i2c_as3935_sensor.readSpikeRejection();
    if ((wdth > AS3935MI::AS3935_WDTH_0) || (srej > AS3935MI::AS3935_SREJ_0)) {
      // alternatively derease spike rejection and watchdog threshold 
      if (srej > wdth) {
        i2c_as3935_sensor.decreaseSpikeRejection();
      } else {
        i2c_as3935_sensor.decreaseWatchdogThreshold();
      }
    }
  }
}

bool i2c_as3935_try_init() {
  pinMode(AS3935_IRQ_PIN, INPUT);
  bool i2c_as3935_found = i2c_as3935_sensor.begin() == 0;
  if (i2c_as3935_found) {
    i2c_as3935_sensor.checkConnection();
    i2c_as3935_sensor.checkIRQ();

    // calibrate the resonance frequency. failing the resonance frequency could indicate an issue 
    // of the sensor. resonance frequency calibration will take about 1.7 seconds to complete.	
    uint8_t division_ratio = AS3935MI::AS3935_DR_16;
    if (F_CPU < 48000000) { // fixes https://bitbucket.org/christandlg/as3935mi/issues/12/autocalibrate-no-longer-working
      division_ratio = AS3935MI::AS3935_DR_64;
    }

    int32_t frequency = 0;
    i2c_as3935_sensor.calibrateResonanceFrequency(frequency, division_ratio);
    i2c_as3935_sensor.calibrateRCO();

    i2c_as3935_sensor.writeAFE(AS3935MI::AS3935_OUTDOORS);
    i2c_as3935_sensor.writeNoiseFloorThreshold(AS3935MI::AS3935_NFL_5);
    i2c_as3935_sensor.writeWatchdogThreshold(AS3935MI::AS3935_WDTH_2);
    i2c_as3935_sensor.writeSpikeRejection(AS3935MI::AS3935_SREJ_2);
    i2c_as3935_sensor.writeMinLightnings(AS3935MI::AS3935_MNL_1);
    i2c_as3935_sensor.writeMaskDisturbers(false);  

    attachInterrupt(digitalPinToInterrupt(AS3935_IRQ_PIN), AS3935_ISR, RISING);

    gen_nmea0183_msg("$BBTXT,01,01,01,ENVIRONMENT found as3935 sensor at address=0x%s", String(AS3935_I2C_ADDR, HEX).c_str());
    app.onRepeat(10, []() {
      i2c_as3935_report();
    });
  }
  return i2c_as3935_found;
}

void IRAM_ATTR AS3935_ISR() {
  AS3935IsrTrig = true;
}

#endif

