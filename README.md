# bbn-sensors-hub-A
NMEA 0183 XDR Sensors on esp32 m5stack atomS3-lite

No soldering required. Connect sensors to i2c. Load firmware to atomS3-lite.
(On Bareboat Necessites OS it's just copy-paste of a script, see below). Plug and play.

## Table of Content

- [bbn_esp32_sensors_hub](#bbn-esp32-sensors-hub)
  * [Hardware](#hardware)
    + [Accessories](#accessories)
  * [Integration with SignalK](#integration-with-signalk)
  * [Supported Sensors](#supported-sensors)
    + [AS3935 Lightning Detector sensor to NMEA-0183](#as3935-lightning-detector-sensor-to-nmea-0183)
    + [Multiple Dallas Temperature 1-Wire Sensors](#multiple-dallas-temperature-1-wire-sensors)
    + [INA219 Voltage and Current sensors (up to eight)](#ina219-voltage-and-current-sensors-up-to-eight)
    + [SGP30 Air Quality and TVOC sensor connected to M5Stack AtomS3-Lite via i2c](#sgp30-air-quality-and-tvoc-sensor-connected-to-m5stack-atoms3-lite-via-i2c)
    + [Time-of-Flight Distance Ranging Sensor Unit (VL53L0X)](#time-of-flight-distance-ranging-sensor-unit-vl53l0x)
    + [Bosch BME680 Air Sensor](#bosch-bme680-air-sensor)
    + [DPS310 Air Pressure Sensor](#dps310-sensor)
    + [M5Stack 4-20mA current sensors to NMEA-0183 (up to two with secondary i2c bus)](#m5stack-4-20ma-current-sensors-to-nmea-0183-up-to-two-with-secondary-i2c-bus)
    + [Voltmeter on m5stack AtomS3 with ADS1115 M5Stack Voltmeter (up to two with secondary i2c bus)](#voltmeter-on-m5stack-atoms3-with-ads1115-m5stack-voltmeter-up-to-two-with-secondary-i2c-bus)
    + [M5Stack DLight Illuminance sensor to NMEA-0183](#m5stack-dlight-illuminance-sensor-to-nmea-0183)
    + [Environmental Air Sensors (Pressure/Temp/Humidity) BMP280, DHT12, QMP6988, SHT30, SHTC3](#environmental-air-sensors-pressuretemphumidity-bmp280-dht12-qmp6988-sht30-shtc3)
  * [NMEA XDR Output](#nmea-xdr-output)
  * [Loading Firmware](#loading-firmware)
    + [On Bareboat Necessities (BBN) OS (full)](#on-bareboat-necessities-bbn-os-full)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


## Hardware

- m5stack atomS3-lite:  https://shop.m5stack.com/products/atoms3-lite-esp32s3-dev-kit
- m5stack ATOMIC PortABC Extension Base: https://shop.m5stack.com/products/atomic-portabc-extension-base

### Accessories

- Grove to StemmaQT Cables for i2c, or StemmaQT to Dupont
- Grove to qwiic
- Grove T-Connector https://shop.m5stack.com/products/grove-t-connector-5pcs-a-pack
- Grove2Dupont Conversion Cable https://shop.m5stack.com/products/grove2dupont-conversion-cable-20cm-5pairs
- GROVE-to-VH3.96
https://shop.m5stack.com/products/3-96-transfer-unit

(There are 3 kinds of connector different sensor could use: m5stack Grove, Adafruit Stemma QT, SparkFun Qwiic)

While both I2C Grove and Qwiic use the same I2C communication protocol, the key difference in voltage is that Grove can operate with both 3.3V and 5V devices, while Qwiic is strictly 3.3V only; meaning most Qwiic devices require voltage level shifting if connecting to a 5V system, whereas Grove modules can generally handle a wider voltage range without needing additional circuitry

## Integration with SignalK

Integration with SignalK is done via NMEA XDR Parser SignalK plugin.

More: https://github.com/GaryWSmith/xdr-parser-plugin

Recommended config file for XDR Parser SignalK plugin:

https://github.com/bareboat-necessities/lysmarine_gen/blob/bookworm/install-scripts/4-server/files/xdrParser-plugin.json

When registering a serial connection in SignalK use device symbolic link from /dev/serial/by-id/ (instead of direct /dev/ttyXXX name).

## Supported Sensors

### AS3935 Lightning Detector sensor to NMEA-0183

AS3935 Lightning Detector connected to M5Stack AtomS3-Lite via i2c and one additional digital GPIO pin (G7) (interrupt)

Install AS3935 Lightning Detector away from other wires to avoid noise.
Don't leave unused wires unconnected. Bring them to the GND. Avoid noisy power.

A piezoelectric kitchen lighter can be used for crude testing.

Generates NMEA-0183 XDR sentences (USB Serial) like this:

````
$BBXDR,D,1000,M,LIGHTNING_RANGE*16
$BBXDR,X,410415,,LIGHTNING_LEVEL*4A
````

DFRobot Gravity AS3935 Lightning Detector:

https://www.dfrobot.com/product-1828.html

### Multiple Dallas Temperature 1-Wire Sensors


DS18B20 with GikFun plugin terminal board (includes required pull-up resistor for 1-wire bus)

DS18B20 data is connected to pin G8 on m5stack AtomS3-lite

Can be used for

- Engine temperature
- Exhaust temperature
- Fridge temperature
- Engine coolant temperature
- Batteries bank temperature
- MPPT controller temperature
- etc

Supports multiple 1-wire Dallas Temperature sensors.

Generates NMEA 0183 XDR and TXT sentences on USB serial 

````
$BBTXT,01,01,01,TEMPERATURE sensors found=1*0A
$BBTXT,01,01,02,TEMPERATURE found sensor address=28478546D4523ABF*1A
$BBXDR,C,23.94,C,TEMP_28478546D4523ABF*38
$BBXDR,C,24.00,C,TEMP_28478546D4523ABF*32
````

Waterproof Digital Temperature Sensor DS18B20 with GikFun plugin terminal board

https://www.amazon.com/Gikfun-DS18B20-Waterproof-Digital-Temperature/dp/B08V93CTM2

### INA219 Voltage and Current sensors

Load and your sensor need to share common ground.

NMEA XDR Sentences:

````
$BBXDR,U,1.036,V,VOLT_INA219_2*28
$BBXDR,I,0.000,A,AMPS_INA219_2*29
$BBXDR,W,0.000,W,WATT_INA219_2*38
````

https://www.adafruit.com/product/904


###  SGP30 Air Quality and TVOC sensor connected to M5Stack AtomS3-Lite via i2c

Generates NMEA-0183 XDR sentences (USB Serial) like this:

````
$BBXDR,X,5,,TVOC*2D
$BBXDR,X,415,,eCO2*7D
$BBXDR,X,13507,,rawH2*38
$BBXDR,X,18875,,rawEthanol*14
````

Here are some TVOC levels and their associated health effects:

- 0 ppb: Good
- 220 ppb: Moderate, may cause some symptoms
- 660 ppb: Unhealthy if sensitive, more likely to cause symptoms
- 2200 ppb: Unhealthy
- 3300 ppb: Very unhealthy
- 4400 ppb: Hazardous

SGP30 Air Quality and TVOC sensor :

[M5Stack SGP30 Air Quality and TVOC sensor](https://shop.m5stack.com/products/tvoc-eco2-gas-unit-sgp30)


### Time-of-Flight Distance Ranging Sensor Unit (VL53L0X) 

Time-of-Flight Distance Ranging Sensor Unit (VL53L0X) sensor connected to M5Stack AtomS3-Lite via i2c

Generates NMEA-0183 XDR sentences (USB Serial) like this:

````
$BBXDR,D,0.2410,M,RANGE_VL53L0X*66
$BBXDR,D,0.2430,M,RANGE_VL53L0X*64
````

Time-of-Flight Distance Ranging Sensor Unit (VL53L0X) sensor :

https://shop.m5stack.com/products/tof-sensor-unit


### Bosch BME680 Air Sensor

Bosch BME680 Sensor to NMEA 0183 via i2c

Connected to M5Stack AtomS3-Lite via i2c

- Air Pressure
- Air Temperature
- Humidity
- Gas Electrical Resistance

Generates NMEA-0183 XDR sentences (USB Serial) like this:

````
$BBXDR,C,25.97,C,TEMP_BME680*4E
$BBXDR,P,102061.00,P,PRES_BME680*5B
$BBXDR,H,48.09,P,HUMI_BME680*4F
$BBXDR,H,32.17,,GASR_BME680*03
````

Bosch BME680 with Temperature Humidity Air Pressure Sensor:

[Adafruit BME680](https://www.adafruit.com/product/3660)


### M5Stack 4-20mA current sensors to NMEA-0183 (up to two with secondary i2c bus)

M5Stack Analog to I2C Unit (4-20mA Input) connected to M5Stack AtomS3-Lite via i2c

Can be used with many industrial 4-20 mAmps current sensors.

Ex:
- Tank level

Generates NMEA-0183 XDR sentences (USB Serial) like this:

````
$BBXDR,I,0.00480,A,AMPS*6B
$BBXDR,I,0.00481,A,AMPS*6A
````

Analog to I2C Unit 4-20mA Input (STM32G030)

https://shop.m5stack.com/products/ain4-20ma-unit

Would be useful for fluid level sensors like this one:

https://www.amazon.com/4-20mA-Liquid-Sensor-Throw-Sensors/dp/B07WDK2PRN
(Two wires from that sensor connect to IN+ and IN- on M5Stack Unit and on same terminal the current loop need to be powered from some DC power source)


### Voltmeter on m5stack AtomS3 with ADS1115 M5Stack Voltmeter


M5Stack Voltmeter unit connected to M5Stack AtomS3-Lite via i2c

Generates NMEA-0183 XDR sentences (USB Serial) like this:

````
$BBXDR,U,12.634,V,VOLT*50
$BBXDR,U,12.633,V,VOLT*57
````

The code is set up to measure 12 V circuits. You can easily modify it for other voltage ranges.

M5Stack Voltmeter unit:

https://shop.m5stack.com/products/voltmeter-unit-ads1115


### M5Stack DLight Illuminance sensor to NMEA-0183

M5Stack DLight illuminance sensor connected to M5Stack AtomS3-Lite via i2c

Generates NMEA-0183 XDR sentences (USB Serial) like this:

````
$BBXDR,X,132.0,L,ILLU*68
$BBXDR,X,128.0,L,ILLU*63
````

M5Stack DLight illuminance sensor :

https://shop.m5stack.com/products/dlight-unit-ambient-light-sensor-bh1750fvi-tr


### Environmental Air Sensors (Pressure/Temp/Humidity) BMP280, DHT12, QMP6988, SHT30, SHTC3

Connected via i2c

### DPS310 Sensor

Connected via i2c

## NMEA XDR Output

on USB-C port with baud rate 38400

````
stty -F /dev/ttyACM1 38400
socat stdio /dev/ttyACM1
````

## Loading Firmware

### On Bareboat Necessities (BBN) OS (full)

````
# shutdown signalk
sudo systemctl stop signalk

if [ -f bbn-flash-sensors-hub-A.sh ]; then rm bbn-flash-sensors-hub-A.sh; fi
wget https://raw.githubusercontent.com/bareboat-necessities/my-bareboat/refs/heads/master/m5stack-tools/bbn-flash-sensors-hub-A.sh
chmod +x bbn-flash-sensors-hub-A.sh 
./bbn-flash-sensors-hub-A.sh -p /dev/ttyACM1
````

