#ifndef PINS_H
#define PINS_H

#include <Arduino.h>
//CORTEX-10K FC - PIN DEFINITIONS
//spi shared bus
//spi bus -> shared by RF, SD, both baros, temp sensor
#define PIN_SPI_MOSI 35 //SDI
#define PIN_SPI_SCK 36 //SCK/SCL
#define PIN_SPI_MISO 37 //SDO

//spi chip select (active LOW unless noted)
#define PIN_CS_RF95 4 //RFM95W chip select
#define PIN_CS_SD 16 //SD card CS (has 4.7k pull-up, active LOW)
#define PIN_CS_BARO_ONBOARD 34 //BMP390L onboard barometer CS
#define PIN_CS_BARO_EXT 47 //expansion / nose-cone barometer CS
#define PIN_CS_TEMP 33 //MAX31855 thermocouple CS

//RFM95W setup
#define PIN_RF_RESET 5   //active LOW reset
#define PIN_RF_DIO0 6   //TX/RX done interrupt
#define PIN_RF_DIO1 7   //FHSS / timeout interrupt
#define PIN_GPS_RESET 3 //GPS reset

//I2C setup (IMU only)
#define PIN_I2C_SDA 8 //SDA pin
#define PIN_I2C_SCL 17 //SCL pin
#define PIN_IMU_INTERRUPT 38 //imu interrupt pin

//GPS setup
#define PIN_GPS_TXD 13 //GPS TX -> esp RX
#define PIN_GPS_RXD 48 //GPS RX <- esp TX
#define PIN_GPS_LOCK_LED 18 //driven HIGH by GPS module on lock
#define PIN_GPS_TRACK 26 //GPS track pin

//sd card extra
#define PIN_SD_DETECT 39 //sd card detect switch

//pyro channels (active HIGH via MOSFET gate)
#define PIN_PYRO_1 14
#define PIN_PYRO_2 2
#define PIN_PYRO_3 9
#define PIN_PYRO_4 1

//status RGB LED (active HIGH)
#define PIN_RGB_RED 40
#define PIN_RGB_GREEN 41
#define PIN_RGB_BLUE 42

//buzzer
#define PIN_BUZZER 45 //strapping pin
//DO NOT DRIVE HIGH BEFORE SETUP()/BOOT COMPLETION.

//expansion ports
#define PIN_EXPANSION_46 46 //strapping pin - do not pull HIGH at startup
//DO NOT DRIVE HIGH BEFORE SETUP()/BOOT COMPLETION.

//battery voltage check
#define PIN_VBAT_SENSE 15 //ADC1 channel, battery voltage divider

#endif // PINS_H
