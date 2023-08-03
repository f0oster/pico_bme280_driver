# Pico BME280 I2C Driver

A naive implementation of a driver for the BME280 sensor communicating via I2C. Created for learning purposes (first time using C, and my first project working with embedded hardware). Intended to be used with the Raspberry Pico C/C++ SDK.

Core Electronics BME280 python repo (https://github.com/CoreElectronics/CE-PiicoDev-BME280-MicroPython-Module/tree/main) was used as a reference (I use the same sensor configuration), as well as vendor datasheet (https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)

## TODO:
* Error handling
* Implement support for interface configuration and sensor configuration
* Implement support for normal mode operation (currently support forced only)
* Implement measurement delays in line with the datasheet specifications (currently waiting static 5000ms between readings)
* Remove output from driver or implement customizable logging?
* ???