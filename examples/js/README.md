# Javascript Examples

A selection of [firmata.js](https://github.com/firmata/firmata.js/tree/master/packages/firmata.js) examples for controlling the coprocessor.

The default [configuration](config.json), will work for the balenaFin.
If testing on a devkit, change this as appropriate for your host/client.

## [blink.js](blink.js)

This example blinks an LED on the silabs devkit.

## [idac.js](idac.js)

This example returns shows how to configure and set the IDAC on the coprocessor.

## [balena.js](balena.js)

This example returns the specific balena firmata firmware release version.
This tracks our release version rather than the firmata version.

## [sleep.js](sleep.js)

This example demonstrates how to use the sleep functionality of the coprocessor to power down the compute module.

## [adxl345.js](adxl345.js)

This example shows how to talk to an I2C sensor; an ADXL345 accelerometer.

## [testbot.js](testbot.js)

This example demonstrates how to communicate with a Hatbot to control a DUT.