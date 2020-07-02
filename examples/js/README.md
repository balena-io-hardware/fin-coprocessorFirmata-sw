# Javascript Examples

A selection of [firmata.js](https://github.com/firmata/firmata.js/tree/master/packages/firmata.js) examples for controlling the coprocessor.

The default [configuration](config.json), `baud: 57600, port: /dev/ttyS0`, will work for the balenaFin.
If testing on a devkit, change this as appropriate for your host/client.

## [blink](blink.js)

This example blinks an LED on the silabs devkit.

## [idac](idac.js)

This example returns shows how to configure and set the IDAC on the coprocessor.

## [balena](balena.js)

This example returns the specific balena firmata firmware release version.
This tracks our release version rather than the firmata version.

## [sleep](sleep.js)

This example demonstrates how to use the sleep functionality of the coprocessor to power down the compute module.

## [adxl345](adxl345.js)

This example shows how to talk to an I2C sensor; an ADXL345 accelerometer.

## [testbot](testbot.js)

This example demonstrates how to communicate with a Hatbot to control a DUT.