# BalenaFin Co-Processor Firmata [![build](https://img.shields.io/badge/release-beta-brightgreen.svg)]()

This is an implementation of the [Firmata](https://github.com/firmata/protocol) protocol for SiliconLabs BGM111. It is compatible with standard Firmata 2.5.8. Please be aware this **project is in beta release** and is subject to change frequently up until release.

### Installation

### Dockerfile

The easiest way to install the Firmata application onto your board is to run the [balena application](https://github.com/balena-io-playground/balena-fin-firmata-flash) provided. This targets the latest verision of the Balena Firmata application. This balena application will run and install [OpenOCD](http://openocd.org/) on your Fin in order to provision the Coprocessor with both a bootloader and the Firmata application.

### Build & Manually Flash

It is also possible to build the source and manually flash the Coprocessor however, in order to flash the Coprocessor you will need to either load the compiled firmware onto the Compute Module and flash it using OpenOCD or program the Coprocessor using an external programmer such as a [Segger JLink ](https://www.segger.com/products/debug-probes/j-link/).

#### Dependencies

 - cmake
 - make
 - arm-none-eabi-gcc*

If using a JLink programmer to externally flash:

 - [JLink tools](https://www.segger.com/jlink-software.html)

*Make sure this is in your `$PATH`

#### Building/Flashing

With the dependencies installed, run:

1. `make setup` to generate the build directory
2. `make balena` to execute the build

If using a JLink programmer to externally flash:

3. `make flash` to flash to a device

### Firmata Protocol v2.5.8

| type                  | command | MIDI channel | first byte          | second byte     | support              |
| --------------------- | ------- | ------------ | ------------------- | --------------- | -------------------- |
| analog I/O message    | 0xE0    | pin #        | LSB(bits 0-6)       | MSB(bits 7-13)  |          ✅          |
| digital I/O message   | 0x90    | port         | LSB(bits 0-6)       | MSB(bits 7-13)  |          ✅          |
| report analog pin     | 0xC0    | pin #        | disable/enable(0/1) | - n/a -         |          ✅          |
| report digital port   | 0xD0    | port         | disable/enable(0/1) | - n/a -         |          ✅          |
|                       |         |              |                     |                 |                      |
| start sysex           | 0xF0    |              |                     |                 |          ✅          |
| set pin mode(I/O)     | 0xF4    |              | pin # (0-127)       | pin mode        |          ✅          |
| set digital pin value | 0xF5    |              | pin # (0-127)       | pin value(0/1)  |          ✅          |
| sysex end             | 0xF7    |              |                     |                 |          ✅          |
| protocol version      | 0xF9    |              | major version       | minor version   |          ✅          |
| system reset          | 0xFF    |              |                     |                 |          ✅          |

Sysex-based sub-commands (0x00 - 0x7F) are used for an extended command set.

| type                  | sub-command | first byte       | second byte   | ...            | support             |
| --------------------- | -------     | ---------------  | ------------- | -------------- | --------------------|
| string                | 0x71        | char *string ... |               |                |          ✅          |
| firmware name/version | 0x79        | major version    | minor version | char *name ... |          ✅          |

### Firmata Balena SYSEX Commands

| type                  | sub-command | first byte       | second byte   | ...            |  support |
| --------------------- | ----------- | ---------------  | ------------- | -------------- |  ---------|
| power down            | 0xB0        | init_delay (uint8_t) | sleep_period[0] (uint8_t) |  sleep_period[8] (uint8_t) |  ✅          |

##### Power Down

This SYSEX command performs a hard power down of the CM3. In order to prevent loss of data or other hard shutdown consequences, users should set an `init_delay` period and gracefully power down the CM3 from the linux userspace, i.e. with `shutdown -h now`. After the `sleep_period` has expired, the coprocessor will resume power to the CM3 allowing it to boot into normal operating mode.

- `init_delay` is specified in seconds (passing 0 will immediate power down the CM3 and is not recommended!)
- `sleep_period` is specified in milliseconds (max value of `uint64_t`)

### Planned Features

- [ ] I2C Support
- [ ] Support for RTC control of Compute Module Power Rails
- [ ] SPI Support
- [ ] Custom Client Library for balenaFin features

