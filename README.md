# BalenaFin Co-Processor Firmata

This is an implementation of the [Firmata](https://github.com/firmata/protocol) protocol for Silicon Labs BGM111. It is compatible with standard Firmata 2.5.8. 

## Installation

### Dockerfile

The easiest way to install the Firmata application onto your board is to run the [balena application](https://github.com/balena-io/balena-fin-firmata-flash) provided. This targets the latest verision of the Balena Firmata application. 
This balena application will run and install [OpenOCD](http://openocd.org/) on your Fin in order to provision the Coprocessor with both a bootloader and the Firmata application.

We also provide a [Dockerfile](Dockerfile) for you to easily generate an environment for compiling this firmware yourself.

#### Build

To build the docker image, run:

```bash
docker build . --tag firmata 
```

A container can then be run with:

```bash
docker run -it -v ${YOUR-OUTPUT-DIRECTORY}:/out --name firmata-build firmata "make" "all"
```

> :wrench: * `all` can be substituted for `balena` if you only wish to target to balenaFin or `devkit` for the Silabs BRD4001A

This will output the build files to a directory specified by `YOUR-OUTPUT-DIRECTORY`, e.g. `~/Downloads/balena`.
The output binaries for flashing the coprocessor can be found under `${YOUR-OUTPUT-DIRECTORY}/builds/balena`.

If you wish to stop this container:

`docker stop firmata-build`

And to remove the container:

`docker rm firmata-build`

### Manually Build

It is also possible to build the source and manually flash the Coprocessor however, in order to flash the Coprocessor you will need to either load the compiled firmware onto the Compute Module and flash it using OpenOCD or program the Coprocessor using an external programmer such as a [Segger JLink ](https://www.segger.com/products/debug-probes/j-link/).

#### Dependencies

Before attempting to build, ensure the following dependencies are installed. 
We recommend ARM GCC version `9-2019-q4-major`, as used in our docker image.

 - cmake
 - make
 - arm-none-eabi-gcc

> :wrench: Make sure these are all located in your `$PATH`

#### Build

With the dependencies installed, build with the following commands:

1. `cd src` to change directory to the source files
2. `make setup` to generate the builds directory
3. `make balena` to execute the build*

> :wrench: * `make devkit` can be used to build for the Silabs BRD4001A

### Flashing

As the coprocessor is coupled to the compute module, we recommend that you use our flashing [example](https://github.com/balena-io/balena-fin-firmata-flash) to automate the flashing process.

#### Flashing via balenaFin Compute Module

If you wish to deploy this coprocessor firmware to a balenaFin running either [balenaOS](https://dashboard.balena-cloud.com) or [Raspbian](https://github.com/balena-os/pi-gen/releases), checkout this [example](https://github.com/balena-io/balena-fin-firmata-flash).

#### JLink Programmer

An external JLink programmer may be used to flash the coprocessor (for example, if using a Silabs Development Kit)

 - [JLink tools](https://www.segger.com/jlink-software.html)

To flash, using an external JLink programmer:

1. `cd builds/devkit && make flash` to flash to a device

## Firmata Protocol v2.5.8

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

| type                  | sub-command | first byte       | second byte     | ...             | support             |
| --------------------- | -------     | ---------------  | --------------- | --------------- | --------------------|
| string                | 0x71        | char *string ... |                 |                 |          ✅          |
| firmware name/version | 0x79        | major version    | minor version   | char *name ...  |          ✅          |
| I2C command           | 0x76        | See [I2C](#Firmata-I2C-SYSEX-Commands)    | See [I2C](#Firmata-I2C-SYSEX-Commands)   | See [I2C](#Firmata-I2C-SYSEX-Commands)  |          ✅          |
| balena subcommand     | 0x0B        | subcommand       | see subcommands | see subcommands |          ✅          |

### Firmata SYSEX I2C Commands

I2C Read/Write request and replies are implemented following the [official protocol](https://github.com/firmata/protocol/blob/master/i2c.md).

#### I2C Config

Optional `Delay` is for I2C devices that require a delay between when the register is written to and the data in that register can be read.

Optional `I2C Mode` allows for changing the I2C interface between the balenaFin's internal pins (`SCL 18` & `SDA 19`) and the external interface (`SCL 12` & `SDA 10`). By default I2C is set to the external interface, upon initialisation under Firmata. Setting the mode to `0` will toggle the internal interface, `1` will set it to the external interface.

```
0  START_SYSEX (0xF0)
1  I2C_CONFIG (0x78)
2  Delay in microseconds (LSB) [optional]
3  Delay in microseconds (MSB) [optional]
4  I2C Mode (LSB) [optional]
5  I2C Mode (MSB) [optional]
n  END_SYSEX (0xF7)
```

### Firmata SYSEX Balena Commands

Balena SYSEX subcommands are structured under the Firmata SYSEX command. 

For example, a balena subcommand to report balena firmata firmware would be represented as follows:

`[0xF0, 0x0B, 0x00, 0xF7]`

Which represents:

`[START_SYSEX, BALENA_SUBCOMMAND, BALENA_REPORT_FIRMWARE, END_SYSEX]`

| type                   | sub-command | first byte          | second byte             | ...                     | support |
| ---------------------- | ----------- | ------------------  | ----------------------- | ----------------------- | ------- |
| report balena firmware | 0x00        |          -          |            -            |             -           |  ✅     |
| power down             | 0x01        | uint8_t init_delay  | uint8_t sleep_period[0] | uint8_t sleep_period[3] |  ✅     |
| configure IDAC         | 0x02        | uint8_t idac_mode   |      uint8_t pin        | [see idac](#idac-write-current-0x02) |  ✅     |

#### Report balena Firmware [`0x00`]

While the SYSEX command `0x79` reports the protocol version of firmata, the SYSEX balena subcommand reports the specific version of firmware that the coprocessor is running (generated by the release tag). 

#### Power Down [`0x01`]

This SYSEX command performs a hard power down of the CM3. In order to prevent loss of data or other hard shutdown consequences, users should set an `init_delay` period and gracefully power down the CM3 from the linux userspace, i.e. with `shutdown -h now`. 
After the `sleep_period` has expired, the coprocessor will resume power to the CM3 allowing it to boot into normal operating mode.

- `init_delay` is composed of **1 byte**, specified in seconds (passing 0 will immediate power down the CM3 and is not recommended!)
- `sleep_period` is composed of **4 bytes**, specified in seconds (max value of (`uint32_t` / 1000), eqv. of ~4294967 seconds)

#### IDAC Write (Current) [`0x02`]

The BGM111 has an additional Current Digital to Analogue Converter (IDAC), which can be used as a current source/sink.
This is mapped to the balena subcommand `0x02` and can be controlled in a similar way as standard `Analogue Write` command at this endpoint.
The BGM111 only has 1 IDAC peripheral so can only be used on one pin at a time.
Changing pin will remove any previous pin configurations.

Each configuration much be **selected** with `0x0F` as the MSB of that option.

| feature                 | option | first byte   | second byte   | third byte                                     | response | Note                                   |
| ------------------------| ------ | -------------| ------------- | ---------------------------------------------- | -------- | -------------------------------------- |
| Set IDAC resolution     | 0      | `0x00`       | pin # (0-127) | `0x00` to `0x03` (defaults to `0x00`)          |     -    | Will return a `SYSEX` string on error  |
| Write IDAC Pin*         | 1      | `0x01`       | pin # (0-127) | `0 - 31` (Steps)                               |     -    | Will return a `SYSEX` string on error  |
| Reset IDAC              | 2      | `0x02`       |       -       |                -                               |     -    |                                        |

> :warning: * Not all pins have access to the IDAC, see [table](#Firmata-Pin-Map)

The IDAC can be set to 4 different ranges as shown in the table below:

| Range | Min | Max | Unit |   | Step Size | Unit |
|-------|-----|-----|------|---|-----------|------|
| `0x00`|0.05 | 1.6 | uA   |   |   50      |   nA |
| `0x01`| 1.6 | 4.7 | uA   |   |   100     |   nA |
| `0x02`| 0.5 | 16  | uA   |   |   500     |   nA |
| `0x03`|  2  | 64  | uA   |   |   2       |   uA |

For examples, if you wish to select set IDAC mode on pin 6 and subsequently write an output current 8uA, it would look as follows:

`[0xF0, 0x0B, 0x02, 0x0, 0x06, 0x02]`

`[0xF0, 0x0B, 0x02, 0x1, 0x06, 0x0F]`

Which represents:

`[START_SYSEX, BALENA_SUBCOMMAND, BALENA_IDAC_WRITE, SET_RESOLUTION, PIN, RESOLUTION_MODE, END_SYSEX]`

`[START_SYSEX, BALENA_SUBCOMMAND, BALENA_IDAC_WRITE, WRITE_STEP, PIN, STEP, END_SYSEX]`

##### IDAC to DAC

The IDAC can be mapped to the same functionality as a standard DAC with additional [circuitry](https://www.silabs.com/community/mcu/8-bit/knowledge-base.entry.html/2004/03/05/generating_voltageu-PlV1).

Alternatively, `PWM` can be used to generate a psuedo-analogue output voltage.

##### Resetting IDAC

Resetting the IDAC will disable and clear the configuration on any pin currently using it.

## Firmata Pin Map

This table refers to the Firmata standard pin mapping supported by the coprocessor.
The `BGM111` table column can be referenced in the [BGM111 Connector Pinout](https://github.com/balena-io/balena-fin/blob/master/documentation/markdown/balenaFin/v1.1/datasheet/datasheet.md#32-silicon-labs-bgm111-connector-pinout) section of the balenaFin datasheet can be used for the physical pin lookup on the balenaFin.

| Firmata Pin  | BGM111 | Function              | IDAC   | Note                                                     |
|--------------|--------|-----------------------|--------|----------------------------------------------------------|
| 0            | PD14   |                       |   ✅   |                                                          |
| 1            | PB13   | SPI_CS                |   ✅   |                                                          |
| 2            | PA2    |                       |   ✅   |                                                          |
| 3            | PC8    | SPI_CLK               |        |                                                          |
| 4            | PA3    |                       |   ✅   |                                                          |
| 5            | PC6    | SPI_MOSI              |        |                                                          |
| 6            | PA4    |                       |   ✅   |                                                          |
| 7            | PC7    | SPI_MISO              |        |                                                          |
| 8            | PA5    |                       |   ✅   |                                                          |
| 9            | PA1    |                       |   ✅   |                                                          |
| 10           | PB11   | I2C_SDA (external)    |   ✅   |                                                          |
| 11           | PA0    |                       |   ✅   |                                                          |
| 12           | PF6    | I2C_SCL (external)    |        |                                                          |
| 13           | PD15   |                       |   ✅   |                                                          |
| 14           | PF7    |                       |        |                                                          |
| 15           | PD13   |                       |   ✅   |                                                          |
| 16*          | PF5    | PW_ON_3V3             |        | balenaFin Power Rail (used for sleep mode)               |
| 17*          | PC9    | PW_ON_5V              |        |                                                          |
| 18*          | PC10   | I2C_SDA (internal)    |        | multi-master on Compute Module's I2C RTC and RGB LED     |
| 19*          | PC11   | I2C_SCL (internal)    |        |                                                          |

> :warning: * These pins are not accessible on the external balenaFin expansion header. Extra care should be taken when mapping Firmata pin 16 and 17 as these control the power rails of the balenaFin.

## Currently Unsupported

- [ ] SPI Support

