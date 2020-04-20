#include "balena.h"
#include "Firmata.h"
#include "Serial.h"

SerialClass Serial;

/* Balena Commands */
#define BALENA                          0x0B
#define BALENA_FIRMWARE                 0x00
#define BALENA_SLEEP                    0x01

#define DELAY_MULTIPLIER 1000 // base period (1) is milliseconds

#define BUFFERSIZE          256
#define I2C_WRITE                   0x00
#define I2C_READ                    0x08
#define I2C_READ_CONTINUOUSLY       0x10
#define I2C_STOP_READING            0x18
#define I2C_READ_WRITE_MODE_MASK    0x18
#define I2C_10BIT_ADDRESS_MODE_MASK 0x20
#define I2C_END_TX_MASK             0x40
#define I2C_STOP_TX                 1
#define I2C_RESTART_TX              0
#define I2C_MAX_QUERIES             8
#define I2C_REGISTER_NOT_SPECIFIED  -1

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL 1


/******************************************************************************
 * @brief  Global Variables
 *
 *****************************************************************************/
#ifdef FIRMATA_SERIAL_FEATURE
SerialFirmata serialFeature;
#endif

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis = 0;       // for comparison with currentMillis
unsigned int samplingInterval = 500; // how often to run the main loop (in ms)

/* i2c data */
struct i2c_device_info {
  byte addr;
  int reg;
  byte bytes;
  byte stopTX;
};

/* balena version */
char balena_version[sizeof(VERSION)] = VERSION;

/* for i2c read continuous more */
i2c_device_info query[I2C_MAX_QUERIES];

// byte i2cDATA[64];
uint8_t i2cCMD[I2C_TXBUFFER_SIZE];
uint8_t i2cDATA[I2C_RXBUFFER_SIZE];
bool isI2CEnabled = false;
signed char queryIndex = -1;
// default delay time between i2c read request and Wire.requestFrom()
unsigned int i2cReadDelayTime = 0;

bool isResetting = false;

// Forward declare a few functions to avoid compiler errors with older versions
// of the Arduino IDE.
void setPinModeCallback(byte, int);
void reportAnalogCallback(byte analogPin, int value);
void sysexCallback(byte, byte, byte*);
void powerOn(RTCDRV_TimerID_t id, void * user);

/******************************************************************************
 * @brief  User Callbacks
 *
 *****************************************************************************/
struct power
{
  uint32_t sleep_delay;
  uint32_t sleep_period;
  bool state;
} power_struct;

void powerOn(RTCDRV_TimerID_t id, void * user)
{
  if(power_struct.state == false){
    digitalWrite(SLEEP_PIN, 0);
    power_struct.state = true;
    RTCDRV_StopTimer(id);
    RTCDRV_StartTimer(id, rtcdrvTimerTypeOneshot, power_struct.sleep_period, powerOn, NULL);
  }
  else {
    digitalWrite(SLEEP_PIN, 1);
  }
}


/******************************************************************************
 * @brief  Firmata Functions
 *
 *****************************************************************************/

void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing
  // Arduino.h to get SCL and SDA pins
  for (i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, PIN_MODE_I2C);
    }
  }

  isI2CEnabled = true;

  // Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
}

void readAndReportData(byte address, int theRegister, byte numBytes, byte stopTX) {
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()

  i2cCMD[0] = theRegister;
  if (theRegister != I2C_REGISTER_NOT_SPECIFIED) {

    transferI2C((u_int16_t) address, i2cCMD, i2cDATA, 1, numBytes, I2C_FLAG_WRITE_READ);

    if (i2cReadDelayTime > 0) {
      // delay is necessary for some devices such as WiiNunchuck
      delay(i2cReadDelayTime);
    }
  } else {
    i2cCMD[0] = 0;  // fill the register with a dummy value
  }

  transferI2C((u_int16_t) address, i2cCMD, i2cDATA, 1, numBytes, I2C_FLAG_WRITE_READ);

  uint8_t payload[numBytes];

  payload[0] = address >> 1;
  payload[1] = theRegister;

  for (int i = 0; i < numBytes; i++) {
    payload[2 + i] = i2cDATA[i];
  }

  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, payload);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];

  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

void setPinModeCallback(byte pin, int mode)
{
  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == PIN_MODE_PULLUP) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  Firmata.setPinState(pin, 0);
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), GPIO_INPUT_PULLUP, 0);    // disable output driver
        }
        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
		if(pin == 7)
        pinMode(PIN_TO_DIGITAL(pin), GPIO_INPUT, 0);    // disable output driver
        Firmata.setPinMode(pin, GPIO_INPUT);
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), GPIO_INPUT_PULLUP, 1);
        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        if (Firmata.getPinMode(pin) == PIN_MODE_PWM) {
          // Disable PWM if pin mode was previously set to PWM.
          resetPWM(pin);
          digitalWrite(PIN_TO_DIGITAL(pin), LOW);
        }
        pinMode(PIN_TO_DIGITAL(pin), GPIO_OUTPUT, 0);
        Firmata.setPinMode(pin, OUTPUT);
      }
      break;
    case PIN_MODE_PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), GPIO_OUTPUT, 0);
        Firmata.setPinMode(pin, PIN_MODE_PWM);
        analogWrite(PIN_TO_PWM(pin), 0);
      }
      break;
    case PIN_MODE_I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        Firmata.setPinMode(pin, PIN_MODE_I2C);
      }
      break;
    case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handlePinMode(pin, PIN_MODE_SERIAL);
#endif
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void setPinValueCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (Firmata.getPinMode(pin) == OUTPUT) {
      Firmata.setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}

void reportAnalogCallback(byte analogPin, int value)
{

  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
      }
    }
  }
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch (Firmata.getPinMode(pin)) {
//      case PIN_MODE_SERVO:
//        if (IS_PIN_DIGITAL(pin))
//          servos[servoPinMap[pin]].write(value);
//        Firmata.setPinState(pin, value);
//        break;
      case PIN_MODE_PWM:
        if (IS_PIN_PWM(pin)){
			analogWrite(PIN_TO_PWM(pin), value);
			Firmata.setPinState(pin, value);
        }
        break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, pinValue, mask = 1, pinWriteMask = 0;
  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (Firmata.getPinMode(pin) == OUTPUT || Firmata.getPinMode(pin) == INPUT) {
          pinValue = ((byte)value & mask) ? 1 : 0;

          if (Firmata.getPinMode(pin) == OUTPUT) {
            pinWriteMask |= mask;
          }
          else if (Firmata.getPinMode(pin) == INPUT && pinValue == 1 && Firmata.getPinState(pin) != 1) {
            pinMode(pin, GPIO_INPUT_PULLUP, 1);
          }
          Firmata.setPinState(pin, pinValue);
          digitalWrite((pin), pinValue);
        }
      }
      mask = mask << 1;
    }
  }
}

/******************************************************************************
 * @brief  Sysex Commands
 *
 *****************************************************************************/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte stopTX;
  byte slaveAddress;
  byte data;
  byte i2c_mode;
  int slaveRegister;
  unsigned int delayTime;

  switch (command) {
  	// TODO: Add I2C
    case I2C_REQUEST:
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
        Firmata.sendString("10-bit addressing not supported");
        return;
      }
      else {
        slaveAddress = argv[0] << 1;
      }

      // need to invert the logic here since 0 will be default for client
      // libraries that have not updated to add support for restart tx
      if (argv[1] & I2C_END_TX_MASK) {
        stopTX = I2C_RESTART_TX;
      }
      else {
        stopTX = I2C_STOP_TX; // default
      }

      switch (mode) {
        case I2C_WRITE:
          for (byte i = 0; i < ((argc-4)/2); i++) {
            i2cDATA[i] = argv[(2*i)+4] + (argv[((2*i)+4)+1] << 7);
          }
          i2cCMD[0] = argv[2] + (argv[3] << 7);
          transferI2C((u_int16_t) slaveAddress, i2cCMD , i2cDATA,  1, ((argc-4)/2), I2C_FLAG_WRITE_WRITE);
          delay(10);
          break;
        case I2C_READ:
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          readAndReportData(slaveAddress, (int)slaveRegister, data, stopTX);
          break;
        case I2C_READ_CONTINUOUSLY:
          if ((queryIndex + 1) >= I2C_MAX_QUERIES) {
            // too many queries, just ignore
            Firmata.sendString("too many queries");
            break;
          }
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            i2cDATA[0] = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = (int)I2C_REGISTER_NOT_SPECIFIED;
            i2cDATA[0] = argv[2] + (argv[3] << 7);  // bytes to read
          }
          queryIndex++;
          query[queryIndex].addr = slaveAddress;
          query[queryIndex].reg = slaveRegister;
          query[queryIndex].bytes = i2cDATA[0];
          query[queryIndex].stopTX = stopTX;
          break;
        case I2C_STOP_READING:
          byte queryIndexToSkip;
          // if read continuous mode is enabled for only 1 i2c device, disable
          // read continuous reporting for that device
          if (queryIndex <= 0) {
            queryIndex = -1;
          } else {
            queryIndexToSkip = 0;
            // if read continuous mode is enabled for multiple devices,
            // determine which device to stop reading and remove it's data from
            // the array, shifiting other array data to fill the space
            for (byte i = 0; i < queryIndex + 1; i++) {
              if (query[i].addr == slaveAddress) {
                queryIndexToSkip = i;
                break;
              }
            }

            for (byte i = queryIndexToSkip; i < queryIndex + 1; i++) {
              if (i < I2C_MAX_QUERIES) {
                query[i].addr = query[i + 1].addr;
                query[i].reg = query[i + 1].reg;
                query[i].bytes = query[i + 1].bytes;
                query[i].stopTX = query[i + 1].stopTX;
              }
            }
            queryIndex--;
          }
          break;
        default:
          break;
      }
      break;
    case I2C_CONFIG:
      delayTime = (argv[0] + (argv[1] << 7));

      if (argc > 1 && delayTime > 0) {
        i2cReadDelayTime = delayTime;
      }

      if(argc > 2){
        i2c_mode = (argv[2] + (argv[3] << 7));
        deinitI2C();
        initI2C(i2c_mode);
      }
      else {
        initI2C(1); // default to external
      }

      if (!isI2CEnabled) {
        enableI2CPins();
      }

      break;
    // TODO: ADD Servo
    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      } else {
        Firmata.sendString("Not enough data");
      }
      break;
    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;
    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (port_pin[pin].pin < TOTAL_PINS) {
          Firmata.write((byte)INPUT);
          Firmata.write(1);
          Firmata.write((byte)PIN_MODE_PULLUP);
          Firmata.write(1);
          Firmata.write((byte)OUTPUT);
          Firmata.write(1);
        }
        if (port_pin[pin].adc != MODE_NONE) {
          Firmata.write(PIN_MODE_ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }
        if (port_pin[pin].pwm_0 != PWM_NONE) {
          Firmata.write(PIN_MODE_PWM);
          Firmata.write(DEFAULT_PWM_RESOLUTION);
        }
        if (port_pin[pin].pin < TOTAL_PINS) {
          Firmata.write(PIN_MODE_SERVO);
          Firmata.write(14);
        }
        if (IS_PIN_I2C(pin)) {
          Firmata.write(PIN_MODE_I2C);
          Firmata.write(1);
        }
#ifdef FIRMATA_SERIAL_FEATURE
        serialFeature.handleCapability(pin);
#endif
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write(Firmata.getPinMode(pin));
          Firmata.write((byte)Firmata.getPinState(pin) & 0x7F);
          if (Firmata.getPinState(pin) & 0xFF80) Firmata.write((byte)(Firmata.getPinState(pin) >> 7) & 0x7F);
          if (Firmata.getPinState(pin) & 0xC000) Firmata.write((byte)(Firmata.getPinState(pin) >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.write(END_SYSEX);
      break;
    case BALENA:
      switch (argv[0]) {
        case BALENA_FIRMWARE:
          Firmata.sendSysex(BALENA, sizeof(balena_version) - 1,(byte *) balena_version);
          break;
        case BALENA_SLEEP:
          if (argc > 5) {
            power_struct.sleep_delay = argv[1] * (uint32_t) DELAY_MULTIPLIER; // in seconds
            power_struct.sleep_period = (((argv[5] << 21) & 0x01) | \
                                        (argv[4] << 14) | \
                                        (argv[3] << 7) | \
                                        (argv[2])) * (uint32_t) DELAY_MULTIPLIER; // in seconds
            if(argv[1] == 0){ // without delayed start
              digitalWrite(SLEEP_PIN, 0);
              power_struct.state = true;
              RTCDRV_StartTimer(id, rtcdrvTimerTypeOneshot, power_struct.sleep_period, powerOn, NULL);
            }
            else { // with delayed start
              digitalWrite(SLEEP_PIN, 1);
              power_struct.state = false;
              RTCDRV_StartTimer(id, rtcdrvTimerTypeOneshot, power_struct.sleep_delay, powerOn, NULL);
            }
          }
          break;
        default:
          break;
      }
      break;
    case SERIAL_MESSAGE:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handleSysex(command, argc, argv);
#endif
      break;
    default:
      break;
  }
}

void systemResetCallback()
{
  isResetting = true;
  // initialize a default state
  // TODO: option to load config from EEPROM instead of default

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.reset();
#endif

//  if (isI2CEnabled) {
//    disableI2CPins();
//  }

  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, PIN_MODE_ANALOG);
    } else if (IS_PIN_DIGITAL(i)) {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }

//    servoPinMap[i] = 255;
  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;

//  detachedServoCount = 0;
//  servoCount = 0;
  /* send digital inputs to set the initial state on the host computer,
   * since once in the loop(), this firmware will only send on change */
  /*
  TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
  for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
  }
  */
  isResetting = false;
  // TODO: Change to graceful reset
  reset();
}

void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

/******************************************************************************
 * @brief  Main function
 *
 *****************************************************************************/
int main(void)
{

	Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);
	Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
	Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
	Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
	Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
	Firmata.attach(SET_PIN_MODE, setPinModeCallback);
	Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
	Firmata.attach(START_SYSEX, sysexCallback);
	Firmata.attach(SYSTEM_RESET, systemResetCallback);

	balenaInit();



	Serial.begin(57600);

	Firmata.begin(Serial);


	while(true){
		 byte pin, analogPin;

		  /* DIGITALREAD - as fast as possible, check for changes and output them to the
		   * UART buffer using Serial.write()  */
		  checkDigitalInputs();

		  /* STREAMREAD - processing incoming messages as soon as possible, while still
		   * checking digital inputs.  */
		  while (Firmata.available() > 0) {
		    Firmata.processInput();
		  }

		  // TODO - ensure that Stream buffer doesn't go over 60 bytes
		  currentMillis = millis();
		  if (currentMillis - previousMillis > samplingInterval) {
		    previousMillis += samplingInterval;
		  /* ANALOGREAD - do all analogReads() at the configured sampling interval */
		    for (pin = 0; pin < TOTAL_PINS; pin++) {
		      if (IS_PIN_ANALOG(pin) && Firmata.getPinMode(pin) == PIN_MODE_ANALOG) {
		        analogPin = PIN_TO_ANALOG(pin);

		        if (analogInputsToReport & (1 << analogPin)) {

		          Firmata.sendAnalog(analogPin, analogRead(analogPin));
		        }
		      }
		    }
		    // report i2c data for all device with read continuous mode enabled
		    if (queryIndex > -1) {
		      for (byte i = 0; i < queryIndex + 1; i++) {
		        readAndReportData(query[i].addr, query[i].reg, query[i].bytes, query[i].stopTX);
		      }
		    }
		  }
	}
}
