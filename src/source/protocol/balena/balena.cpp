#include "balena.h"

/******************************************************************************
 * @brief  Balena Definitions
 *
 *****************************************************************************/

/* ADC */
ADC_Init_TypeDef ADCInit = ADC_INIT_DEFAULT;
ADC_InitSingle_TypeDef ADCSingle = ADC_INITSINGLE_DEFAULT;
ADC_PosSel_TypeDef *ADCNone;
bool ADCset = false;

/* IDAC */
IDAC_Init_TypeDef IDACInit = IDAC_INIT_DEFAULT;
IDAC_OutMode_TypeDef *IDACNone;

/* RTC */
RTCDRV_TimerID_t id;
RTCDRV_TimerID_t wall;

/* PWM */
TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
TIMER_INDEX timer_index[TIMER_CHANNELS] = {
    {PWM_NONE, TIMER_NONE},
    {PWM_NONE, TIMER_NONE},
    {PWM_NONE, TIMER_NONE},
    {PWM_NONE, TIMER_NONE}};
byte TIMER_FREE_CHANNEL = 0;
byte channel = 0;
uint32_t duty[TIMER_CHANNELS];

/* I2C */
// Using default settings
I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
// Transmission flags
volatile bool i2c_rxInProgress;
volatile bool i2c_startTx;
uint8_t i2c_rxBuffer[I2C_RXBUFFER_SIZE];
uint8_t i2c_rxBufferIndex;

/* Pin State */

/******************************************************************************
 * @brief  Pin Definitions
 *
 *****************************************************************************/

IDAC_Range_TypeDef idac_lookup[4] = {
    idacCurrentRange0,
    idacCurrentRange1,
    idacCurrentRange2,
    idacCurrentRange3};

ADC_LOOKUP adc_pin[NUM_PINS]{
    {adcPosSelAPORT3XCH6},  // P0
    {adcPosSelAPORT3YCH29}, // P1
    {adcPosSelAPORT4YCH10}, // P2
    {adcPosSelAPORT2YCH8},  // P3
    {adcPosSelAPORT3YCH11}, // P4
    {adcPosSelAPORT1XCH6},  // P5
    {adcPosSelAPORT4YCH12}, // P6
    {adcPosSelAPORT2XCH7},  // P7
    {adcPosSelAPORT3YCH13}, // P8
    {adcPosSelAPORT3YCH9},  // P9
    {adcPosSelAPORT3YCH27}, // P10
    {adcPosSelAPORT3XCH8},  // P11
    {adcPosSelAPORT2YCH22}, // P12
    {adcPosSelAPORT3YCH7},  // P13
    {adcPosSelAPORT2XCH23}, // P14
    {adcPosSelAPORT3YCH5},  // P15
};

IDAC_LOOKUP idac_pin[NUM_PINS]{
    {idacOutputAPORT1XCH6},  // P0
    {idacOutputAPORT1YCH29}, // P1
    {idacOutputAPORT1XCH10}, // P2
    {idacOutputAPORT1YCH11}, // P4
    {idacOutputAPORT1XCH12}, // P6
    {idacOutputAPORT1YCH13}, // P8
    {idacOutputAPORT1YCH9},  // P9
    {idacOutputAPORT1YCH27}, // P10
    {idacOutputAPORT1XCH8},  // P11
    {idacOutputAPORT1YCH7},  // P13
    {idacOutputAPORT1YCH5}   // P15
};

IDAC_RANGE_LOOKUP idac_range[NUM_PINS]{
    {idacCurrentRange0}, // P0
    {idacCurrentRange0}, // P1
    {idacCurrentRange0}, // P2
    {idacCurrentRange0}, // P4
    {idacCurrentRange0}, // P6
    {idacCurrentRange0}, // P8
    {idacCurrentRange0}, // P9
    {idacCurrentRange0}, // P10
    {idacCurrentRange0}, // P11
    {idacCurrentRange0}, // P13
    {idacCurrentRange0}  // P15
};

/* PORT | PIN | ADC | IDAC | MODE | PWM_0 | PWM_1 | PWM_2 | PWM_3 */
PIN_MAP port_pin[NUM_PINS] = {
    {gpioPortD, 14, 0, 0, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC22, TIMER_ROUTELOC0_CC1LOC_LOC21, TIMER_ROUTELOC0_CC2LOC_LOC20, TIMER_ROUTELOC0_CC3LOC_LOC19},         // P0
    {gpioPortB, 13, 1, 1, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC8, TIMER_ROUTELOC0_CC1LOC_LOC7, TIMER_ROUTELOC0_CC2LOC_LOC6, TIMER_ROUTELOC0_CC3LOC_LOC5},             // P1 (SPI CS)
    {gpioPortA, 2, 2, 2, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC2, TIMER_ROUTELOC0_CC1LOC_LOC1, TIMER_ROUTELOC0_CC2LOC_LOC0, TIMER_ROUTELOC0_CC3LOC_LOC31},             // P2
    {gpioPortC, 8, 3, MODE_NONE, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC13, TIMER_ROUTELOC0_CC1LOC_LOC12, TIMER_ROUTELOC0_CC2LOC_LOC11, TIMER_ROUTELOC0_CC3LOC_LOC10},  // P3 (SPI CLK)
    {gpioPortA, 3, 4, 3, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC3, TIMER_ROUTELOC0_CC1LOC_LOC2, TIMER_ROUTELOC0_CC2LOC_LOC1, TIMER_ROUTELOC0_CC3LOC_LOC0},              // P4
    {gpioPortC, 6, 5, MODE_NONE, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC11, TIMER_ROUTELOC0_CC1LOC_LOC10, TIMER_ROUTELOC0_CC2LOC_LOC9, TIMER_ROUTELOC0_CC3LOC_LOC8},    // P5 (SPI MOSI)
    {gpioPortA, 4, 6, 4, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC4, TIMER_ROUTELOC0_CC1LOC_LOC3, TIMER_ROUTELOC0_CC2LOC_LOC2, TIMER_ROUTELOC0_CC3LOC_LOC1},              // P6
    {gpioPortC, 7, 7, MODE_NONE, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC12, TIMER_ROUTELOC0_CC1LOC_LOC11, TIMER_ROUTELOC0_CC2LOC_LOC10, TIMER_ROUTELOC0_CC3LOC_LOC9},   // P7 (SPI MISO)
    {gpioPortA, 5, 8, 5, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC5, TIMER_ROUTELOC0_CC1LOC_LOC4, TIMER_ROUTELOC0_CC2LOC_LOC3, TIMER_ROUTELOC0_CC3LOC_LOC2},              // P8
    {gpioPortA, 1, 9, 6, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC1, TIMER_ROUTELOC0_CC1LOC_LOC0, TIMER_ROUTELOC0_CC2LOC_LOC31, TIMER_ROUTELOC0_CC3LOC_LOC30},            // P9
    {gpioPortB, 11, 10, 7, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC6, TIMER_ROUTELOC0_CC1LOC_LOC5, TIMER_ROUTELOC0_CC2LOC_LOC4, TIMER_ROUTELOC0_CC3LOC_LOC3},            // P10
    {gpioPortA, 0, 11, 8, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC0, TIMER_ROUTELOC0_CC1LOC_LOC31, TIMER_ROUTELOC0_CC2LOC_LOC30, TIMER_ROUTELOC0_CC3LOC_LOC29},          // P11
    {gpioPortF, 6, 12, MODE_NONE, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC30, TIMER_ROUTELOC0_CC1LOC_LOC29, TIMER_ROUTELOC0_CC2LOC_LOC28, TIMER_ROUTELOC0_CC3LOC_LOC27}, // P12 (Dev Kit LED0)
    {gpioPortD, 15, 13, 9, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC23, TIMER_ROUTELOC0_CC1LOC_LOC22, TIMER_ROUTELOC0_CC2LOC_LOC21, TIMER_ROUTELOC0_CC3LOC_LOC20},        // P13
    {gpioPortF, 7, 14, MODE_NONE, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC31, TIMER_ROUTELOC0_CC1LOC_LOC30, TIMER_ROUTELOC0_CC2LOC_LOC29, TIMER_ROUTELOC0_CC3LOC_LOC28}, // P14 (Dev Kit LED1)
    {gpioPortD, 13, 15, 10, MODE_NONE, TIMER_ROUTELOC0_CC0LOC_LOC21, TIMER_ROUTELOC0_CC1LOC_LOC20, TIMER_ROUTELOC0_CC2LOC_LOC19, TIMER_ROUTELOC0_CC3LOC_LOC18},       // P15
    {gpioPortF, 5, MODE_NONE, MODE_NONE, MODE_NONE, PWM_NONE, PWM_NONE, PWM_NONE, PWM_NONE},                                                                          // P16 (PW_ON_3V3)
    {gpioPortC, 9, MODE_NONE, MODE_NONE, MODE_NONE, PWM_NONE, PWM_NONE, PWM_NONE, PWM_NONE},                                                                          // P17 (PW_ON_5V)
    {gpioPortC, 10, MODE_NONE, MODE_NONE, MODE_NONE, PWM_NONE, PWM_NONE, PWM_NONE, PWM_NONE},                                                                         // P18 (I2C SDA)
    {gpioPortC, 11, MODE_NONE, MODE_NONE, MODE_NONE, PWM_NONE, PWM_NONE, PWM_NONE, PWM_NONE}                                                                          // P19 (I2C SCL)
};

/******************************************************************************
 * @brief  Balena Functions
 *
 *****************************************************************************/

void balenaInit()
{
  initMCU();
  initGPIO();
  initTimer();
  initADC();
  initPWM();
  initI2C(1);
};

void reset()
{
  NVIC_SystemReset();
};

/******************************************************************************
 * @brief  System Functions
 *
 *****************************************************************************/

void initMCU()
{
  // Device errata
  CHIP_Init();
  // Set up DC-DC converter
  EMU_DCDCInit_TypeDef dcdcInit = BSP_DCDC_INIT;
#if HAL_DCDC_BYPASS
  dcdcInit.dcdcMode = emuDcdcMode_Bypass;
#endif
  EMU_DCDCInit(&dcdcInit);
  // Set up clocks
  initMCU_CLK();
  RTCC_Init_TypeDef rtccInit = RTCC_INIT_DEFAULT;
  rtccInit.enable = true;
  rtccInit.debugRun = false;
  rtccInit.precntWrapOnCCV0 = false;
  rtccInit.cntWrapOnCCV1 = false;
  rtccInit.prescMode = rtccCntTickPresc;
  rtccInit.presc = rtccCntPresc_1;
  rtccInit.enaOSCFailDetect = false;
  rtccInit.cntMode = rtccCntModeNormal;
  RTCC_Init(&rtccInit);

#if defined(_EMU_CMD_EM01VSCALE0_MASK)
  // Set up EM0, EM1 energy mode configuration
  EMU_EM01Init_TypeDef em01Init = EMU_EM01INIT_DEFAULT;
  EMU_EM01Init(&em01Init);
#endif // _EMU_CMD_EM01VSCALE0_MASK

#if defined(_EMU_CTRL_EM23VSCALE_MASK)
  // Set up EM2, EM3 energy mode configuration
  EMU_EM23Init_TypeDef em23init = EMU_EM23INIT_DEFAULT;
  em23init.vScaleEM23Voltage = emuVScaleEM23_LowPower;
  EMU_EM23Init(&em23init);
#endif //_EMU_CTRL_EM23VSCALE_MASK

  TEMPDRV_Init();
}

void initMCU_CLK(void)
{
  // Initialize HFXO
  CMU_HFXOInit_TypeDef hfxoInit = BSP_CLK_HFXO_INIT;
  // if Factory Cal exists in DEVINFO then use it
  if (0 == (DEVINFO->MODULEINFO & DEVINFO_MODULEINFO_HFXOCALVAL_MASK))
  {
    hfxoInit.ctuneSteadyState = DEVINFO_MODULEINFO_CRYSTALOSCCALVAL & DEVINFO_HFXOCTUNE_MASK;
  }
  CMU_HFXOInit(&hfxoInit);

  // Set system HFXO frequency
  SystemHFXOClockSet(BSP_CLK_HFXO_FREQ);

  // Enable HFXO oscillator, and wait for it to be stable
  CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

  // Enable HFXO Autostart only if EM2 voltage scaling is disabled.
  // In 1.0 V mode the chip does not support frequencies > 21 MHz,
  // this is why HFXO autostart is not supported in this case.
#if !defined(_EMU_CTRL_EM23VSCALE_MASK)
  // Automatically start and select HFXO
  CMU_HFXOAutostartEnable(0, true, true);
#else
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
#endif //_EMU_CTRL_EM23VSCALE_MASK

  // HFRCO not needed when using HFXO
  CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

  // Enabling HFBUSCLKLE clock for LE peripherals
  CMU_ClockEnable(cmuClock_HFLE, true);

  // Initialize LFXO
  CMU_LFXOInit_TypeDef lfxoInit = BSP_CLK_LFXO_INIT;
  lfxoInit.ctune = BSP_CLK_LFXO_CTUNE;
  CMU_LFXOInit(&lfxoInit);

  // Set system LFXO frequency
  SystemLFXOClockSet(BSP_CLK_LFXO_FREQ);

  // Set LFXO if selected as LFCLK
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
  CMU_ClockSelectSet(cmuClock_LFE, cmuSelect_LFXO);
}

/******************************************************************************
 * @brief  ADC Functions
 *
 *****************************************************************************/

void initADC()
{
  CMU_ClockEnable(cmuClock_ADC0, true);

  ADCInit.prescale = ADC_PrescaleCalc(adcFreq, 0);
  ADCSingle.diff = false;             // single ended
  ADCSingle.reference = adcRef2V5;    // internal 2.5V reference
  ADCSingle.resolution = adcRes12Bit; // 12-bit resolution
  ADCSingle.acqTime = adcAcqTime4;    // set acquisition time to meet minimum requirement
};

int setADC(unsigned int pin_no, ADC_TypeDef *adc)
{
  int adc_config = port_pin[pin_no].adc;
  if (adc_config == MODE_NONE)
    return BALENA_ERR;
  ADCSingle.posSel = adc_pin[adc_config].adc;
  port_pin[pin_no].state = MODE_ANALOG_IN;
  ADC_Init(adc, &ADCInit);
  ADC_InitSingle(adc, &ADCSingle);
  return BALENA_SUCCESS;
};

/******************************************************************************
 * @brief  PWM Functions
 *
 *****************************************************************************/
void TIMER1_IRQHandler(void)
{
  // Get pending channel flags and clear
  for (byte i = 0; i < 4; i++)
    TIMER_IntClear(TIMER1, TIMER_IF_CC0 << i);
  for (byte i = 0; i < 4; i++)
    TIMER_CompareBufSet(TIMER1, i, (TIMER_TopGet(TIMER1) * duty[i]) / 100);
}

void initPWM()
{
  CMU_ClockEnable(cmuClock_TIMER1, true);
  timerCCInit.mode = timerCCModePWM;
  timerCCInit.cmoa = timerOutputActionToggle;
  TIMER_InitCC(TIMER1, 0, &timerCCInit);
  TIMER_InitCC(TIMER1, 1, &timerCCInit);
  TIMER_InitCC(TIMER1, 2, &timerCCInit);
};

void resetPWM(byte pin)
{
  int chan = -1;
  for (byte i = 0; i < 4; i++)
  {
    if (timer_index[i].pin == pin)
      chan = i;
  }
  if (chan == -1)
    return;
  timer_index[chan].route = TIMER_NONE;
  timer_index[chan].pin = PWM_NONE;
  TIMER1->ROUTELOC0 = RESET;
  for (byte i = 0; i < 4; i++)
  {
    if (timer_index[i].route != TIMER_NONE)
      TIMER1->ROUTELOC0 |= timer_index[i].route;
  }
  TIMER_IntClear(TIMER1, (TIMER_IF_CC0 << chan));
  TIMER_IntDisable(TIMER1, (TIMER_IF_CC0 << chan));
}

void incChannel(unsigned int pin_no, byte chan)
{
  switch (chan)
  {
  case 0:
    timer_index[chan].route = port_pin[pin_no].pwm_0;
    break;
  case 1:
    timer_index[chan].route = port_pin[pin_no].pwm_1;
    break;
  case 2:
    timer_index[chan].route = port_pin[pin_no].pwm_2;
    break;
  case 3:
    timer_index[chan].route = port_pin[pin_no].pwm_3;
    break;
  default:
    timer_index[chan].route = port_pin[pin_no].pwm_0;
    break;
  }
  timer_index[chan].pin = pin_no;
};

int setPWM(unsigned int pin_no, byte duty_cycle)
{
  TIMER1->ROUTELOC0 = RESET;
  uint32_t channel_route = channel;
  if (port_pin[pin_no].pwm_0 != PWM_NONE)
  {
    for (byte i = 0; i < 4; i++)
    {
      if (timer_index[i].pin == pin_no)
        channel_route = i;
    }
    duty[channel_route] = duty_cycle;

    incChannel(pin_no, channel_route);

    for (byte i = 0; i < 4; i++)
    {
      if (timer_index[i].route != TIMER_NONE)
        TIMER1->ROUTELOC0 |= timer_index[i].route;
    }
  }
  else
  {
    return BALENA_ERR;
  }

  // Enable route on current channel
  TIMER1->ROUTEPEN |= (TIMER_ROUTEPEN_CC0PEN << channel_route);

  // Set top value to overflow once per signal period
  TIMER_TopSet(TIMER1, CMU_ClockFreqGet(cmuClock_HFPER) / PWM_FREQ);

  // Set compare value for initial duty cycle
  TIMER_CompareSet(TIMER1, channel_route, (TIMER_TopGet(TIMER1) * duty[channel_route]) / 100);

  // Initialize and start timer with no prescaling
  timerInit.prescale = timerPrescale1;
  TIMER_Init(TIMER1, &timerInit);

  // Safely enable TIMER0 CC0 interrupt
  TIMER_IntClear(TIMER1, (TIMER_IF_CC0 << channel_route));
  NVIC_ClearPendingIRQ(TIMER1_IRQn);

  // Interrupt on compare event to set CCVB to update duty cycle on next period
  TIMER_IntEnable(TIMER1, (TIMER_IF_CC0 << channel));

  NVIC_EnableIRQ(TIMER1_IRQn);
  if (channel_route == channel)
  {
    channel = (channel + 1) % 4;
  }
  return BALENA_SUCCESS;
};

/******************************************************************************
 * @brief  IDAC Functions
 *
 *****************************************************************************/

void initIDAC()
{
  CMU_ClockEnable(cmuClock_IDAC0, true);

  IDAC_Init(IDAC0, &IDACInit);

  // Choose the output current to be 2 microamps
  IDAC_RangeSet(IDAC0, idacCurrentRange1);
  IDAC_StepSet(IDAC0, 4);
  IDAC_Enable(IDAC0, true);
}

int setIDAC(unsigned int pin_no, byte range)
{
  int idac_config = port_pin[pin_no].idac;
  if (idac_config == MODE_NONE)
    return BALENA_ERR;
  idac_range[port_pin[pin_no].idac].idac_range = idac_lookup[range];
  return BALENA_SUCCESS;
}

void resetIDAC()
{
  IDAC_Enable(IDAC0, false);
  IDAC_Reset(IDAC0);
}

int writeIDAC(unsigned int pin_no, byte step)
{
  int idac_config = port_pin[pin_no].idac;
  if (idac_config == MODE_NONE)
    return BALENA_ERR;
  IDACInit.outMode = idac_pin[idac_config].idac; // Choose output to be on PB8
  // Choose the output current to be 2 microamps
  IDAC_RangeSet(IDAC0, idac_range[port_pin[pin_no].idac].idac_range);
  IDAC_StepSet(IDAC0, step);

  IDAC_Init(IDAC0, &IDACInit);

  // Enable IDAC output mode and also enable the IDAC module itself
  IDAC_OutEnable(IDAC0, true);
  return BALENA_SUCCESS;
};

/******************************************************************************
 * @brief  Arduino-Firmata Functions
 *
 *****************************************************************************/

void serialMode(bool enable)
{
  GPIO_PinModeSet(DEVKIT_VCOM_ENABLE_PORT, DEVKIT_VCOM_ENABLE_PIN, gpioModePushPull, enable);
};

void pinMode(unsigned int pin_no, GPIO_Mode_TypeDef mode, unsigned int direction)
{
  GPIO_PinModeSet(port_pin[pin_no].port, port_pin[pin_no].pin, mode, direction);
};

bool pinExists(unsigned int pin_no)
{
  if (pin_no <= ((sizeof(port_pin) / sizeof(PIN_MAP))))
  {
    return false;
  }
  else
  {
    return true;
  };
};

/******************************************************************************
 * @brief  Digital I/O Functions
 *
 *****************************************************************************/

void initGPIO()
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
  EMU_DCDCInit(&dcdcInit);
};

unsigned int digitalRead(unsigned int pin_no)
{
  return GPIO_PinInGet(port_pin[pin_no].port, port_pin[pin_no].pin);
};

void digitalWrite(unsigned int pin_no, unsigned int value)
{
  GPIO_PinModeSet(port_pin[pin_no].port, port_pin[pin_no].pin, gpioModePushPull, value);
};

/******************************************************************************
 * @brief  Analogue I/O Functions
 *
 *****************************************************************************/

void analogWrite(unsigned int pin_no, byte value)
{
  // Handles converting from Arduino-style analogWrite to percentage based duty cycle
  port_pin[pin_no].state = MODE_ANALOG_OUT;
  if (port_pin[pin_no].idac != MODE_NONE)
  {

    if (port_pin[pin_no].state == MODE_PWM)
    {
      value = map(value, 0, 255, 0, 100);
      setPWM(pin_no, value);
    }
    else if (port_pin[pin_no].state == MODE_ANALOG_OUT)
    {
      digitalWrite(14, 0);
      value = map(value, 0, 255, 0, 31);
      setIDAC(pin_no, value);
    }
  }
  else
  {
    if (port_pin[pin_no].state == MODE_PWM)
    {

      setPWM(pin_no, value);
    }
  }
};

uint32_t analogRead(unsigned int pin_no)
{
  if (port_pin[pin_no].state != MODE_ANALOG_IN)
  {
    setADC(pin_no, ADC0);
  }
  ADC_Start(ADC0, adcStartSingle);
  while (!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK))
    ;
  return ADC_DataSingleGet(ADC0);
};

/******************************************************************************
 * @brief  Timer/RTC Functions
 *
 *****************************************************************************/

void initTimer()
{
  USTIMER_Init();
  RTCDRV_Init();
  RTCDRV_AllocateTimer(&id);
};

uint32_t millis()
{
  uint64_t ticks = RTCDRV_GetWallClockTicks64();
  uint32_t milliseconds = RTCDRV_TicksToMsec(ticks);
  return (milliseconds);
};

void delay(unsigned int n)
{
  USTIMER_Delay(n * 1000);
};

void triggerEvent(uint32_t timeout, RTCDRV_Callback_t callback)
{
  RTCDRV_StartTimer(id, rtcdrvTimerTypeOneshot, timeout, callback, NULL);
}

/******************************************************************************
 * @brief  I2C Functions
 *
 *****************************************************************************/

void initI2C(byte mode)
{
  CMU_ClockEnable(cmuClock_I2C0, true);

  // Use ~400khz SCK
  i2cInit.freq = I2C_FREQ_FAST_MAX;

  I2C0->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
  if (mode == 0)
  {
    // Using PC10 (SDA) and PC11 (SCL)
    GPIO_PinModeSet(gpioPortC, 10, gpioModeWiredAndPullUpFilter, 1);
    GPIO_PinModeSet(gpioPortC, 11, gpioModeWiredAndPullUpFilter, 1);
    // Internal I2C interface (SCL_PC11 & SDA_PC10)
    I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SDALOC_MASK)) | I2C_ROUTELOC0_SDALOC_LOC15;
    I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SCLLOC_MASK)) | I2C_ROUTELOC0_SCLLOC_LOC15;
  }
  else
  {
    // Using PB11 (SDA) and PF6 (SCL)
    GPIO_PinModeSet(gpioPortB, 11, gpioModeWiredAndPullUpFilter, 1);
    GPIO_PinModeSet(gpioPortF, 6, gpioModeWiredAndPullUpFilter, 1);
    // External I2C interface (SCL_PF6 & SDA_PB11)
    I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SDALOC_MASK)) | I2C_ROUTELOC0_SDALOC_LOC6;
    I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SCLLOC_MASK)) | I2C_ROUTELOC0_SCLLOC_LOC29;
  }

  // Initializing the I2C
  I2C_Init(I2C0, &i2cInit);

  // Setting the status flags and index
  i2c_rxInProgress = false;
  i2c_startTx = false;
  i2c_rxBufferIndex = 0;
}

void deinitI2C()
{
  I2C_Reset(I2C0);
}

int transferI2C(uint16_t device_addr, uint8_t cmd_array[], uint8_t data_array[], uint16_t cmd_len, uint16_t data_len, uint8_t flag)
{
  uint16_t pre_time, cur_time, timeout = 500;
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;

  // Initialize I2C transfer
  I2C_TransferReturn_TypeDef result;
  i2cTransfer.addr = device_addr;
  i2cTransfer.flags = flag;
  i2cTransfer.buf[0].data = cmd_array;
  i2cTransfer.buf[0].len = cmd_len;

  // Note that WRITE_WRITE this is tx2 data
  i2cTransfer.buf[1].data = data_array;
  i2cTransfer.buf[1].len = data_len;

  // Set up the transfer
  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Do it until the transfer is done
  cur_time = millis();
  while (result != i2cTransferDone)
  {
    pre_time = millis();
    result = I2C_Transfer(I2C0);
    if ((pre_time - cur_time > timeout) && (result != i2cTransferDone))
    {
      for (byte i = 0; i < i2cTransfer.buf[1].len; i++)
      {
        // flush data buffer
        i2cTransfer.buf[1].data[i] = 0;
      }
      return I2C_ERR;
    }
  }
  return 0;
}

/******************************************************************************
 * @brief  SPI Functions
 * TODO
 *
 *****************************************************************************/

// TODO

/******************************************************************************
 * @brief  Maths Functions
 * Maths functions for Arduino, etc.
 *
 *****************************************************************************/

// Maps value to a range, used in analogWrite
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/******************************************************************************
 * @brief  Helper Functions
 * Generic functions for logging, etc.
 *
 *****************************************************************************/

char *insertInt(std::string str, int i)
{
  str += std::to_string(i);
  int n = str.length();

  // declaring character array
  char *char_array = new char[n + 1];

  // copying the contents of the
  // string to char array
  strcpy(char_array, str.c_str());
  return char_array;
}