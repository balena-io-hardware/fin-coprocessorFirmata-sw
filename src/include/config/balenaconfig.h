#ifndef BALENACONFIG_H
#define BALENACONFIG_H

#define BSP_CLK_LFXO_PRESENT                          (1)
#define BSP_CLK_HFXO_PRESENT                          (1)
#define BSP_CLK_LFXO_INIT                              CMU_LFXOINIT_DEFAULT
#define BSP_CLK_LFXO_CTUNE                            (32U)
#define BSP_CLK_LFXO_FREQ                             (32768U)
#define BSP_CLK_HFXO_FREQ                             (38400000UL)
#define BSP_CLK_HFXO_CTUNE                            (-1)
#define BSP_CLK_HFXO_INIT                              CMU_HFXOINIT_DEFAULT
#define BSP_CLK_HFXO_CTUNE_TOKEN                      (0)
#define BSP_DCDC_INIT                                  EMU_DCDCINIT_DEFAULT
#define DEVKIT_VCOM_ENABLE_PIN                        (5U)
#define DEVKIT_VCOM_ENABLE_PORT                       (gpioPortA)

#endif // BALENACONFIG_H