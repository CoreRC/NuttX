/****************************************************************************************************
 * configs/stm32f746g-disco/src/stm32f746g-disco.h
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************************************/

#ifndef __CONFIGS_STM32F746G_DISCO_SRC_STM32F746G_DISCO__H
#define __CONFIGS_STM32F746G_DISCO_SRC_STM32F746G_DISCO__H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* STM32F736G Discovery GPIOs ***********************************************************************/
/* The STM32F746G-DISCO board has numerous LEDs but only one, LD1 located near the reset button, that
 * can be controlled by software (LD2 is a power indicator, LD3-6 indicate USB status, LD7 is
 * controlled by the ST-Link).
 *
 * LD1 is controlled by PI1 which is also the SPI2_SCK at the Arduino interface. One end of LD1 is
 * grounded so a high output on PI1 will illuminate the LED.
 */

#define GPIO_LD1           (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTI | GPIO_PIN1)

/* Pushbutton B1, labelled "User", is connected to GPIO PI11.  A high value will be sensed when the
 * button is depressed. Note that the EXTI interrupt is configured.
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER      (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTI | GPIO_PIN11)

/* Sporadic scheduler instrumentation. This configuration has been used for evaluating the NuttX
 * sporadic scheduler.  In this evaluation, two GPIO outputs are used.  One indicating the priority
 * (high or low) of the sporadic thread and one indicating where the thread is running or not.
 *
 * There is nothing special about the pin selections:
 *
 *   Arduino D2 PG6 - Indicates priority
 *   Arduino D4 PG7 - Indicates that the thread is running
 */

#define GPIO_SCHED_HIGHPRI (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTG | GPIO_PIN6)
#define GPIO_SCHED_RUNNING (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTG | GPIO_PIN7)

#define GPIO_LCD_DISP     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN12)

#define GPIO_LCD_BL       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTK|GPIO_PIN3)

#define GPIO_TS_INT     (GPIO_INPUT|GPIO_EXTI|GPIO_SPEED_50MHz|GPIO_PORTI|GPIO_PIN13)
#define TS_INT_EXTI_IRQn STM32_IRQ_EXTI1510

#define FT5x06_I2C_ADDRESS  0x38

/* SD card detect pin: PC13   (line is pulled up on board) */
#define GPIO_SDIO_NCD      (GPIO_INPUT|GPIO_EXTI|GPIO_SPEED_50MHz|GPIO_PORTC|GPIO_PIN13)

#define SDIO_SLOTNO 0
#define SDIO_MINOR 0

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************************************/

int stm32_bringup(void);

/****************************************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/****************************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f746g-disco board.
 *
 ****************************************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************************************
 * Name: arch_sporadic_initialize
 *
 * Description:
 *   This configuration has been used for evaluating the NuttX sporadic scheduler.
 *
 ****************************************************************************************************/

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
void arch_sporadic_initialize(void);
#endif

/****************************************************************************

 * Name: stm32_enablefmc
 *
 * Description:
 *  enable clocking to the FMC module
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_FMC
void stm32_enablefmc(void);
#endif

/****************************************************************************

 * Name: stm32_disablefmc
 *
 * Description:
 *  disable clocking to the FMC module
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_FMC
void stm32_disablefmc(void);
#endif

/****************************************************************************
 * Name: stm32_ft5x06_register
 *
 * Description:
 *   Register the FT5x06 touch panel driver
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_FT5336
int stm32_ft5x06_register(void);
#endif

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_STM32F7_SDMMC1)
int stm32_sdio_initialize(void);
#endif

/* Do we need to register I2C drivers on behalf of the I2C tool? */

#define HAVE_I2CTOOL 1
#if !defined(CONFIG_SYSTEM_I2CTOOL) || !defined(CONFIG_I2C_DRIVER)
#  undef HAVE_I2CTOOL
#endif

#define HAVE_SDIO 1
#define HAVE_NCD 1
#endif /* __ASSEMBLY__ */

#endif /* __CONFIGS_STM32F746G_DISCO_SRC_STM32F746G_DISCO_H */
